#include <gazebo_segmentation/gazebo_segmentation.h>

#include <gazebo_segmentation/NewMOC.h>
#include <sensor_msgs/fill_image.h>
#include <gazebo/rendering/Conversions.hh>
#include <thread>

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(SegmentationPlugin)

SegmentationPlugin::SegmentationPlugin() 
    : logic_cam_(nullptr),
      color_cam_(nullptr),
      depth_cam_(nullptr),
      clip_near_(0.0),
      clip_far_(10.0),
      robot_namespace_("") {

}

SegmentationPlugin::~SegmentationPlugin() {
  this->rosnode_->shutdown();
}

void SegmentationPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  // Validity checking
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized," <<
                     "unable to load plugin. Load the Gazebo system plugin " <<
                     "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Setup params
  if(_sdf->HasElement("robot_namespace")) {
    this->robot_namespace_ = 
        _sdf->GetElement("robot_namespace")->Get<std::string>() + "/";
  }

  // Get Gazebo sensors
  sensors::SensorManager *sensor_mngr = sensors::SensorManager::Instance();
  this->logic_cam_ = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(
      sensor_mngr->GetSensor(this->robot_namespace_ + LOGICAL_CAMERA_NAME));
  auto color_sensor = std::dynamic_pointer_cast<sensors::CameraSensor>(
      sensor_mngr->GetSensor(this->robot_namespace_ + COLOR_CAMERA_NAME));
  auto depth_sensor = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(
      sensor_mngr->GetSensor(this->robot_namespace_ + DEPTH_CAMERA_NAME));
  if(!this->logic_cam_) {
    ROS_FATAL_STREAM("SegmentationPlugin: Missing logical camera!\n" <<
                     "-> needs to be named 'logical'");
    return;
  }
  if(!color_sensor || color_sensor->Camera()->ImageFormat() != "RGB_INT8") {
    ROS_FATAL_STREAM("SegmentationPlugin: Missing color camera!\n" <<
                     "-> needs to be named 'color'" << 
                     "-> needs to have format 'RGB_INT8'");
    return;
  }
  if(!depth_sensor) {
    ROS_FATAL_STREAM("SegmentationPlugin: Missing depth camera!\n" <<
                     "-> needs to be named 'depth'");
    return;
  }
  this->color_cam_ = color_sensor->Camera();
  this->depth_cam_ = depth_sensor->DepthCamera();
  if(this->depth_cam_->ImageWidth() != this->color_cam_->ImageWidth() ||
     this->depth_cam_->ImageHeight() != this->color_cam_->ImageHeight()) {
    ROS_FATAL_STREAM("SegmentationPlugin: Resolution mismatch!\n" <<
                     "-> Color: " << 
                     this->color_cam_->ImageWidth() << "*" << this->color_cam_->ImageWidth() <<
                     "-> Depth: " << 
                     this->depth_cam_->ImageWidth() << "*" << this->depth_cam_->ImageWidth());
    return;
  }
  this->clip_near_ = this->depth_cam_->NearClip();
  this->clip_far_  = this->depth_cam_->FarClip();

  // Setup Gazebo transport
  this->world_ = _parent->GetWorld();
  this->scene_ = this->color_cam_->GetScene();
  this->transport_node_ = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION >= 9
  this->transport_node_->Init(_parent->GetWorld()->Name());
#else
  this->transport_node_->Init(_parent->GetWorld()->GetName());
#endif
  this->color_cam_conn_ = this->color_cam_->ConnectNewImageFrame(
      std::bind(&SegmentationPlugin::onColorFrame, this));
  this->depth_cam_conn_ = this->depth_cam_->ConnectNewImageFrame(
      std::bind(&SegmentationPlugin::onDepthFrame, this));
  this->logic_cam_sub_ = this->transport_node_->Subscribe(this->logic_cam_->Topic(),
                                                          &SegmentationPlugin::onLogicFrame, 
                                                          this);
  // Setup segmentation map
  try {
    this->segmentation_map_.resize(this->color_cam_->ImageWidth() * this->color_cam_->ImageHeight());
  } catch (std::bad_alloc &e) {
    ROS_FATAL_STREAM("SegmentationPlugin: segmentation_map allocation failed: " << e.what());
    return;
  }
  // Setup depth map
  try {
    this->depth_map_.resize(this->depth_cam_->ImageWidth() * this->depth_cam_->ImageHeight());
  } catch (std::bad_alloc &e) {
    ROS_FATAL_STREAM("SegmentationPlugin: segmentation_map allocation failed: " << e.what());
    return;
  }

  // Setup ROS publishers
  this->rosnode_ = new ros::NodeHandle(this->GetHandle());
  this->camera_info_manager_.reset(
      new camera_info_manager::CameraInfoManager(*this->rosnode_, 
                                                 this->GetHandle()));
  this->it_node_ = new image_transport::ImageTransport(*this->rosnode_);
  this->color_pub_        = this->it_node_->advertiseCamera("camera/color/image_raw", 2);
  this->depth_pub_        = this->it_node_->advertiseCamera("camera/depth/image_raw", 2);
  this->segmentation_pub_ = this->it_node_->advertiseCamera("camera/segmentation/image_raw", 2);
}

inline sensor_msgs::CameraInfo cameraInfo(const sensor_msgs::Image& image, 
                                          const float horizontal_fov) {
  sensor_msgs::CameraInfo info_msg;

  info_msg.header = image.header;
  info_msg.height = image.height;
  info_msg.width  = image.width;

  float focal = 0.5 * image.width / tan(0.5 * horizontal_fov);

  info_msg.K[0] = focal;
  info_msg.K[4] = focal;
  info_msg.K[2] = info_msg.width * 0.5;
  info_msg.K[5] = info_msg.height * 0.5;
  info_msg.K[8] = 1.;

  info_msg.P[0]  = info_msg.K[0];
  info_msg.P[5]  = info_msg.K[4];
  info_msg.P[2]  = info_msg.K[2];
  info_msg.P[6]  = info_msg.K[5];
  info_msg.P[10] = info_msg.K[8];

  return info_msg;
}

void SegmentationPlugin::onLogicFrame(ConstLogicalCameraImagePtr &_msg) {
  // Store all objects to consider for segmentation map
  auto scene_mngr = this->scene_->OgreSceneManager();
  this->segmentation_objects_.clear();
  std::vector<std::string> visible_objects;
  for(auto model : _msg->model()) {
    visible_objects.push_back(model.name());
  }
  auto entities = scene_mngr->getMovableObjectIterator("Entity");
  while(entities.hasMoreElements()) {
    Ogre::Entity* entity = static_cast<Ogre::Entity*>(entities.getNext());
    for(auto vobj : visible_objects) {
      if(!entity->getName().rfind("VISUAL_" + vobj + "::", 0)) {
        this->segmentation_objects_.push_back(entity);
      }
    }
  }
}

void SegmentationPlugin::onDepthFrame() {
  const float *depth_data_float = this->depth_cam_->DepthData();
  for(uint32_t i = 0; i < this->depth_map_.size(); ++i) {
    // Check clipping and overflow
    if(depth_data_float[i] < this->clip_near_             ||
       depth_data_float[i] > this->clip_far_              ||
       depth_data_float[i] > DEPTH_CAM_SCALE * UINT16_MAX ||
       depth_data_float[i] < 0) {
      this->depth_map_[i] = 0;
    } else {
      this->depth_map_[i] = (uint16_t) (depth_data_float[i] / DEPTH_CAM_SCALE);
    }
  }
}

void SegmentationPlugin::onColorFrame() {
  // Register objects for collision detection
  auto collidor = new ::Collision::CollisionTools();
  for(auto obj : this->segmentation_objects_) {
    collidor->register_entity(obj);
  }

  // Store helper variables
  uint16_t img_height = this->color_cam_->ImageHeight();
  uint16_t img_width  = this->color_cam_->ImageWidth();
  uint32_t num_pixels = img_height * img_width;

  // Setup multithreading
  const size_t num_threads = std::thread::hardware_concurrency();
  std::vector<std::thread> threads(num_threads);
  for(uint8_t t = 0; t < num_threads; t++) {
    threads[t] = std::thread(std::bind(
      // Calculate segmentation map
      [&](const int i_beg, const int i_end, const uint8_t t) {
        ignition::math::Vector3d origin, dir;
        for(int i = i_beg; i < i_end; i++) {
          uint16_t i_h = i / img_width;
          uint16_t i_w = i % img_width;
          uint16_t id = 0;
          // Only calculate for pixels with depth info
          if(this->depth_map_[i]) {
            // Setup ray for collision check
            this->color_cam_->CameraToViewportRay(i_w, i_h, origin, dir);
            Ogre::Ray ray(rendering::Conversions::Convert(origin), 
                          rendering::Conversions::Convert(dir));
            // Do collision check
            auto collision = collidor->check_ray_collision(ray, 
                                                           Ogre::SceneManager::ENTITY_TYPE_MASK, 
                                                           nullptr, this->clip_far_, true);
            // Get ID of collided object
            if(collision.collided)
              id = this->scene_->GetVisual(Ogre::any_cast<std::string>(
                  collision.entity->getUserObjectBindings().getUserAny()))->GetId();
          }
          // Store ID if there was a collision
          this->segmentation_map_[i] = id;
        }
      },
    t * num_pixels / num_threads,
    (t + 1) == num_threads ? num_pixels : (t + 1) * num_pixels / num_threads,
    t));
  }
  // Do multithreading
  std::for_each(threads.begin(), threads.end(), 
                [](std::thread& x) {x.join();});

  // Setup ROS msg
  sensor_msgs::Image image_msg;
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = this->world_->SimTime();
#else
  common::Time current_time = this->world_->GetSimTime();
#endif
  image_msg.header.frame_id   = this->robot_namespace_ + COLOR_CAMERA_NAME;
  image_msg.header.stamp.sec  = current_time.sec;
  image_msg.header.stamp.nsec = current_time.nsec;

  // Publish color image
  fillImage(image_msg, 
            sensor_msgs::image_encodings::RGB8,
            this->color_cam_->ImageHeight(), this->color_cam_->ImageWidth(),
            this->color_cam_->ImageDepth() * this->color_cam_->ImageWidth(),
            reinterpret_cast<const void*>(this->color_cam_->ImageData()));
  auto camera_info_msg = cameraInfo(image_msg, 
                                    this->color_cam_->HFOV().Radian());
  this->color_pub_.publish(image_msg, camera_info_msg);

  // Publish segmentation map
  fillImage(image_msg,
            sensor_msgs::image_encodings::TYPE_16UC1,
            this->color_cam_->ImageHeight(), this->color_cam_->ImageWidth(),
            2 * this->color_cam_->ImageWidth(),
            reinterpret_cast<const void*>(this->segmentation_map_.data()));
  this->segmentation_pub_.publish(image_msg, camera_info_msg);

  // Publish depth map
  fillImage(image_msg,
            sensor_msgs::image_encodings::TYPE_16UC1,
            this->color_cam_->ImageHeight(), this->color_cam_->ImageWidth(),
            2 * this->color_cam_->ImageWidth(),
            reinterpret_cast<const void*>(this->depth_map_.data()));
  this->depth_pub_.publish(image_msg, camera_info_msg);
}

}