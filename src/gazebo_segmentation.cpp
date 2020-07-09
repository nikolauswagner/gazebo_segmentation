#include <gazebo_segmentation/gazebo_segmentation.h>

#include <sensor_msgs/fill_image.h>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/RayQuery.hh>
#include <gazebo/rendering/Conversions.hh>
#include <ignition/math.hh>
#include <gazebo_segmentation/NewMOC.h>

#include <stdlib.h>

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(SegmentationPlugin)

SegmentationPlugin::SegmentationPlugin() 
    : color_cam_(nullptr),
      logic_cam_(nullptr) {

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
  this->robot_namespace_ = "";
  if(_sdf->HasElement("robot_namespace")) {
    this->robot_namespace_ = _sdf->GetElement("robot_namespace")->Get<std::string>() + "/";
  }

  // Get Gazebo sensors
  sensors::SensorManager *sensor_mngr = sensors::SensorManager::Instance();
  this->logic_cam_ = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(
      sensor_mngr->GetSensor(this->robot_namespace_ + LOGICAL_CAMERA_NAME));
  this->color_cam_ = std::dynamic_pointer_cast<sensors::CameraSensor>(
      sensor_mngr->GetSensor(this->robot_namespace_ + COLOR_CAMERA_NAME))->Camera();
  if(!this->logic_cam_) {
    ROS_FATAL_STREAM("SegmentationPlugin: Missing logical camera!\n" <<
                     "-> (needs to be named 'logical')");
    return;
  }
  if(!this->color_cam_ || this->color_cam_->ImageFormat() != "RGB_INT8") {
    ROS_FATAL_STREAM("SegmentationPlugin: Missing color camera!\n" <<
                     "-> (needs to be named 'color')" << 
                     "-> (needs to have format 'RGB_INT8')");
    return;
  }

  // Setup Gazebo transport
  this->world_ = _parent->GetWorld();
  this->transport_node_ = transport::NodePtr(new transport::Node());
  #if GAZEBO_MAJOR_VERSION >= 9
    this->transport_node_->Init(_parent->GetWorld()->Name());
  #else
    this->transport_node_->Init(_parent->GetWorld()->GetName());
  #endif
  this->color_cam_conn_ = this->color_cam_->ConnectNewImageFrame(
      std::bind(&SegmentationPlugin::onColorFrame, this));

  // Subscribe to Gazebo publishers
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

  // Setup ROS publishers
  this->rosnode_ = new ros::NodeHandle(this->GetHandle());
  this->camera_info_manager_.reset(
      new camera_info_manager::CameraInfoManager(*this->rosnode_, 
                                                 this->GetHandle()));
  this->it_node_ = new image_transport::ImageTransport(*this->rosnode_);
  this->color_pub_ = this->it_node_->advertiseCamera("camera/color/image_raw", 2);
  this->segmentation_pub_ = this->it_node_->advertiseCamera("camera/segmentation/image_raw", 2);

  // Setup variables
  this->scene_      = this->color_cam_->GetScene();
}

void SegmentationPlugin::onLogicFrame(ConstLogicalCameraImagePtr &_msg) {
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

void SegmentationPlugin::onColorFrame() {
  std::cout << "Starting at: " << this->world_->SimTime().sec << std::endl;
  auto collidor = new ::Collision::CollisionTools();
  for(auto obj : this->segmentation_objects_) {
    collidor->register_entity(obj);
  }

  ignition::math::Vector3d origin, dir;
  for(uint16_t i_h = 0; i_h < this->color_cam_->ImageHeight(); i_h++) {
    for(uint16_t i_w = 0; i_w < this->color_cam_->ImageWidth(); i_w++) {
      uint16_t id = 0;
      this->color_cam_->CameraToViewportRay(i_w, i_h, origin, dir);
      Ogre::Ray ray(rendering::Conversions::Convert(origin), 
                    rendering::Conversions::Convert(dir));
//      auto collision = collidor->check_ray_collision(ray, 
//                                                     Ogre::SceneManager::ENTITY_TYPE_MASK, 
//                                                     nullptr, 10.0, true);
//      if(collision.collided)
//        id = this->scene_->GetVisual(Ogre::any_cast<std::string>(
//            collision.entity->getUserObjectBindings().getUserAny()))->GetId();

      this->segmentation_map_[this->color_cam_->ImageWidth() * i_h + i_w] = id * 50; // TODO: remove factor
    }
  }

  sensor_msgs::Image image_msg;
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = this->world_->SimTime();
#else
  common::Time current_time = this->world_->GetSimTime();
#endif
  image_msg.header.frame_id   = this->robot_namespace_ + COLOR_CAMERA_NAME;
  image_msg.header.stamp.sec  = current_time.sec;
  image_msg.header.stamp.nsec = current_time.nsec;

  // copy from simulation image to ROS msg
  fillImage(image_msg, 
            sensor_msgs::image_encodings::RGB8,
            this->color_cam_->ImageHeight(), this->color_cam_->ImageWidth(),
            this->color_cam_->ImageDepth() * this->color_cam_->ImageWidth(),
            reinterpret_cast<const void*>(this->color_cam_->ImageData()));

  // publish to ROS
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
  std::cout << "Published at: " << this->world_->SimTime().sec << std::endl;
}

}