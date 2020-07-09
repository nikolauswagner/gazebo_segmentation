#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/msgs/logical_camera_image.pb.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <gazebo_segmentation/NewMOC.h>
#include <gazebo/rendering/Scene.hh>

namespace gazebo {

#define COLOR_CAMERA_NAME   "color"
#define LOGICAL_CAMERA_NAME "logical"
#define SEGM_CAMERA_TOPIC   "segmentation"
#define SEGM_PUB_FREQ_HZ     60

/// \brief Segmentation plugin
class SegmentationPlugin : public ModelPlugin {
public:
  /// \brief Constructor.
  SegmentationPlugin();

  /// \brief Destructor.
  ~SegmentationPlugin();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  void onLogicFrame(ConstLogicalCameraImagePtr &_msg);

  void onColorFrame();

private:

  sensors::LogicalCameraSensorPtr logic_cam_;
  rendering::CameraPtr color_cam_;

  std::vector<uint16_t> segmentation_map_;

  transport::NodePtr transport_node_;
  transport::SubscriberPtr logic_cam_sub_;
  event::ConnectionPtr color_cam_conn_;

  ros::NodeHandle *rosnode_;
  image_transport::ImageTransport *it_node_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  image_transport::CameraPublisher color_pub_, segmentation_pub_;

  rendering::ScenePtr scene_;
  physics::WorldPtr world_;

  std::vector<Ogre::Entity*> segmentation_objects_;
  std::string robot_namespace_;
};

}
