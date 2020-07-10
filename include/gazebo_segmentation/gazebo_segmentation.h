#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <gazebo/rendering/Scene.hh>

namespace gazebo {

#define LOGICAL_CAMERA_NAME "logical"
#define COLOR_CAMERA_NAME   "color"
#define DEPTH_CAMERA_NAME   "depth"
#define SEGM_CAMERA_TOPIC   "segmentation"
#define SEGM_PUB_FREQ_HZ     60

#define DEPTH_CAM_SCALE 0.001

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
  void onDepthFrame();

private:

	// Sensors
  sensors::LogicalCameraSensorPtr logic_cam_;
  rendering::CameraPtr            color_cam_;
  rendering::DepthCameraPtr       depth_cam_;

  // Image storage
  std::vector<uint16_t> segmentation_map_;
  std::vector<uint16_t> depth_map_;

  // Gazebo messaging
  transport::NodePtr       transport_node_;
  transport::SubscriberPtr logic_cam_sub_;
  event::ConnectionPtr     color_cam_conn_;
  event::ConnectionPtr     depth_cam_conn_;

  // ROS messaging
  ros::NodeHandle *rosnode_;
  image_transport::ImageTransport *it_node_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  image_transport::CameraPublisher color_pub_, segmentation_pub_;//, depth_pub_;

  // Params
  std::string robot_namespace_;
  float       clip_near_;
  float       clip_far_;

  // Buffers
  rendering::ScenePtr        scene_;
  physics::WorldPtr          world_;
  std::vector<Ogre::Entity*> segmentation_objects_;
};

}
