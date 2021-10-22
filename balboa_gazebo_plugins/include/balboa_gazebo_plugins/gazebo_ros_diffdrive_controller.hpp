// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#ifndef BALBOA_GAZEBO_PLUGINS__GAZEBO_DIFFDRIVE_CONTROLLER_HPP_
#define BALBOA_GAZEBO_PLUGINS__GAZEBO_DIFFDRIVE_CONTROLLER_HPP_

#include "gazebo_ros/utils.hpp"
#include "gazebo/common/Plugin.hh"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/physics/World.hh>
#include <geometry_msgs/msg/twist.hpp>
#include <balboa_gazebo_plugins/gazebo_ros_helpers.hpp>

namespace balboa_gazebo_plugins
{
/// Plugin to attach to a gazebo IMU sensor and publish ROS message as output
class GazeboDiffdriveController : public gazebo::ModelPlugin
{
public:
  /// Constructor.
  GazeboDiffdriveController();
  /// Destructor.
  virtual ~GazeboDiffdriveController();

protected:
  /// Called by Gazebo when the plugin is loaded.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_{nullptr};

  gazebo::physics::JointPtr left_wheel_joint_;
  gazebo::physics::JointPtr right_wheel_joint_;

  void OnMsgReceived(geometry_msgs::msg::Twist::SharedPtr msg);

};

}  // namespace balboa_gazebo_plugins

#endif  // BALBOA_GAZEBO_PLUGINS__GAZEBO_DIFFDRIVE_CONTROLLER_HPP_
