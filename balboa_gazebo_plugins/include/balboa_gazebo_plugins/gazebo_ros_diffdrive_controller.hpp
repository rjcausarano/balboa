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
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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
  double wheel_radius_{1.0};
  double wheel_distance_{1.0};
  int encoder_resolution_{1};

  // Mutex to protect variables written from different threads
  std::mutex linear_mutex_;
  std::mutex angular_mutex_;

  rclcpp::TimerBase::SharedPtr vels_timer_;

  double desired_linear_{0.0};
  double desired_angular_{0.0};

  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};
  sensor_msgs::msg::JointState joint_state_msg_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr linear_sub_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angular_sub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_{nullptr};

  gazebo::physics::JointPtr left_wheel_joint_{nullptr};
  gazebo::physics::JointPtr right_wheel_joint_{nullptr};

  void ApplyVelsCallback();
  void OnLinearReceived(std_msgs::msg::Float64::SharedPtr msg);
  void OnAngularReceived(std_msgs::msg::Float64::SharedPtr msg);
};

}  // namespace balboa_gazebo_plugins

#endif  // BALBOA_GAZEBO_PLUGINS__GAZEBO_DIFFDRIVE_CONTROLLER_HPP_
