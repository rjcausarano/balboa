// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include "balboa_controllers/velocity_controller.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace balboa_controllers
{
VelocityController::VelocityController()
: rclcpp::Node("velocity_controller_node")
{
  Kp_ = declare_parameter("Kp").get<double>();
  Kd_ = declare_parameter("Kd").get<double>();
  Ki_ = declare_parameter("Ki").get<double>();
  max_angle_ = declare_parameter("max_angle").get<double>();
  double publish_rate = declare_parameter("publish_rate").get<double>();

  angular_vel_pub_ = create_publisher<std_msgs::msg::Float64>(
    "__angular", rclcpp::SystemDefaultsQoS());

  pitch_pub_ = create_publisher<std_msgs::msg::Float64>(
    "pitch_set_point", rclcpp::SystemDefaultsQoS());

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(&VelocityController::CmdVelCallback, this, std::placeholders::_1));

  control_timer_ = create_timer(
    this,
    this->get_clock(),
    std::chrono::duration<double>(1/publish_rate),
    std::bind(&VelocityController::AngularVelUpdate, this));
}

void VelocityController::AngularVelUpdate(){
  angular_vel_msg_.data = desired_angular_;
  angular_vel_pub_->publish(angular_vel_msg_);
}

void VelocityController::CmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg){
  std::lock_guard<std::mutex> lock{mutex_};
  desired_linear_ = msg->linear.x;
  desired_angular_ = msg->angular.z;
}

}  // namespace balboa_controllers