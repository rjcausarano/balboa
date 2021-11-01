// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include "balboa_gazebo_plugins/gazebo_ros_diffdrive_controller.hpp"

#include <memory>

namespace balboa_gazebo_plugins
{
GazeboDiffdriveController::GazeboDiffdriveController() {}

GazeboDiffdriveController::~GazeboDiffdriveController() {}

void GazeboDiffdriveController::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  double vel_update_rate{0.0};
  utils::initialize(wheel_radius_, sdf, "wheel_radius", 1.0);
  utils::initialize(wheel_distance_, sdf, "wheel_distance", 1.0);
  utils::initialize(encoder_resolution_, sdf, "encoder_resolution", 1);
  utils::initialize(vel_update_rate, sdf, "vel_update_rate", 1);

  ros_node_ = gazebo_ros::Node::Get(sdf);

  linear_sub_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
    "__linear", rclcpp::SystemDefaultsQoS(),
    std::bind(&GazeboDiffdriveController::OnLinearReceived, this, std::placeholders::_1));

  angular_sub_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
    "__angular", rclcpp::SystemDefaultsQoS(),
    std::bind(&GazeboDiffdriveController::OnAngularReceived, this, std::placeholders::_1));

  joint_state_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SystemDefaultsQoS());

  vels_timer_ = rclcpp::create_timer(
    ros_node_,
    ros_node_->get_clock(),
    std::chrono::duration<double>(1/vel_update_rate),
    std::bind(&GazeboDiffdriveController::ApplyVelsCallback, this));
  std::string left_joint = "left_wheel_joint";
  std::string right_joint = "right_wheel_joint";
  left_wheel_joint_ = model->GetJoint(left_joint);
  right_wheel_joint_ = model->GetJoint(right_joint);
  joint_state_msg_.name = {left_joint, right_joint};
  joint_state_msg_.effort = {0.0, 0.0};
  joint_state_msg_.position = {0.0, 0.0};
  joint_state_msg_.velocity = {0.0, 0.0};
}

void GazeboDiffdriveController::OnLinearReceived(std_msgs::msg::Float64::SharedPtr msg){
  std::lock_guard<std::mutex> lock{linear_mutex_};
  desired_linear_ = msg->data;
}

void GazeboDiffdriveController::OnAngularReceived(std_msgs::msg::Float64::SharedPtr msg){
  std::lock_guard<std::mutex> lock{angular_mutex_};
  desired_angular_ = msg->data;
}

void GazeboDiffdriveController::ApplyVelsCallback(){
  double right_linear = desired_linear_ - desired_angular_ * wheel_distance_ / 2;
  double left_linear = desired_linear_ + desired_angular_ * wheel_distance_ / 2;
  left_wheel_joint_->SetParam("vel", 0, left_linear/wheel_radius_);
  left_wheel_joint_->SetParam("fmax", 0, 1.0);
  right_wheel_joint_->SetParam("vel", 0, right_linear/wheel_radius_);
  right_wheel_joint_->SetParam("fmax", 0, 1.0);
  joint_state_msg_.velocity[0] = left_wheel_joint_->GetParam("vel", 0);
  joint_state_msg_.velocity[1] = right_wheel_joint_->GetParam("vel", 0);
  joint_state_pub_->publish(joint_state_msg_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboDiffdriveController)
}  // namespace balboa_gazebo_plugins
