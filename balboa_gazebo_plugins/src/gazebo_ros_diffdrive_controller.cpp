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
  utils::initialize(wheel_radius_, sdf, "wheel_radius", 1.0);
  utils::initialize(wheel_distance_, sdf, "wheel_distance", 1.0);
  utils::initialize(encoder_resolution_, sdf, "encoder_resolution", 1);

  ros_node_ = gazebo_ros::Node::Get(sdf);

  sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(&GazeboDiffdriveController::OnMsgReceived, this, std::placeholders::_1));

  const std::chrono::milliseconds timeout{10};  // update rate of 100 Hz
  cmd_vel_timer_ = rclcpp::create_timer(
    ros_node_,
    ros_node_->get_clock(),
    timeout,
    std::bind(&GazeboDiffdriveController::ApplyVelsCallback, this));

  left_wheel_joint_ = model->GetJoint("left_wheel_joint");
  right_wheel_joint_ = model->GetJoint("right_wheel_joint");
}

void GazeboDiffdriveController::OnMsgReceived(geometry_msgs::msg::Twist::SharedPtr msg){
  std::lock_guard<std::mutex> lock{mutex_};
  desired_linear_ = msg->linear.x;
  desired_angular_ = msg->angular.z;
}

void GazeboDiffdriveController::ApplyVelsCallback(){
  double right_linear = desired_linear_ - desired_angular_ * wheel_distance_ / 2;
  double left_linear = desired_linear_ + desired_angular_ * wheel_distance_ / 2;
  right_wheel_joint_->SetParam("vel", 0, right_linear/wheel_radius_);
  right_wheel_joint_->SetParam("fmax", 0, 1000.0);
  left_wheel_joint_->SetParam("vel", 0, left_linear/wheel_radius_);
  left_wheel_joint_->SetParam("fmax", 0, 1000.0);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboDiffdriveController)
}  // namespace balboa_gazebo_plugins
