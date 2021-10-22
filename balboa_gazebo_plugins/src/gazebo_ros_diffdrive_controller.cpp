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
  ros_node_ = gazebo_ros::Node::Get(sdf);

  sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(&GazeboDiffdriveController::OnMsgReceived, this, std::placeholders::_1));

  left_wheel_joint_ = model->GetJoint("left_wheel_joint");
  right_wheel_joint_ = model->GetJoint("right_wheel_joint");
}

void GazeboDiffdriveController::OnMsgReceived(geometry_msgs::msg::Twist::SharedPtr msg){
  (void)msg;
  left_wheel_joint_->SetVelocity(0, 1.0);
  right_wheel_joint_->SetVelocity(0, 1.0);
  std::cout << "HELLO SIMULATED WORLD" << std::endl;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboDiffdriveController)
}  // namespace balboa_gazebo_plugins
