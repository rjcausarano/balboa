// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include "balboa_controllers/pitch_controller.hpp"

namespace balboa_controllers
{
PitchController::PitchController()
: rclcpp::Node("pitch_controller_node")
{
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SystemDefaultsQoS());

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SystemDefaultsQoS(),
    std::bind(&PitchController::OnImuUpdate, this, std::placeholders::_1));

  const std::chrono::milliseconds timeout{10};  // update rate of 100 Hz
  control_timer_ = create_timer(
    this,
    this->get_clock(),
    timeout,
    std::bind(&PitchController::CmdVelCallback, this));
}

void PitchController::CmdVelCallback(){
    std::cout << "CmdVelCallback" << std::endl;
}


void PitchController::OnImuUpdate(sensor_msgs::msg::Imu::SharedPtr msg){
    std::lock_guard<std::mutex> lock{mutex_};
    current_pitch_ = std::asin(2 * (msg->orientation.w*msg->orientation.y - msg->orientation.x*msg->orientation.z));
    std::cout << current_pitch_ * 180 / M_PI << std::endl;
}


}  // namespace balboa_controllers
