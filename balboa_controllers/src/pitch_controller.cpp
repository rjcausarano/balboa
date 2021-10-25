// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include "balboa_controllers/pitch_controller.hpp"

namespace balboa_controllers
{
PitchController::PitchController()
: rclcpp::Node("pitch_controller_node")
{
  Kp_ = declare_parameter("Kp").get<double>();
  Kd_ = declare_parameter("Kd").get<double>();
  Ki_ = declare_parameter("Ki").get<double>();
  max_linear_ = declare_parameter("max_linear").get<double>();
  double cmd_vel_publish_rate = declare_parameter("publish_rate").get<double>();

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SystemDefaultsQoS());

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SystemDefaultsQoS(),
    std::bind(&PitchController::OnImuUpdate, this, std::placeholders::_1));

  pitch_set_point_sub_ = create_subscription<std_msgs::msg::Float64>(
    "pitch_set_point", rclcpp::SystemDefaultsQoS(),
    std::bind(&PitchController::OnSetPitch, this, std::placeholders::_1));

  const std::chrono::milliseconds timeout{(int)(1/cmd_vel_publish_rate)*1000};  // cmd_vel publish rate
  control_timer_ = create_timer(
    this,
    this->get_clock(),
    timeout,
    std::bind(&PitchController::CmdVelCallback, this));
}

void PitchController::CmdVelCallback(){
  double pid_correction = DoPid();
  if(pid_correction > max_linear_) pid_correction = max_linear_;
  else if(pid_correction < -max_linear_) pid_correction = -max_linear_;
  cmd_vel_msg_.linear.x = pid_correction;
  cmd_vel_pub_->publish(cmd_vel_msg_);
}

double PitchController::DoPid(){
  double error = desired_pitch_ - current_pitch_;
  error_sum_ += error;
  double error_diff = error - last_error_;
  double p_correction = Kp_ * error;
  double i_correction = Ki_ * error_sum_;
  double d_correction = Kd_ * error_diff;
  last_error_ = error;
  return p_correction + i_correction + d_correction;
}

void PitchController::OnImuUpdate(sensor_msgs::msg::Imu::SharedPtr msg){
  std::lock_guard<std::mutex> lock{mutex_};
  current_pitch_ = std::asin(2 * (msg->orientation.w*msg->orientation.y - msg->orientation.x*msg->orientation.z));
  current_pitch_ *= 180 / M_PI;
}

void PitchController::OnSetPitch(std_msgs::msg::Float64::SharedPtr msg){
  std::lock_guard<std::mutex> lock{mutex_};
  desired_pitch_ = msg->data;
}

}  // namespace balboa_controllers
