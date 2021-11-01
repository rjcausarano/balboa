// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include "balboa_controllers/pitch_controller.hpp"
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace balboa_controllers
{
PitchController::PitchController()
: rclcpp::Node("pitch_controller_node")
{
  Kp_ = declare_parameter("Kp").get<double>();
  Kd_ = declare_parameter("Kd").get<double>();
  Ki_ = declare_parameter("Ki").get<double>();
  max_linear_ = declare_parameter("max_linear").get<double>();
  double linear_vel_publish_rate = declare_parameter("publish_rate").get<double>();

  linear_vel_pub_ = create_publisher<std_msgs::msg::Float64>(
    "__linear", rclcpp::SystemDefaultsQoS());

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SystemDefaultsQoS(),
    std::bind(&PitchController::OnImuUpdate, this, std::placeholders::_1));

  pitch_set_point_sub_ = create_subscription<std_msgs::msg::Float64>(
    "pitch_set_point", rclcpp::SystemDefaultsQoS(),
    std::bind(&PitchController::OnSetPitch, this, std::placeholders::_1));

  control_timer_ = create_timer(
    this,
    this->get_clock(),
    std::chrono::duration<double>(1/linear_vel_publish_rate),
    std::bind(&PitchController::LinearVelCallback, this));
}

void PitchController::LinearVelCallback(){
  if(abs(current_pitch_) > 90){
    linear_vel_msg_.data = 0.0;  // If more than 90 degrees, just stop.
  } else {
    double pid_correction = DoPid();
    if(pid_correction > max_linear_) pid_correction = max_linear_;
    else if(pid_correction < -max_linear_) pid_correction = -max_linear_;
    linear_vel_msg_.data = pid_correction;
  }
  linear_vel_pub_->publish(linear_vel_msg_);
}

double PitchController::DoPid(){
  double error = current_pitch_ - desired_pitch_;
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
  tf2::Vector3 up{0.0, 0.0, 1.0};
  tf2::Quaternion q{
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z,
    msg->orientation.w};
  tf2::Vector3 direction = tf2::quatRotate(q, up);
  tf2::Matrix3x3 m{q};
  double roll, yaw;
  m.getRPY(roll, current_pitch_, yaw);
  current_pitch_ *= 180 / M_PI;  // convert to degrees
  if(tf2IsNegative(direction.z())){
    current_pitch_ = copysign(1.0, current_pitch_) * (180 - abs(current_pitch_));
  }
}

void PitchController::OnSetPitch(std_msgs::msg::Float64::SharedPtr msg){
  std::lock_guard<std::mutex> lock{mutex_};
  desired_pitch_ = msg->data;
}

}  // namespace balboa_controllers
