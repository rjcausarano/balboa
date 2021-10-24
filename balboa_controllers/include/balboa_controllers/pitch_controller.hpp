// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#ifndef BALBOA_CONTROLLERS__PITCH_CONTROLLER_HPP_
#define BALBOA_CONTROLLERS__PITCH_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <geometry_msgs/msg/twist.hpp>

namespace balboa_controllers
{
class PitchController : public rclcpp::Node{
public:
  /// \brief Constructor
  PitchController();

private:
  double desired_pitch_{0.0};
  double current_pitch_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Mutex
  std::mutex mutex_;
  void CmdVelCallback();
  void OnImuUpdate(sensor_msgs::msg::Imu::SharedPtr msg);
};

}  // namespace balboa_controllers

#endif  // BALBOA_CONTROLLERS__PITCH_CONTROLLER_HPP_
