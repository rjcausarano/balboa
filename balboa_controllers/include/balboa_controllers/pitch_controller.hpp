// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#ifndef BALBOA_CONTROLLERS__PITCH_CONTROLLER_HPP_
#define BALBOA_CONTROLLERS__PITCH_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace balboa_controllers
{
class PitchController : public rclcpp::Node{
public:
  /// \brief Constructor
  PitchController();

private:
  double desired_pitch_{0.0};  // in degrees
  double current_pitch_{0.0}; // in degrees

  const double Kp_{1};
  const double Kd_{1};
  const double Ki_{1};
  double error_sum_{0.0};
  double last_error_{0.0};

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_set_point_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Mutex
  std::mutex mutex_;
  double DoPid();
  void CmdVelCallback();
  void OnImuUpdate(sensor_msgs::msg::Imu::SharedPtr msg);
  void OnSetPitch(std_msgs::msg::Float64::SharedPtr msg);
};

}  // namespace balboa_controllers

#endif  // BALBOA_CONTROLLERS__PITCH_CONTROLLER_HPP_
