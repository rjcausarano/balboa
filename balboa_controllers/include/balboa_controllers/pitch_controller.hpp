// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#ifndef BALBOA_CONTROLLERS__PITCH_CONTROLLER_HPP_
#define BALBOA_CONTROLLERS__PITCH_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

namespace balboa_controllers
{
class PitchController : public rclcpp::Node{
public:
  /// \brief Constructor
  PitchController();

private:
  double desired_pitch_{0.0};  // in degrees
  double current_pitch_{0.0}; // in degrees
  double current_roll_{0.0};

  double Kp_{0.001};
  double Kd_{1};
  double Ki_{1};
  double max_linear_{0.0};
  double error_sum_{0.0};
  double last_error_{0.0};

  std_msgs::msg::Float64 linear_vel_msg_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_set_point_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr linear_vel_pub_;

  // Mutex
  std::mutex mutex_;
  double DoPid();
  void LinearVelCallback();
  void OnImuUpdate(sensor_msgs::msg::Imu::SharedPtr msg);
  void OnSetPitch(std_msgs::msg::Float64::SharedPtr msg);
};

}  // namespace balboa_controllers

#endif  // BALBOA_CONTROLLERS__PITCH_CONTROLLER_HPP_
