// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#ifndef BALBOA_CONTROLLERS__PITCH_CONTROLLER_HPP_
#define BALBOA_CONTROLLERS__PITCH_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace balboa_controllers
{
class VelocityController : public rclcpp::Node{
public:
  /// \brief Constructor
  VelocityController();

private:
  double desired_linear_{0.0};
  double desired_angular_{0.0};
  double wheel_radius_{0.0};
  double current_linear_{0.0};

  double Kp_{0.0};
  double Kd_{0.0};
  double Ki_{0.0};
  double max_angle_{0.0};
  double error_sum_{0.0};
  double last_error_{0.0};

  std_msgs::msg::Float64 pitch_msg_;
  std_msgs::msg::Float64 angular_vel_msg_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angular_vel_pub_;

  // Mutex
  std::mutex mutex_;
  // double DoPid();  // velocity control PID
  void VelUpdate();
  void CmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg);
  void JointStatesCallback(sensor_msgs::msg::JointState::SharedPtr msg);
};

}  // namespace balboa_controllers

#endif  // BALBOA_CONTROLLERS__PITCH_CONTROLLER_HPP_
