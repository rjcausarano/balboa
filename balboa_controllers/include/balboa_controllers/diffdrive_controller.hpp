// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

#ifndef BALBOA_CONTROLLERS__DIFFDRIVE_CONTROLLER_HPP_
#define BALBOA_CONTROLLERS__DIFFDRIVE_CONTROLLER_HPP_

namespace balboa_controllers
{
class DiffdriveController : public rclcpp::Node
{
public:
  /// \brief Constructor
  DiffdriveController();

private:
  void OnMsgReceived(geometry_msgs::msg::Twist::SharedPtr msg);

  // Encoder parameters
  double encoder_resolution_;
  double wheel_circumference_;

  geometry_msgs::msg::Twist msg_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

};

}  // namespace balboa_controllers

#endif  // BALBOA_CONTROLLERS__DIFFDRIVE_CONTROLLER_HPP_
