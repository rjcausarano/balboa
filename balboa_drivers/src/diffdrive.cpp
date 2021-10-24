// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include "balboa_drivers/diffdrive.hpp"

#include <string>
#include <vector>

namespace balboa_drivers
{
Diffdrive::Diffdrive()
: rclcpp::Node("diffdrive_node")
{
  subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(&Diffdrive::OnMsgReceived, this, std::placeholders::_1));
}

void Diffdrive::OnMsgReceived(geometry_msgs::msg::Twist::SharedPtr msg){
  (void)msg;
  std::cout << "HELLO WORLD" << std::endl;
}

}  // namespace balboa_drivers
