// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include "balboa_controllers/diffdrive_controller.hpp"

#include <string>
#include <vector>

namespace balboa_controllers
{
DiffdriveController::DiffdriveController()
: rclcpp::Node("diffdrive_controller_node")
{
  subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(&DiffdriveController::OnMsgReceived, this, std::placeholders::_1));
}

void DiffdriveController::OnMsgReceived(geometry_msgs::msg::Twist::SharedPtr msg){
  (void)msg;
  std::cout << "HELLO WORLD" << std::endl;
}

}  // namespace balboa_controllers
