// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)

#include <memory>

#include "balboa_controllers/diffdrive_controller.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<balboa_controllers::DiffdriveController>());
  rclcpp::shutdown();
  return 0;
}
