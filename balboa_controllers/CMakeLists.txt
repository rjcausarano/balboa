cmake_minimum_required(VERSION 3.5)

project(balboa_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

#### Libraries

set(dependencies
  sensor_msgs
  std_msgs
  geometry_msgs
  rclcpp
)

add_library(pitch_controller_lib SHARED)
target_sources(
  pitch_controller_lib
  PRIVATE
    src/pitch_controller.cpp
)
target_include_directories(pitch_controller_lib PUBLIC include)
ament_target_dependencies(pitch_controller_lib
  ${dependencies}
)

add_library(velocity_controller_lib SHARED)
target_sources(
  velocity_controller_lib
  PRIVATE
    src/velocity_controller.cpp
)
target_include_directories(velocity_controller_lib PUBLIC include)
ament_target_dependencies(velocity_controller_lib
  ${dependencies}
)

set(libraries_names
  pitch_controller_lib
  velocity_controller_lib
)

#### Executables

# Pitch controller
add_executable(pitch_controller_node)
target_sources(
  pitch_controller_node
  PRIVATE
    src/pitch_controller_node.cpp
)
target_link_libraries(pitch_controller_node pitch_controller_lib)

# Velocity controller
add_executable(velocity_controller_node)
target_sources(
  velocity_controller_node
  PRIVATE
    src/velocity_controller_node.cpp
)
target_link_libraries(velocity_controller_node velocity_controller_lib)

set(executables_names
  pitch_controller_node
  velocity_controller_node
)

#### Install

install(TARGETS ${libraries_names}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executables_names}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${libraries_names})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
