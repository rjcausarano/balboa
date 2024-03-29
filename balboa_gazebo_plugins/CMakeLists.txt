cmake_minimum_required(VERSION 3.5)

project(balboa_gazebo_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# Add libraries
add_library(gazebo_ros_diffdrive_controller SHARED
  src/gazebo_ros_diffdrive_controller.cpp
)

add_library(gazebo_ros_helpers SHARED
  src/gazebo_ros_helpers.cpp
)

# Linking
## gazebo_ros_helpers
target_include_directories(gazebo_ros_helpers PUBLIC include)
ament_target_dependencies(gazebo_ros_helpers
  "gazebo_dev"
  "gazebo_ros"
)
ament_export_libraries(gazebo_ros_helpers)

## gazebo_ros_diffdrive_controller
target_include_directories(gazebo_ros_diffdrive_controller PUBLIC include)
ament_target_dependencies(gazebo_ros_diffdrive_controller
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_diffdrive_controller)
target_link_libraries(gazebo_ros_diffdrive_controller gazebo_ros_helpers)

# Exports
ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(rclcpp)

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    gazebo_ros_diffdrive_controller
    gazebo_ros_helpers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
