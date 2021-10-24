cmake_minimum_required(VERSION 3.5)

project(balboa_drivers)

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
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

#### Libraries

set(dependencies
  geometry_msgs
  rclcpp
)

add_library(diffdrive_lib SHARED)
target_sources(
  diffdrive_lib
  PRIVATE
    src/diffdrive.cpp
)
target_include_directories(diffdrive_lib PUBLIC include)
ament_target_dependencies(diffdrive_lib
  ${dependencies}
)

set(libraries_names
  diffdrive_lib
)

#### Executables

# Diffdrive controller
add_executable(diffdrive_node)
target_sources(
  diffdrive_node
  PRIVATE
    src/diffdrive_node.cpp
)
target_link_libraries(diffdrive_node diffdrive_lib)

set(executables_names
  diffdrive_node
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

install(DIRECTORY launch
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
