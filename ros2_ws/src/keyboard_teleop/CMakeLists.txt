cmake_minimum_required(VERSION 3.8)
project(keyboard_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(keyboard_teleop_node
  src/keyboard_teleop_node.cpp
)

ament_target_dependencies(keyboard_teleop_node
  rclcpp
  geometry_msgs
)

install(
  TARGETS keyboard_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
