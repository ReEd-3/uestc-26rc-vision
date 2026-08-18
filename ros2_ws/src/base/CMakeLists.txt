cmake_minimum_required(VERSION 3.8)
project(base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)

add_executable(base_node 
  src/base.cpp
  src/uart_interact.cpp
  src/interact_cmds.cpp
)
target_include_directories(base_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(base_node rclcpp geometry_msgs std_msgs serial)

install(TARGETS base_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
