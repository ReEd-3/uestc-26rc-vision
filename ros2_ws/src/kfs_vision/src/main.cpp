#include <exception>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "kfs_vision/kfs_vision_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<kfs_vision::KfsVisionNode>();
    node->start();
    rclcpp::spin(node);
    node->stop();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "kfs_vision startup failed: " << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}

