#pragma once

#include <atomic>
#include <chrono>
#include <filesystem>
#include <optional>
#include <string>
#include <thread>

#include <custom_msgs/msg/kfs_target.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kfs/config.hpp"

namespace kfs_vision {

class KfsVisionNode : public rclcpp::Node {
 public:
  KfsVisionNode();
  ~KfsVisionNode() override;

  void start();
  void stop();

 private:
  void processingLoop() noexcept;
  void runPipeline();

  kfs::AppConfig app_config_;
  bool show_gui_ = false;
  std::chrono::milliseconds terminal_period_{1000};
  std::optional<std::filesystem::path> plane_config_output_path_;

  rclcpp::Publisher<custom_msgs::msg::KfsTarget>::SharedPtr target_publisher_;
  std::atomic_bool stop_requested_{false};
  std::thread processing_thread_;
};

}  // namespace kfs_vision

