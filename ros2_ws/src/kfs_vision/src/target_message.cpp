#include "kfs_vision/target_message.hpp"

#include <cmath>
#include <cstdint>

namespace kfs_vision {

std::optional<custom_msgs::msg::KfsTarget> makeTargetMessage(
    const kfs::SegmentationDetection& detection,
    const kfs::Measurement& measurement) {
  if (!measurement.plane || !measurement.pose) return std::nullopt;

  custom_msgs::msg::KfsTarget message;
  if (detection.class_id == 0) {
    message.color = custom_msgs::msg::KfsTarget::BLUE;
  } else if (detection.class_id == 1) {
    message.color = custom_msgs::msg::KfsTarget::RED;
  } else {
    return std::nullopt;
  }

  const kfs::HorizontalPose& pose = *measurement.pose;
  if (!std::isfinite(pose.x_right_mm) ||
      !std::isfinite(pose.z_forward_mm) ||
      !std::isfinite(pose.yaw_deg)) {
    return std::nullopt;
  }

  constexpr double kPi = 3.14159265358979323846;
  // KfsTarget 对外采用车体平面坐标：前为 +x、左为 +y。
  // 内部测量仍保持相机坐标：右为 +x_right、前为 +z_forward。
  message.x_m = static_cast<float>(pose.z_forward_mm / 1000.0 - 0.007);  // 0.007 m 为相机光心到前盖板的 x 偏移。
  message.y_m = static_cast<float>(-pose.x_right_mm / 1000.0  + 0.011);  // 0.011 m 为相机光心到车体中心的 y 偏移。
  message.yaw_rad = static_cast<float>(pose.yaw_deg * kPi / 180.0);

  if (!std::isfinite(message.x_m) ||
      !std::isfinite(message.y_m) ||
      !std::isfinite(message.yaw_rad)) {
    return std::nullopt;
  }
  return message;
}

}  // namespace kfs_vision
