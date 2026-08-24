#pragma once

#include <optional>

#include <custom_msgs/msg/kfs_target.hpp>

#include "kfs/types.hpp"
#include "kfs/yolo_seg.hpp"

namespace kfs_vision {

std::optional<custom_msgs::msg::KfsTarget> makeTargetMessage(
    const kfs::SegmentationDetection& detection,
    const kfs::Measurement& measurement);

}  // namespace kfs_vision

