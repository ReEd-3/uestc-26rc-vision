#pragma once

#include <filesystem>

#include "kfs/types.hpp"

namespace kfs {

struct AppConfig {
  std::filesystem::path model_path;
  std::filesystem::path plane_config_path;
  int frame_width = 1280;
  int frame_height = 720;
  int fps = 30;
  float yolo_confidence = 0.50F;
  double front_housing_from_depth_zero_mm = 4.930;
  PlaneFitConfig plane;
};

PlaneFitConfig loadPlaneConfig(const std::filesystem::path& path);
void savePlaneConfig(const PlaneFitConfig& config, const std::filesystem::path& path);

}  // namespace kfs
