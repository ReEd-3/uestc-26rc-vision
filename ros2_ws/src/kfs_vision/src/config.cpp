#include "kfs/config.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace kfs {
namespace {

PlaneFitConfig defaults() {
  PlaneFitConfig config;
  config.max_depth_mm = 1000;
  return config;
}

void sanitize(PlaneFitConfig& config) {
  config.min_depth_mm = std::max(10, config.min_depth_mm);
  config.max_depth_mm = std::max(config.min_depth_mm + 10, config.max_depth_mm);
  config.erosion_px = std::max(0, config.erosion_px);
  config.sample_step_px = std::max(1, config.sample_step_px);
  config.ransac_iterations = std::max(1, config.ransac_iterations);
  config.inlier_threshold_mm = std::max(0.1, config.inlier_threshold_mm);
  config.min_inliers = std::max(3, config.min_inliers);
  config.known_width_mm = std::max(1.0, config.known_width_mm);
  config.width_tolerance_mm = std::max(0.1, config.width_tolerance_mm);
  config.center_band_mm = std::max(0.1, config.center_band_mm);
  config.image_border_margin_px = std::max(0, config.image_border_margin_px);
}

template <typename T>
void assignIfNumber(const nlohmann::json& data, const char* key, T& destination) {
  const auto it = data.find(key);
  if (it != data.end() && it->is_number()) destination = it->get<T>();
}

nlohmann::json toJson(const PlaneFitConfig& config) {
  return {
      {"min_depth_mm", config.min_depth_mm},
      {"max_depth_mm", config.max_depth_mm},
      {"erosion_px", config.erosion_px},
      {"sample_step_px", config.sample_step_px},
      {"ransac_iterations", config.ransac_iterations},
      {"inlier_threshold_mm", config.inlier_threshold_mm},
      {"min_inliers", config.min_inliers},
      {"known_width_mm", config.known_width_mm},
      {"width_tolerance_mm", config.width_tolerance_mm},
      {"center_band_mm", config.center_band_mm},
      {"image_border_margin_px", config.image_border_margin_px},
  };
}

}  // namespace

PlaneFitConfig loadPlaneConfig(const std::filesystem::path& path) {
  PlaneFitConfig config = defaults();
  try {
    std::ifstream stream(path);
    if (!stream) return config;
    nlohmann::json data;
    stream >> data;
    assignIfNumber(data, "min_depth_mm", config.min_depth_mm);
    assignIfNumber(data, "max_depth_mm", config.max_depth_mm);
    assignIfNumber(data, "erosion_px", config.erosion_px);
    assignIfNumber(data, "sample_step_px", config.sample_step_px);
    assignIfNumber(data, "ransac_iterations", config.ransac_iterations);
    assignIfNumber(data, "inlier_threshold_mm", config.inlier_threshold_mm);
    assignIfNumber(data, "min_inliers", config.min_inliers);
    assignIfNumber(data, "known_width_mm", config.known_width_mm);
    assignIfNumber(data, "width_tolerance_mm", config.width_tolerance_mm);
    assignIfNumber(data, "center_band_mm", config.center_band_mm);
    assignIfNumber(data, "image_border_margin_px", config.image_border_margin_px);
    sanitize(config);
  } catch (const std::exception& error) {
    std::cerr << "Plane config load failed; using defaults: " << path << " ("
              << error.what() << ")\n";
    return defaults();
  }
  return config;
}

void savePlaneConfig(const PlaneFitConfig& config, const std::filesystem::path& path) {
  std::filesystem::create_directories(path.parent_path());
  const std::filesystem::path temporary_path = path.string() + ".tmp";
  {
    std::ofstream stream(temporary_path, std::ios::trunc);
    if (!stream) throw std::runtime_error("cannot open temporary config: " + temporary_path.string());
    stream << toJson(config).dump(2) << '\n';
    stream.flush();
    if (!stream) throw std::runtime_error("cannot write temporary config: " + temporary_path.string());
  }
  std::filesystem::rename(temporary_path, path);
}

}  // namespace kfs

