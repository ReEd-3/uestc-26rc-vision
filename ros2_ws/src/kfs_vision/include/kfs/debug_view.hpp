#pragma once

#include <filesystem>
#include <optional>
#include <vector>

#include "kfs/types.hpp"

namespace kfs {

class OpenCvControls {
 public:
  OpenCvControls(PlaneFitConfig& config,
                 std::optional<std::filesystem::path> config_output_path);
  void create();
  void pollAndSave();

 private:
  struct SliderSpec {
    const char* name;
    int PlaneFitConfig::*integer_member;
    double PlaneFitConfig::*double_member;
    int low;
    int high;
  };

  static constexpr const char* kWindowName = "Plane Controls";
  PlaneFitConfig& config_;
  std::optional<std::filesystem::path> config_output_path_;
  std::vector<SliderSpec> specs_;
  bool created_ = false;
};

cv::Mat buildDebugView(const cv::Mat& source, const cv::Mat& inlier_mask,
                       const std::vector<Measurement>& measurements,
                       const RuntimeDebug& runtime_debug);

}  // namespace kfs
