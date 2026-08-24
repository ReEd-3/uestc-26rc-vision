#pragma once

#include <array>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>

namespace kfs {

struct SegmentationDetection {
  int class_id = -1;
  float confidence = 0.0F;
  cv::Mat mask;  // Full-resolution CV_8UC1, 0 or 255.
  int mask_pixels = 0;
  cv::Rect mask_bounds;
};

struct YoloTimings {
  double preprocess_ms = 0.0;
  double inference_ms = 0.0;
  double decode_ms = 0.0;
  double mask_ms = 0.0;
};

class OnnxYoloSegmenter {
 public:
  explicit OnnxYoloSegmenter(const std::filesystem::path& model_path,
                             float confidence_threshold = 0.50F);

  std::optional<SegmentationDetection> inferBest(const cv::Mat& frame_bgr);
  std::optional<SegmentationDetection> inferBestRgb(const cv::Mat& frame_rgb);
  [[nodiscard]] const std::string& executionProvider() const { return execution_provider_; }
  [[nodiscard]] const YoloTimings& lastTimings() const { return last_timings_; }

 private:
  struct LetterboxInfo {
    float scale = 1.0F;
    int left = 0;
    int top = 0;
    int resized_width = 0;
    int resized_height = 0;
  };

  cv::Mat preprocess(const cv::Mat& frame, bool input_is_rgb,
                     LetterboxInfo& info, std::vector<float>& input_values) const;
  std::optional<SegmentationDetection> inferBestImpl(const cv::Mat& frame,
                                                     bool input_is_rgb);
  void validateModelContract();

  static constexpr int kInputSize = 640;
  static constexpr int kNumClasses = 2;
  static constexpr int kMaskDim = 32;

  float confidence_threshold_;
  std::string execution_provider_ = "CUDAExecutionProvider";
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "kfs_depth_pose"};
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  Ort::MemoryInfo memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<const char*> input_name_ptrs_;
  std::vector<const char*> output_name_ptrs_;
  std::vector<float> input_values_;
  YoloTimings last_timings_;
};

const char* className(int class_id);

}  // namespace kfs
