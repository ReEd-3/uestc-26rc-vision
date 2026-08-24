#include "kfs/yolo_seg.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace kfs {
namespace {

using Clock = std::chrono::steady_clock;

double elapsedMilliseconds(Clock::time_point started) {
  return std::chrono::duration<double, std::milli>(Clock::now() - started).count();
}

struct Candidate {
  cv::Rect2f box_xyxy;
  int class_id = -1;
  float score = 0.0F;
  int row = -1;
};

float intersectionOverUnion(const cv::Rect2f& a, const cv::Rect2f& b) {
  const float left = std::max(a.x, b.x);
  const float top = std::max(a.y, b.y);
  const float right = std::min(a.x + a.width, b.x + b.width);
  const float bottom = std::min(a.y + a.height, b.y + b.height);
  const float intersection = std::max(0.0F, right - left) * std::max(0.0F, bottom - top);
  const float union_area = a.area() + b.area() - intersection;
  return union_area > 0.0F ? intersection / union_area : 0.0F;
}

std::vector<Candidate> nonMaximumSuppression(std::vector<Candidate> candidates,
                                              float iou_threshold = 0.70F) {
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& lhs, const Candidate& rhs) { return lhs.score > rhs.score; });
  std::vector<Candidate> kept;
  std::vector<unsigned char> suppressed(candidates.size(), 0);
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (suppressed[i]) continue;
    kept.push_back(candidates[i]);
    for (std::size_t j = i + 1; j < candidates.size(); ++j) {
      if (suppressed[j] || candidates[i].class_id != candidates[j].class_id) continue;
      if (intersectionOverUnion(candidates[i].box_xyxy, candidates[j].box_xyxy) > iou_threshold) {
        suppressed[j] = 1;
      }
    }
  }
  return kept;
}

std::string shapeString(const std::vector<int64_t>& shape) {
  std::ostringstream stream;
  stream << '[';
  for (std::size_t i = 0; i < shape.size(); ++i) {
    if (i != 0) stream << ',';
    stream << shape[i];
  }
  stream << ']';
  return stream.str();
}

void requireShape(const std::vector<int64_t>& actual, const std::vector<int64_t>& expected,
                  const std::string& tensor_name) {
  if (actual != expected) {
    throw std::runtime_error(tensor_name + " shape mismatch: got " + shapeString(actual) +
                             ", expected " + shapeString(expected));
  }
}

}  // namespace

OnnxYoloSegmenter::OnnxYoloSegmenter(const std::filesystem::path& model_path,
                                     float confidence_threshold)
    : confidence_threshold_(confidence_threshold) {
  if (!std::filesystem::is_regular_file(model_path)) {
    throw std::runtime_error("ONNX model does not exist: " + model_path.string());
  }
  const auto providers = Ort::GetAvailableProviders();
  if (std::find(providers.begin(), providers.end(), execution_provider_) == providers.end()) {
    std::ostringstream available;
    for (std::size_t i = 0; i < providers.size(); ++i) {
      if (i != 0) available << ", ";
      available << providers[i];
    }
    throw std::runtime_error("CUDAExecutionProvider is unavailable (available: " + available.str() + ")");
  }

  session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  session_options_.SetIntraOpNumThreads(1);
  Ort::CUDAProviderOptions cuda_options;
  cuda_options.Update({{"device_id", "0"}});
  session_options_.AppendExecutionProvider_CUDA_V2(*cuda_options);
  session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);

  Ort::AllocatorWithDefaultOptions allocator;
  for (std::size_t i = 0; i < session_->GetInputCount(); ++i) {
    auto name = session_->GetInputNameAllocated(i, allocator);
    input_names_.emplace_back(name.get());
  }
  for (std::size_t i = 0; i < session_->GetOutputCount(); ++i) {
    auto name = session_->GetOutputNameAllocated(i, allocator);
    output_names_.emplace_back(name.get());
  }
  for (const std::string& name : input_names_) input_name_ptrs_.push_back(name.c_str());
  for (const std::string& name : output_names_) output_name_ptrs_.push_back(name.c_str());
  validateModelContract();
}

void OnnxYoloSegmenter::validateModelContract() {
  if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 2) {
    throw std::runtime_error("model must have exactly one input and two outputs");
  }
  requireShape(session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape(),
               {1, 3, kInputSize, kInputSize}, "input");
  requireShape(session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape(),
               {1, 4 + kNumClasses + kMaskDim, 8400}, "detection output");
  requireShape(session_->GetOutputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape(),
               {1, kMaskDim, 160, 160}, "prototype output");
}

cv::Mat OnnxYoloSegmenter::preprocess(const cv::Mat& frame, bool input_is_rgb,
                                      LetterboxInfo& info,
                                      std::vector<float>& input_values) const {
  if (frame.empty() || frame.type() != CV_8UC3) {
    throw std::invalid_argument("YOLO input must be a non-empty CV_8UC3 image");
  }
  info.scale = std::min(static_cast<float>(kInputSize) / frame.cols,
                        static_cast<float>(kInputSize) / frame.rows);
  info.resized_width = static_cast<int>(std::round(frame.cols * info.scale));
  info.resized_height = static_cast<int>(std::round(frame.rows * info.scale));
  const int horizontal_padding = kInputSize - info.resized_width;
  const int vertical_padding = kInputSize - info.resized_height;
  info.left = static_cast<int>(std::round(horizontal_padding * 0.5 - 0.1));
  info.top = static_cast<int>(std::round(vertical_padding * 0.5 - 0.1));
  const int right = horizontal_padding - info.left;
  const int bottom = vertical_padding - info.top;

  cv::Mat resized;
  cv::resize(frame, resized, cv::Size(info.resized_width, info.resized_height),
             0.0, 0.0, cv::INTER_LINEAR);
  cv::Mat letterboxed;
  cv::copyMakeBorder(resized, letterboxed, info.top, bottom, info.left, right,
                     cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

  cv::Mat rgb_float;
  if (input_is_rgb) {
    rgb_float = letterboxed;
  } else {
    cv::cvtColor(letterboxed, rgb_float, cv::COLOR_BGR2RGB);
  }
  rgb_float.convertTo(rgb_float, CV_32FC3, 1.0 / 255.0);
  const int plane_size = kInputSize * kInputSize;
  input_values.resize(static_cast<std::size_t>(3 * plane_size));
  std::vector<cv::Mat> channels;
  channels.reserve(3);
  for (int channel = 0; channel < 3; ++channel) {
    channels.emplace_back(kInputSize, kInputSize, CV_32FC1,
                          input_values.data() + channel * plane_size);
  }
  cv::split(rgb_float, channels);
  return letterboxed;
}

std::optional<SegmentationDetection> OnnxYoloSegmenter::inferBest(const cv::Mat& frame_bgr) {
  return inferBestImpl(frame_bgr, false);
}

std::optional<SegmentationDetection> OnnxYoloSegmenter::inferBestRgb(const cv::Mat& frame_rgb) {
  return inferBestImpl(frame_rgb, true);
}

std::optional<SegmentationDetection> OnnxYoloSegmenter::inferBestImpl(
    const cv::Mat& frame, bool input_is_rgb) {
  last_timings_ = {};
  auto stage_started = Clock::now();
  LetterboxInfo letterbox;
  preprocess(frame, input_is_rgb, letterbox, input_values_);
  last_timings_.preprocess_ms = elapsedMilliseconds(stage_started);

  stage_started = Clock::now();
  const std::array<int64_t, 4> input_shape{1, 3, kInputSize, kInputSize};
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      memory_info_, input_values_.data(), input_values_.size(), input_shape.data(), input_shape.size());
  auto outputs = session_->Run(Ort::RunOptions{nullptr}, input_name_ptrs_.data(), &input_tensor, 1,
                               output_name_ptrs_.data(), output_name_ptrs_.size());
  last_timings_.inference_ms = elapsedMilliseconds(stage_started);

  stage_started = Clock::now();
  if (outputs.size() != 2 || !outputs[0].IsTensor() || !outputs[1].IsTensor()) {
    throw std::runtime_error("ONNX Runtime returned an invalid output set");
  }

  const auto detection_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
  const auto prototype_shape = outputs[1].GetTensorTypeAndShapeInfo().GetShape();
  requireShape(detection_shape, {1, 4 + kNumClasses + kMaskDim, 8400}, "runtime detection output");
  requireShape(prototype_shape, {1, kMaskDim, 160, 160}, "runtime prototype output");
  const int prediction_count = static_cast<int>(detection_shape[2]);
  float* detection_data = outputs[0].GetTensorMutableData<float>();
  float* prototype_data = outputs[1].GetTensorMutableData<float>();
  std::vector<Candidate> candidates;
  candidates.reserve(128);
  for (int row_index = 0; row_index < prediction_count; ++row_index) {
    const auto value = [detection_data, prediction_count, row_index](int attribute) {
      return detection_data[attribute * prediction_count + row_index];
    };
    int class_id = 0;
    float score = value(4);
    for (int class_index = 1; class_index < kNumClasses; ++class_index) {
      if (value(4 + class_index) > score) {
        score = value(4 + class_index);
        class_id = class_index;
      }
    }
    if (score < confidence_threshold_) continue;
    const float width = value(2);
    const float height = value(3);
    const float x1 = value(0) - width * 0.5F;
    const float y1 = value(1) - height * 0.5F;
    candidates.push_back({cv::Rect2f(x1, y1, width, height), class_id, score, row_index});
  }
  if (candidates.empty()) {
    last_timings_.decode_ms = elapsedMilliseconds(stage_started);
    return std::nullopt;
  }
  candidates = nonMaximumSuppression(std::move(candidates));
  last_timings_.decode_ms = elapsedMilliseconds(stage_started);

  stage_started = Clock::now();
  cv::Mat prototypes(kMaskDim, 160 * 160, CV_32FC1, prototype_data);
  const cv::Rect content_region(letterbox.left, letterbox.top,
                                letterbox.resized_width, letterbox.resized_height);
  for (const Candidate& candidate : candidates) {
    std::array<float, kMaskDim> coefficient_values;
    for (int index = 0; index < kMaskDim; ++index) {
      coefficient_values[static_cast<std::size_t>(index)] =
          detection_data[(4 + kNumClasses + index) * prediction_count + candidate.row];
    }
    cv::Mat coefficients(1, kMaskDim, CV_32FC1, coefficient_values.data());
    cv::Mat mask_flat = coefficients * prototypes;
    cv::Mat mask_prototype = mask_flat.reshape(1, 160);
    cv::Mat mask_logits;
    cv::resize(mask_prototype, mask_logits, cv::Size(kInputSize, kInputSize),
               0.0, 0.0, cv::INTER_LINEAR);
    cv::Mat binary_mask;
    cv::compare(mask_logits, 0.0, binary_mask, cv::CMP_GT);

    // Ultralytics uses integer pixel coordinates r >= x1 && r < x2.
    const int x1 = std::clamp(static_cast<int>(std::ceil(candidate.box_xyxy.x)), 0, kInputSize);
    const int y1 = std::clamp(static_cast<int>(std::ceil(candidate.box_xyxy.y)), 0, kInputSize);
    const int x2 = std::clamp(static_cast<int>(std::ceil(candidate.box_xyxy.x + candidate.box_xyxy.width)),
                              0, kInputSize);
    const int y2 = std::clamp(static_cast<int>(std::ceil(candidate.box_xyxy.y + candidate.box_xyxy.height)),
                              0, kInputSize);
    binary_mask.rowRange(0, y1).setTo(0);
    binary_mask.rowRange(y2, kInputSize).setTo(0);
    if (x2 > x1 && y2 > y1) {
      binary_mask(cv::Rect(0, y1, x1, y2 - y1)).setTo(0);
      binary_mask(cv::Rect(x2, y1, kInputSize - x2, y2 - y1)).setTo(0);
    } else {
      binary_mask.setTo(0);
    }

    // Fixed 640x640 ONNX requires square letterboxing. Remove the padding before
    // restoring the color/depth pixel grid so geometry stays registered.
    cv::Mat full_resolution_mask;
    cv::resize(binary_mask(content_region), full_resolution_mask, frame.size(),
               0.0, 0.0, cv::INTER_NEAREST);
    const int mask_pixels = cv::countNonZero(full_resolution_mask);
    if (mask_pixels == 0) continue;
    const cv::Rect mask_bounds = cv::boundingRect(full_resolution_mask);
    last_timings_.mask_ms = elapsedMilliseconds(stage_started);
    return SegmentationDetection{candidate.class_id, candidate.score,
                                 std::move(full_resolution_mask), mask_pixels, mask_bounds};
  }
  last_timings_.mask_ms = elapsedMilliseconds(stage_started);
  return std::nullopt;
}

const char* className(int class_id) {
  switch (class_id) {
    case 0:
      return "Blue KFS";
    case 1:
      return "Red KFS";
    default:
      return "Unknown KFS";
  }
}

}  // namespace kfs
