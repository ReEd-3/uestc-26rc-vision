#include "kfs/debug_view.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "kfs/config.hpp"

namespace kfs {
namespace {

std::string fixed(double value, int precision = 1) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

double timing(const RuntimeDebug& debug, const std::string& key) {
  if (debug.timings == nullptr) return 0.0;
  const auto it = debug.timings->find(key);
  return it == debug.timings->end() ? 0.0 : it->second;
}

int medianX(const std::vector<cv::Point>& points) {
  std::vector<int> values;
  values.reserve(points.size());
  for (const cv::Point& point : points) values.push_back(point.x);
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
  if (values.size() % 2 != 0) return values[middle];
  const int lower = *std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
  return static_cast<int>(std::round((lower + values[middle]) * 0.5));
}

}  // namespace

OpenCvControls::OpenCvControls(
    PlaneFitConfig& config,
    std::optional<std::filesystem::path> config_output_path)
    : config_(config), config_output_path_(std::move(config_output_path)),
      specs_({
          {"Min depth (mm)", &PlaneFitConfig::min_depth_mm, nullptr, 10, 3000},
          {"Max depth (mm)", &PlaneFitConfig::max_depth_mm, nullptr, 20, 5000},
          {"Min valid depth px", &PlaneFitConfig::min_valid_depth_pixels, nullptr, 1, 1000},
          {"Min in-range (%)", nullptr, &PlaneFitConfig::min_in_range_ratio, 1, 100, 100.0},
          {"Temporal forward jump (mm)", nullptr, &PlaneFitConfig::temporal_max_forward_jump_mm, 0, 300},
          {"Temporal right jump (mm)", nullptr, &PlaneFitConfig::temporal_max_right_jump_mm, 0, 300},
          {"Temporal yaw jump (deg)", nullptr, &PlaneFitConfig::temporal_max_yaw_jump_deg, 0, 45},
          {"Temporal reset (ms)", &PlaneFitConfig::temporal_reset_after_ms, nullptr, 0, 2000},
          {"Mask erosion (px)", &PlaneFitConfig::erosion_px, nullptr, 0, 31},
          {"Sample step (px)", &PlaneFitConfig::sample_step_px, nullptr, 1, 40},
          {"RANSAC iterations", &PlaneFitConfig::ransac_iterations, nullptr, 50, 1500},
          {"Inlier threshold (mm)", nullptr, &PlaneFitConfig::inlier_threshold_mm, 1, 50},
          {"Min plane inliers", &PlaneFitConfig::min_inliers, nullptr, 20, 1500},
          {"Known width (mm)", nullptr, &PlaneFitConfig::known_width_mm, 100, 1000},
          {"Width tolerance (mm)", nullptr, &PlaneFitConfig::width_tolerance_mm, 1, 100},
          {"Center band (mm)", nullptr, &PlaneFitConfig::center_band_mm, 1, 50},
          {"Border margin (px)", &PlaneFitConfig::image_border_margin_px, nullptr, 0, 30},
      }) {}

void OpenCvControls::create() {
  cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
  cv::Mat header(72, 760, CV_8UC3, cv::Scalar(35, 35, 35));
  const char* title = config_output_path_ ? "Plane-fit controls (auto-saved)"
                                          : "Plane-fit controls (runtime only)";
  cv::putText(header, title, cv::Point(18, 43),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(80, 230, 255), 2);
  cv::imshow(kWindowName, header);
  cv::resizeWindow(kWindowName, 760, 520);
  for (const SliderSpec& spec : specs_) {
    cv::createTrackbar(spec.name, kWindowName, nullptr, spec.high,
                       [](int, void*) {}, nullptr);
    cv::setTrackbarMin(spec.name, kWindowName, spec.low);
    const double value = spec.integer_member != nullptr ? config_.*(spec.integer_member)
                                                        : config_.*(spec.double_member);
    cv::setTrackbarPos(spec.name, kWindowName,
                       std::clamp(static_cast<int>(std::round(value * spec.scale)), spec.low, spec.high));
  }
  created_ = true;
}

void OpenCvControls::pollAndSave() {
  if (!created_) return;
  bool changed = false;
  for (const SliderSpec& spec : specs_) {
    const int value = std::clamp(cv::getTrackbarPos(spec.name, kWindowName), spec.low, spec.high);
    if (spec.integer_member != nullptr) {
      int& destination = config_.*(spec.integer_member);
      if (destination != value) {
        destination = value;
        changed = true;
      }
    } else {
      double& destination = config_.*(spec.double_member);
      const double scaled_value = static_cast<double>(value) / spec.scale;
      if (destination != scaled_value) {
        destination = scaled_value;
        changed = true;
      }
    }
  }
  if (config_.max_depth_mm < config_.min_depth_mm + 10) {
    config_.max_depth_mm = config_.min_depth_mm + 10;
    cv::setTrackbarPos("Max depth (mm)", kWindowName, config_.max_depth_mm);
    changed = true;
  }
  if (changed && config_output_path_) {
    try {
      savePlaneConfig(config_, *config_output_path_);
    } catch (const std::exception& error) {
      throw std::runtime_error(std::string("failed to save plane controls: ") + error.what());
    }
  }
}

cv::Mat buildDebugView(const cv::Mat& source, const cv::Mat& inlier_mask,
                       const std::vector<Measurement>& measurements,
                       const RuntimeDebug& runtime_debug) {
  CV_Assert(source.type() == CV_8UC3 && inlier_mask.type() == CV_8UC1 &&
            source.size() == inlier_mask.size() && runtime_debug.plane_cfg != nullptr);
  const cv::Size tile_size(source.cols / 2, source.rows / 2);
  const double scale_x = static_cast<double>(tile_size.width) / source.cols;
  const double scale_y = static_cast<double>(tile_size.height) / source.rows;
  cv::Mat source_tile;
  cv::resize(source, source_tile, tile_size, 0.0, 0.0, cv::INTER_LINEAR);
  cv::Mat mask_tile;
  cv::resize(inlier_mask, mask_tile, tile_size, 0.0, 0.0, cv::INTER_NEAREST);
  cv::Mat status = cv::Mat::zeros(source_tile.size(), source_tile.type());

  const auto scaledPoint = [scale_x, scale_y](double x, double y) {
    return cv::Point(static_cast<int>(std::round(x * scale_x)),
                     static_cast<int>(std::round(y * scale_y)));
  };
  const auto drawStatus = [&status](std::string text, int y, cv::Scalar color, double scale) {
    const int max_width = status.cols - 32;
    while (text.size() > 1 &&
           cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, 1, nullptr).width > max_width) {
      text.resize(text.size() - 1);
      if (!text.empty()) text.back() = '.';
    }
    cv::putText(status, text, cv::Point(16, y), cv::FONT_HERSHEY_SIMPLEX,
                scale, color, 1, cv::LINE_AA);
  };
  const cv::Scalar white(235, 235, 235);
  drawStatus("KFS Runtime Diagnostics", 28, cv::Scalar(90, 220, 255), 0.70);
  drawStatus("Output FPS: " + fixed(runtime_debug.output_fps),
             55, cv::Scalar(80, 255, 120), 0.54);
  drawStatus("Target: " + runtime_debug.target_state + "  |  Class: " +
                 runtime_debug.class_name + "  conf: " + fixed(runtime_debug.confidence, 3),
             80, runtime_debug.target_state == "accepted" ? cv::Scalar(255, 230, 100)
                                                            : cv::Scalar(80, 80, 255), 0.50);
  drawStatus("Mask: " + std::to_string(runtime_debug.mask_pixels) + " px  |  bbox: " +
                 std::to_string(runtime_debug.bbox_w) + " x " + std::to_string(runtime_debug.bbox_h) +
                 " | center dx: " + fixed(runtime_debug.target_center_offset_px, 1) + " px",
             105, white, 0.54);
  const DepthGateStats& gate = runtime_debug.depth_gate;
  drawStatus("Depth: valid " + std::to_string(gate.valid_count) + "/" +
                 std::to_string(gate.mask_count) + " | in range " +
                 std::to_string(gate.in_range_count) + " | ratio " +
                 fixed(gate.inRangeRatio(), 3),
             130, white, 0.54);

  const Measurement* measurement = measurements.empty() ? nullptr : &measurements.front();
  if (measurement == nullptr) {
    drawStatus("Pose: unavailable (target was not accepted)", 155, cv::Scalar(80, 80, 255), 0.54);
  } else if (!measurement->plane) {
    drawStatus("Front plane: FAILED", 155, cv::Scalar(80, 80, 255), 0.62);
  } else {
    const PlaneModel& plane = *measurement->plane;
    drawStatus("Plane: angle=" + fixed(plane.angle_deg) + " deg  rms=" + fixed(plane.rms_mm) + " mm",
               155, white, 0.54);
    drawStatus("Inliers: " + std::to_string(plane.inlier_count) + "/" +
                   std::to_string(plane.sample_count) + "  |  side: " +
                   (measurement->side_plane ? "yes" : "no"),
               180, white, 0.54);
    if (!measurement->pose) {
      drawStatus("Pose: INVALID (" + measurement->pose_reason + ")", 205,
                 cv::Scalar(80, 80, 255), 0.50);
    } else {
      const HorizontalPose& pose = *measurement->pose;
      drawStatus(std::string("Pose: X=") + (pose.x_right_mm >= 0.0 ? "+" : "") + fixed(pose.x_right_mm, 0) +
                     "  Z=" + fixed(pose.z_forward_mm, 0) + " mm",
                 205, white, 0.54);
      drawStatus(std::string("Yaw=") + (pose.yaw_deg >= 0.0 ? "+" : "") + fixed(pose.yaw_deg) +
                     " deg  width=" + fixed(pose.observed_width_mm, 0) + " mm",
                 230, white, 0.54);
    }
  }

  drawStatus("YOLO " + fixed(timing(runtime_debug, "yolo")) + " ms: infer " +
                 fixed(timing(runtime_debug, "yolo_infer")) + " | pre " +
                 fixed(timing(runtime_debug, "yolo_pre")),
             258, cv::Scalar(180, 220, 255), 0.48);
  drawStatus("decode " + fixed(timing(runtime_debug, "yolo_decode")) + " | mask " +
                 fixed(timing(runtime_debug, "yolo_mask")) + " | dense " +
                 fixed(timing(runtime_debug, "dense_pose")),
             280, cv::Scalar(180, 220, 255), 0.48);
  drawStatus("align " + fixed(timing(runtime_debug, "align_depth")) + " | mosaic " +
                 fixed(timing(runtime_debug, "build_mosaic")) + " | UI " +
                 fixed(timing(runtime_debug, "display")),
             302, cv::Scalar(180, 220, 255), 0.48);
  const PlaneFitConfig& config = *runtime_debug.plane_cfg;
  drawStatus("Depth " + std::to_string(config.min_depth_mm) + "-" +
                 std::to_string(config.max_depth_mm) + " mm | step " +
                 std::to_string(config.sample_step_px) + " | RANSAC " +
                 std::to_string(config.ransac_iterations),
             326, cv::Scalar(170, 170, 170), 0.46);
  drawStatus("Inlier <= " + fixed(config.inlier_threshold_mm, 0) +
                 " mm | min inliers " + std::to_string(config.min_inliers),
             348, cv::Scalar(170, 170, 170), 0.46);

  cv::Mat colored = source_tile.clone();
  colored.setTo(cv::Scalar(0, 255, 0), mask_tile);
  cv::Mat inlier_overlay;
  cv::addWeighted(source_tile, 0.45, colored, 0.55, 0.0, inlier_overlay);
  for (const Measurement& item : measurements) {
    if (!item.pose) continue;
    for (const auto* edge : {&item.pose->left_edge_pixels, &item.pose->right_edge_pixels}) {
      if (edge->size() < 2) continue;
      const auto [min_y, max_y] = std::minmax_element(
          edge->begin(), edge->end(),
          [](const cv::Point& lhs, const cv::Point& rhs) { return lhs.y < rhs.y; });
      const int x = medianX(*edge);
      cv::line(inlier_overlay, scaledPoint(x, min_y->y), scaledPoint(x, max_y->y),
               cv::Scalar(0, 255, 255), 1);
    }
    cv::drawMarker(inlier_overlay, scaledPoint(item.pose->center_pixel.x, item.pose->center_pixel.y),
                   cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 12, 1);
  }
  cv::putText(inlier_overlay, "Front-plane inliers (green)", scaledPoint(16, 35),
              cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1);

  cv::Mat result;
  cv::hconcat(inlier_overlay, status, result);
  return result;
}

}  // namespace kfs
