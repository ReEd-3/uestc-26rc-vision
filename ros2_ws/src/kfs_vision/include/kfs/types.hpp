#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace kfs {

struct PlaneFitConfig {
  int min_depth_mm = 100;
  int max_depth_mm = 1100;
  int erosion_px = 3;
  int sample_step_px = 25;
  int ransac_iterations = 300;
  double inlier_threshold_mm = 17.0;
  int min_inliers = 95;
  double known_width_mm = 350.0;
  double width_tolerance_mm = 20.0;
  double center_band_mm = 12.0;
  int image_border_margin_px = 2;
};

struct Intrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
};

struct PlaneModel {
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  double offset = 0.0;
  std::vector<cv::Point> inlier_pixels;
  std::size_t sample_count = 0;
  std::size_t inlier_count = 0;
  double rms_mm = 0.0;
  double angle_deg = 0.0;

  [[nodiscard]] double distanceMm() const { return std::abs(offset); }
};

struct HorizontalPose {
  double x_right_mm = 0.0;
  double z_forward_mm = 0.0;
  double yaw_deg = 0.0;
  double observed_width_mm = 0.0;
  cv::Point center_pixel;
  std::vector<cv::Point> left_edge_pixels;
  std::vector<cv::Point> right_edge_pixels;
};

struct SampledPoints {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>};
  std::vector<int> xs;
  std::vector<int> ys;
};

struct DensePlaneResult {
  cv::Mat mask;  // CV_8UC1, values are 0 or 255.
  std::vector<cv::Point> pixels;
  std::vector<double> depth_mm;
};

struct PoseResult {
  std::optional<HorizontalPose> pose;
  std::string reason;
};

struct Measurement {
  std::string name;
  std::optional<PlaneModel> plane;
  std::optional<PlaneModel> side_plane;
  std::optional<HorizontalPose> pose;
  std::string pose_reason;
};

struct RuntimeDebug {
  double output_fps = 0.0;
  std::string target_state = "none";
  std::string class_name = "-";
  float confidence = 0.0F;
  int mask_pixels = 0;
  int bbox_w = 0;
  int bbox_h = 0;
  std::size_t sample_count = 0;
  std::string plane_state = "not run";
  const std::map<std::string, double>* timings = nullptr;
  const PlaneFitConfig* plane_cfg = nullptr;
};

}  // namespace kfs
