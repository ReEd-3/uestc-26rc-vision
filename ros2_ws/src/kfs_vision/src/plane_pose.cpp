#include "kfs/plane_pose.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

#include <Eigen/Eigenvalues>
#include <opencv2/imgproc.hpp>
#include <pcl/common/centroid.h>

namespace kfs {
namespace {

constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

struct FittedPlane {
  Eigen::Vector3d normal;
  double offset;
};

std::optional<FittedPlane> fitPlanePca(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                       const std::vector<int>& indices) {
  if (indices.size() < 3) return std::nullopt;

  pcl::PointCloud<pcl::PointXYZ> subset;
  subset.reserve(indices.size());
  for (const int index : indices) subset.push_back(cloud.points.at(static_cast<std::size_t>(index)));

  Eigen::Vector4f centroid;
  if (pcl::compute3DCentroid(subset, centroid) < 3) return std::nullopt;
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized(subset, centroid, covariance);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
  if (solver.info() != Eigen::Success) return std::nullopt;

  Eigen::Vector3d normal = solver.eigenvectors().col(0).cast<double>();
  const double length = normal.norm();
  if (!std::isfinite(length) || length < 1e-9) return std::nullopt;
  normal /= length;
  double offset = -normal.dot(centroid.head<3>().cast<double>());
  if (normal.z() < 0.0) {
    normal = -normal;
    offset = -offset;
  }
  return FittedPlane{normal, offset};
}

double pointPlaneDistance(const pcl::PointXYZ& point, const FittedPlane& plane) {
  return std::abs(plane.normal.x() * point.x + plane.normal.y() * point.y +
                  plane.normal.z() * point.z + plane.offset);
}

double pointPlaneDistance(const pcl::PointXYZ& point, const PlaneModel& plane) {
  return std::abs(plane.normal.x() * point.x + plane.normal.y() * point.y +
                  plane.normal.z() * point.z + plane.offset);
}

std::vector<unsigned char> classifyInliers(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                           const FittedPlane& plane, double threshold_mm) {
  std::vector<unsigned char> inliers(cloud.size(), 0);
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    inliers[i] = pointPlaneDistance(cloud[i], plane) <= threshold_mm;
  }
  return inliers;
}

std::vector<int> indicesOf(const std::vector<unsigned char>& flags) {
  std::vector<int> indices;
  indices.reserve(flags.size());
  for (std::size_t i = 0; i < flags.size(); ++i) {
    if (flags[i]) indices.push_back(static_cast<int>(i));
  }
  return indices;
}

double frontAngleDegrees(const Eigen::Vector3d& normal) {
  return std::acos(std::clamp(normal.z(), -1.0, 1.0)) * kRadiansToDegrees;
}

double median(std::vector<double> values) {
  if (values.empty()) throw std::invalid_argument("median of empty vector");
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
  const double upper = values[middle];
  if (values.size() % 2 != 0) return upper;
  const double lower = *std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
  return (lower + upper) * 0.5;
}

double medianCoordinate(const std::vector<cv::Point>& points, bool x_coordinate) {
  std::vector<double> values;
  values.reserve(points.size());
  for (const cv::Point& point : points) values.push_back(x_coordinate ? point.x : point.y);
  return median(std::move(values));
}

std::array<int, 3> drawDistinctTriple(std::mt19937& rng, int size) {
  std::uniform_int_distribution<int> distribution(0, size - 1);
  std::array<int, 3> result{};
  result[0] = distribution(rng);
  do {
    result[1] = distribution(rng);
  } while (result[1] == result[0]);
  do {
    result[2] = distribution(rng);
  } while (result[2] == result[0] || result[2] == result[1]);
  return result;
}

std::optional<PlaneModel> estimateFrontImpl(const SampledPoints& samples,
                                            const PlaneFitConfig& config,
                                            std::mt19937& rng) {
  const auto& cloud = *samples.cloud;
  if (cloud.size() < 3) return std::nullopt;

  std::vector<unsigned char> best_inliers;
  int best_count = 0;
  double best_angle = std::numeric_limits<double>::infinity();

  for (int iteration = 0; iteration < config.ransac_iterations; ++iteration) {
    const auto selected = drawDistinctTriple(rng, static_cast<int>(cloud.size()));
    const std::vector<int> sample_indices(selected.begin(), selected.end());
    const auto candidate = fitPlanePca(cloud, sample_indices);
    if (!candidate) continue;
    auto inliers = classifyInliers(cloud, *candidate, config.inlier_threshold_mm);
    const int count = std::accumulate(inliers.begin(), inliers.end(), 0);
    if (count < config.min_inliers) continue;
    const double angle = frontAngleDegrees(candidate->normal);
    if (angle < best_angle - 1e-6 ||
        (std::abs(angle - best_angle) <= 1e-6 && count > best_count)) {
      best_angle = angle;
      best_count = count;
      best_inliers = std::move(inliers);
    }
  }

  if (best_inliers.empty() || best_count < config.min_inliers) return std::nullopt;
  std::optional<FittedPlane> refined;
  for (int refinement = 0; refinement < 2; ++refinement) {
    refined = fitPlanePca(cloud, indicesOf(best_inliers));
    if (!refined) return std::nullopt;
    best_inliers = classifyInliers(cloud, *refined, config.inlier_threshold_mm);
  }

  const int count = std::accumulate(best_inliers.begin(), best_inliers.end(), 0);
  if (!refined || count < config.min_inliers) return std::nullopt;
  double squared_residual_sum = 0.0;
  std::vector<cv::Point> inlier_pixels;
  inlier_pixels.reserve(static_cast<std::size_t>(count));
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    if (!best_inliers[i]) continue;
    const double residual = pointPlaneDistance(cloud[i], *refined);
    squared_residual_sum += residual * residual;
    inlier_pixels.emplace_back(samples.xs[i], samples.ys[i]);
  }

  PlaneModel result;
  result.normal = refined->normal;
  result.offset = refined->offset;
  result.inlier_pixels = std::move(inlier_pixels);
  result.sample_count = cloud.size();
  result.inlier_count = static_cast<std::size_t>(count);
  result.rms_mm = std::sqrt(squared_residual_sum / count);
  result.angle_deg = frontAngleDegrees(result.normal);
  return result;
}

}  // namespace

FrontPlaneEstimator::FrontPlaneEstimator() : rng_(std::random_device{}()) {}

SampledPoints FrontPlaneEstimator::samplePoints(const cv::Mat& mask, const cv::Mat& depth_mm,
                                                const Intrinsics& intrinsics,
                                                const PlaneFitConfig& config,
                                                const cv::Mat* valid_depth_in_range,
                                                const cv::Rect* known_mask_bounds) const {
  CV_Assert(mask.type() == CV_8UC1 && depth_mm.type() == CV_32FC1 && mask.size() == depth_mm.size());
  SampledPoints result;
  const cv::Rect bounds = known_mask_bounds != nullptr ? *known_mask_bounds
                                                       : cv::boundingRect(mask);
  if (bounds.empty()) return result;
  const int kernel_size = config.erosion_px <= 0 ? 1 : (config.erosion_px | 1);
  const int kernel_radius = config.erosion_px <= 0 ? 0 : kernel_size / 2;
  const cv::Rect expanded(std::max(0, bounds.x - kernel_radius),
                          std::max(0, bounds.y - kernel_radius),
                          std::min(mask.cols, bounds.x + bounds.width + kernel_radius) -
                              std::max(0, bounds.x - kernel_radius),
                          std::min(mask.rows, bounds.y + bounds.height + kernel_radius) -
                              std::max(0, bounds.y - kernel_radius));

  cv::Mat interior;
  if (config.erosion_px <= 0) {
    interior = mask(expanded);
  } else {
    cv::erode(mask(expanded), interior,
              cv::Mat::ones(kernel_size, kernel_size, CV_8UC1));
  }

  cv::Mat local_valid;
  if (valid_depth_in_range != nullptr) {
    local_valid = (*valid_depth_in_range)(expanded);
  } else {
    cv::inRange(depth_mm(expanded), config.min_depth_mm, config.max_depth_mm, local_valid);
  }

  const int estimated_count = cv::countNonZero(interior) /
                              std::max(1, config.sample_step_px * config.sample_step_px);
  const std::size_t reserve_count = static_cast<std::size_t>(std::max(estimated_count, 3));
  result.cloud->reserve(reserve_count);
  result.xs.reserve(reserve_count);
  result.ys.reserve(reserve_count);
  for (int local_y = 0; local_y < expanded.height; ++local_y) {
    const auto* mask_row = interior.ptr<unsigned char>(local_y);
    const auto* valid_row = local_valid.ptr<unsigned char>(local_y);
    const int y = expanded.y + local_y;
    if (y % config.sample_step_px != 0) continue;
    for (int local_x = 0; local_x < expanded.width; ++local_x) {
      if (!mask_row[local_x] || !valid_row[local_x]) continue;
      const int x = expanded.x + local_x;
      if (x % config.sample_step_px != 0) continue;
      const float z = depth_mm.at<float>(y, x);
      result.cloud->emplace_back(static_cast<float>((x - intrinsics.cx) * z / intrinsics.fx),
                                 static_cast<float>((y - intrinsics.cy) * z / intrinsics.fy), z);
      result.xs.push_back(x);
      result.ys.push_back(y);
    }
  }
  result.cloud->width = static_cast<std::uint32_t>(result.cloud->size());
  result.cloud->height = 1;
  result.cloud->is_dense = true;
  return result;
}

std::optional<PlaneModel> FrontPlaneEstimator::estimate(const SampledPoints& samples,
                                                        const PlaneFitConfig& config) {
  return estimateFrontImpl(samples, config, rng_);
}

std::optional<PlaneModel> FrontPlaneEstimator::estimateSecondaryPlane(
    const SampledPoints& samples, const PlaneFitConfig& config,
    const PlaneModel& front_plane) {
  SampledPoints remaining;
  remaining.cloud->reserve(samples.cloud->size());
  remaining.xs.reserve(samples.xs.size());
  remaining.ys.reserve(samples.ys.size());
  for (std::size_t i = 0; i < samples.cloud->size(); ++i) {
    if (pointPlaneDistance((*samples.cloud)[i], front_plane) <= config.inlier_threshold_mm) continue;
    remaining.cloud->push_back((*samples.cloud)[i]);
    remaining.xs.push_back(samples.xs[i]);
    remaining.ys.push_back(samples.ys[i]);
  }
  remaining.cloud->width = static_cast<std::uint32_t>(remaining.cloud->size());
  remaining.cloud->height = 1;
  if (remaining.cloud->size() < 3) return std::nullopt;

  const int required = std::max(
      20, std::min(config.min_inliers / 4, static_cast<int>(remaining.cloud->size() / 2)));
  std::vector<unsigned char> best_inliers;
  int best_count = 0;
  for (int iteration = 0; iteration < config.ransac_iterations; ++iteration) {
    const auto selected = drawDistinctTriple(rng_, static_cast<int>(remaining.cloud->size()));
    const std::vector<int> sample_indices(selected.begin(), selected.end());
    const auto candidate = fitPlanePca(*remaining.cloud, sample_indices);
    if (!candidate) continue;
    auto inliers = classifyInliers(*remaining.cloud, *candidate, config.inlier_threshold_mm);
    const int count = std::accumulate(inliers.begin(), inliers.end(), 0);
    if (count > best_count) {
      best_count = count;
      best_inliers = std::move(inliers);
    }
  }
  if (best_inliers.empty() || best_count < required) return std::nullopt;

  std::optional<FittedPlane> refined;
  for (int refinement = 0; refinement < 2; ++refinement) {
    refined = fitPlanePca(*remaining.cloud, indicesOf(best_inliers));
    if (!refined) return std::nullopt;
    best_inliers = classifyInliers(*remaining.cloud, *refined, config.inlier_threshold_mm);
  }
  const int count = std::accumulate(best_inliers.begin(), best_inliers.end(), 0);
  if (!refined || count < required) return std::nullopt;

  double squared_residual_sum = 0.0;
  for (std::size_t i = 0; i < remaining.cloud->size(); ++i) {
    if (!best_inliers[i]) continue;
    const double residual = pointPlaneDistance((*remaining.cloud)[i], *refined);
    squared_residual_sum += residual * residual;
  }
  PlaneModel result;
  result.normal = refined->normal;
  result.offset = refined->offset;
  result.sample_count = remaining.cloud->size();
  result.inlier_count = static_cast<std::size_t>(count);
  result.rms_mm = std::sqrt(squared_residual_sum / count);
  result.angle_deg = frontAngleDegrees(result.normal);
  return result;
}

void depthValidityMasks(const cv::Mat& depth_mm, const PlaneFitConfig& config,
                        cv::Mat& positive_depth, cv::Mat& in_range_depth) {
  CV_Assert(depth_mm.type() == CV_32FC1);
  cv::Mat finite;
  cv::compare(depth_mm, std::numeric_limits<float>::max(), finite, cv::CMP_LT);
  cv::compare(depth_mm, 0.0, positive_depth, cv::CMP_GT);
  cv::bitwise_and(positive_depth, finite, positive_depth);
  cv::inRange(depth_mm, config.min_depth_mm, config.max_depth_mm, in_range_depth);
  cv::bitwise_and(in_range_depth, finite, in_range_depth);
}

bool keepInstanceByDepth(const cv::Mat& mask, const cv::Mat& positive_depth,
                         const cv::Mat& in_range_depth,
                         const cv::Rect* known_mask_bounds) {
  CV_Assert(mask.type() == CV_8UC1 && mask.size() == positive_depth.size() &&
            mask.size() == in_range_depth.size());
  const cv::Rect bounds = known_mask_bounds != nullptr ? *known_mask_bounds
                                                       : cv::Rect(0, 0, mask.cols, mask.rows);
  if (bounds.empty()) return false;
  cv::Mat selected_positive;
  cv::bitwise_and(mask(bounds), positive_depth(bounds), selected_positive);
  const int valid_count = cv::countNonZero(selected_positive);
  if (valid_count < 30) return false;
  cv::Mat selected_in_range;
  cv::bitwise_and(mask(bounds), in_range_depth(bounds), selected_in_range);
  return static_cast<double>(cv::countNonZero(selected_in_range)) / valid_count >= 0.60;
}

DensePlaneResult densePlaneInliers(const cv::Mat& mask, const cv::Mat& depth_mm,
                                   const Intrinsics& intrinsics,
                                   const PlaneFitConfig& config,
                                   const PlaneModel& front_plane,
                                   const std::optional<PlaneModel>& competing_plane,
                                   const cv::Mat& valid_depth_in_range,
                                   const cv::Rect* known_mask_bounds,
                                   int known_mask_pixels) {
  CV_Assert(mask.type() == CV_8UC1 && depth_mm.type() == CV_32FC1 &&
            valid_depth_in_range.type() == CV_8UC1 && mask.size() == depth_mm.size());
  DensePlaneResult result;
  result.mask = cv::Mat::zeros(mask.size(), CV_8UC1);
  const cv::Rect bounds = known_mask_bounds != nullptr ? *known_mask_bounds
                                                       : cv::boundingRect(mask);
  if (bounds.empty() || known_mask_pixels == 0) return result;
  const int mask_pixels = known_mask_pixels >= 0 ? known_mask_pixels
                                                 : cv::countNonZero(mask(bounds));
  result.pixels.reserve(static_cast<std::size_t>(mask_pixels));
  result.depth_mm.reserve(result.pixels.capacity());

  for (int y = bounds.y; y < bounds.y + bounds.height; ++y) {
    const auto* mask_row = mask.ptr<unsigned char>(y);
    const auto* valid_row = valid_depth_in_range.ptr<unsigned char>(y);
    const auto* depth_row = depth_mm.ptr<float>(y);
    auto* result_row = result.mask.ptr<unsigned char>(y);
    for (int x = bounds.x; x < bounds.x + bounds.width; ++x) {
      if (!mask_row[x] || !valid_row[x]) continue;
      const float z = depth_row[x];
      const pcl::PointXYZ point(static_cast<float>((x - intrinsics.cx) * z / intrinsics.fx),
                                static_cast<float>((y - intrinsics.cy) * z / intrinsics.fy), z);
      const double front_distance = pointPlaneDistance(point, front_plane);
      if (front_distance > config.inlier_threshold_mm) continue;
      if (competing_plane && front_distance > pointPlaneDistance(point, *competing_plane)) continue;
      result_row[x] = 255;
      result.pixels.emplace_back(x, y);
      result.depth_mm.push_back(z);
    }
  }
  return result;
}

cv::Mat fillDensePlaneMaskHoles(const cv::Mat& dense_mask,
                                const cv::Rect* known_support_bounds) {
  CV_Assert(dense_mask.type() == CV_8UC1);
  static const cv::Mat kernel = cv::Mat::ones(5, 5, CV_8UC1);
  if (known_support_bounds != nullptr) {
    if (known_support_bounds->empty()) return cv::Mat::zeros(dense_mask.size(), CV_8UC1);

    // A 5x5 closing is one radius-2 dilation followed by one radius-2 erosion,
    // so every output pixel depends on input pixels at most four pixels away.
    constexpr int dependency_radius = 4;
    const cv::Rect image_bounds(0, 0, dense_mask.cols, dense_mask.rows);
    const cv::Rect work_bounds(
        known_support_bounds->x - dependency_radius,
        known_support_bounds->y - dependency_radius,
        known_support_bounds->width + 2 * dependency_radius,
        known_support_bounds->height + 2 * dependency_radius);
    const cv::Rect clipped_bounds = work_bounds & image_bounds;
    cv::Mat filled = cv::Mat::zeros(dense_mask.size(), CV_8UC1);
    cv::Mat filled_roi;
    cv::morphologyEx(dense_mask(clipped_bounds), filled_roi,
                     cv::MORPH_CLOSE, kernel);
    filled_roi.copyTo(filled(clipped_bounds));
    return filled;
  }

  cv::Mat filled;
  cv::morphologyEx(dense_mask, filled, cv::MORPH_CLOSE, kernel);
  return filled;
}

PoseResult estimateHorizontalPose(const DensePlaneResult& front_inliers,
                                  const Intrinsics& intrinsics,
                                  const PlaneModel& plane,
                                  const PlaneFitConfig& config) {
  if (front_inliers.pixels.size() < 3) return {std::nullopt, "no front-plane pixels"};
  int min_x = std::numeric_limits<int>::max();
  int max_x = std::numeric_limits<int>::min();
  for (const cv::Point& point : front_inliers.pixels) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
  }
  const int margin = config.image_border_margin_px;
  if (min_x <= margin || max_x >= front_inliers.mask.cols - 1 - margin) {
    return {std::nullopt, "horizontal border clipped"};
  }

  Eigen::Vector2d normal_h(plane.normal.x(), plane.normal.z());
  const double normal_length = normal_h.norm();
  if (normal_length < 1e-8) return {std::nullopt, "invalid horizontal normal"};
  normal_h /= normal_length;
  const double yaw_deg = std::atan2(normal_h.x(), normal_h.y()) * kRadiansToDegrees;
  const Eigen::Vector2d horizontal_axis(normal_h.y(), -normal_h.x());

  std::vector<Eigen::Vector2d> points_xz;
  std::vector<double> horizontal_coordinates;
  points_xz.reserve(front_inliers.pixels.size());
  horizontal_coordinates.reserve(front_inliers.pixels.size());
  for (std::size_t i = 0; i < front_inliers.pixels.size(); ++i) {
    const double z = front_inliers.depth_mm[i];
    const Eigen::Vector2d point((front_inliers.pixels[i].x - intrinsics.cx) * z / intrinsics.fx, z);
    points_xz.push_back(point);
    horizontal_coordinates.push_back(point.dot(horizontal_axis));
  }
  const auto [left_it, right_it] = std::minmax_element(horizontal_coordinates.begin(),
                                                       horizontal_coordinates.end());
  const double left_s = *left_it;
  const double right_s = *right_it;
  const double observed_width = right_s - left_s;
  if (std::abs(observed_width - config.known_width_mm) > config.width_tolerance_mm) {
    return {std::nullopt,
            "visible width " + std::to_string(static_cast<int>(std::round(observed_width))) +
                " mm is incomplete"};
  }

  const double center_s = (left_s + right_s) * 0.5;
  std::vector<cv::Point> center_pixels;
  std::vector<cv::Point> left_edge;
  std::vector<cv::Point> right_edge;
  std::vector<double> forward_coordinates;
  center_pixels.reserve(front_inliers.pixels.size());
  forward_coordinates.reserve(front_inliers.pixels.size());
  const double edge_band_mm = std::max(5.0, config.center_band_mm);
  for (std::size_t i = 0; i < horizontal_coordinates.size(); ++i) {
    const double coordinate = horizontal_coordinates[i];
    if (std::abs(coordinate - center_s) <= config.center_band_mm) {
      center_pixels.push_back(front_inliers.pixels[i]);
    }
    if (coordinate <= left_s + edge_band_mm) left_edge.push_back(front_inliers.pixels[i]);
    if (coordinate >= right_s - edge_band_mm) right_edge.push_back(front_inliers.pixels[i]);
    forward_coordinates.push_back(points_xz[i].dot(normal_h));
  }
  if (center_pixels.empty()) return {std::nullopt, "center is occluded"};

  const double forward_coordinate = median(std::move(forward_coordinates));
  const Eigen::Vector2d center_xz = horizontal_axis * center_s + normal_h * forward_coordinate;
  HorizontalPose pose;
  pose.x_right_mm = center_xz.x();
  pose.z_forward_mm = center_xz.y();
  pose.yaw_deg = yaw_deg;
  pose.observed_width_mm = observed_width;
  pose.center_pixel = cv::Point(static_cast<int>(std::round(medianCoordinate(center_pixels, true))),
                                static_cast<int>(std::round(medianCoordinate(center_pixels, false))));
  pose.left_edge_pixels = std::move(left_edge);
  pose.right_edge_pixels = std::move(right_edge);
  return {std::move(pose), "valid"};
}

Eigen::Vector3d rgbFrontHousingOrigin(const Eigen::Matrix3d& depth_to_rgb_rotation,
                                     const Eigen::Vector3d& depth_to_rgb_translation_mm,
                                     double depth_surface_z_mm) {
  const Eigen::Vector3d rgb_origin_in_depth =
      -depth_to_rgb_rotation.transpose() * depth_to_rgb_translation_mm;
  const Eigen::Vector3d rgb_forward_in_depth =
      depth_to_rgb_rotation.transpose() * Eigen::Vector3d(0.0, 0.0, 1.0);
  if (std::abs(rgb_forward_in_depth.z()) < 1e-8) {
    throw std::runtime_error("RGB optical axis is parallel to the front-housing plane");
  }
  const double distance_on_rgb_axis =
      (depth_surface_z_mm - rgb_origin_in_depth.z()) / rgb_forward_in_depth.z();
  return Eigen::Vector3d(0.0, 0.0, distance_on_rgb_axis);
}

double planeDistanceFromReference(const PlaneModel& plane,
                                  const Eigen::Vector3d& reference_point_rgb) {
  return std::abs(plane.normal.dot(reference_point_rgb) + plane.offset);
}

}  // namespace kfs
