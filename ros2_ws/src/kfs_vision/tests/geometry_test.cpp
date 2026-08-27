#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "kfs/config.hpp"
#include "kfs/plane_pose.hpp"

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void requireNear(double actual, double expected, double tolerance, const std::string& message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(message + ": got " + std::to_string(actual) +
                             ", expected " + std::to_string(expected));
  }
}

void testDepthGate() {
  cv::Mat mask(10, 10, CV_8UC1, cv::Scalar(255));
  cv::Mat positive(10, 10, CV_8UC1, cv::Scalar(255));
  cv::Mat in_range = cv::Mat::zeros(10, 10, CV_8UC1);
  kfs::PlaneFitConfig config;
  const cv::Rect known_bounds = cv::boundingRect(mask);
  in_range.rowRange(0, 6).setTo(255);
  require(kfs::keepInstanceByDepth(mask, positive, in_range, config),
          "exactly 60 percent in-range depth must pass");
  require(kfs::keepInstanceByDepth(mask, positive, in_range, config, &known_bounds),
          "known-bounds depth gate must preserve the passing result");
  in_range.at<unsigned char>(5, 9) = 0;
  require(!kfs::keepInstanceByDepth(mask, positive, in_range, config),
          "less than 60 percent in-range depth must fail");
  require(!kfs::keepInstanceByDepth(mask, positive, in_range, config, &known_bounds),
          "known-bounds depth gate must preserve the rejection result");

  config.min_depth_mm = 150;
  config.max_depth_mm = 1000;
  cv::Mat depth(10, 10, CV_32FC1, cv::Scalar(600.0F));
  depth.rowRange(0, 2).setTo(0.0F);
  depth.rowRange(2, 4).setTo(1200.0F);
  depth.rowRange(4, 6).setTo(100.0F);
  const kfs::DepthGateStats stats =
      kfs::inspectDepthGate(mask, depth, config, &known_bounds);
  require(stats.mask_count == 100 && stats.valid_count == 80 &&
              stats.in_range_count == 40 && stats.invalid_count == 20 &&
              stats.too_near_count == 20 && stats.too_far_count == 20 &&
              !stats.accepted,
          "depth-gate diagnostics must classify invalid, near, and far pixels");
  config.min_in_range_ratio = 0.50;
  require(kfs::inspectDepthGate(mask, depth, config, &known_bounds).accepted,
          "configured in-range ratio must control the depth gate");
  config.min_valid_depth_pixels = 81;
  require(!kfs::inspectDepthGate(mask, depth, config, &known_bounds).accepted,
          "configured valid-depth minimum must control the depth gate");
}

void testDepthGateConfigRoundTrip() {
  kfs::PlaneFitConfig configured;
  configured.min_valid_depth_pixels = 47;
  configured.min_in_range_ratio = 0.73;
  configured.temporal_max_forward_jump_mm = 41.0;
  configured.temporal_max_right_jump_mm = 42.0;
  configured.temporal_max_yaw_jump_deg = 8.0;
  configured.temporal_reset_after_ms = 450;
  const std::filesystem::path config_path =
      std::filesystem::temp_directory_path() / "kfs_geometry_depth_gate_config.json";
  std::filesystem::remove(config_path);
  kfs::savePlaneConfig(configured, config_path);
  const kfs::PlaneFitConfig loaded = kfs::loadPlaneConfig(config_path);
  std::filesystem::remove(config_path);
  require(loaded.min_valid_depth_pixels == 47,
          "depth-gate valid-pixel minimum must round-trip through JSON");
  requireNear(loaded.min_in_range_ratio, 0.73, 1e-12,
              "depth-gate in-range ratio must round-trip through JSON");
  requireNear(loaded.temporal_max_forward_jump_mm, 41.0, 1e-12,
              "temporal forward threshold must round-trip through JSON");
  requireNear(loaded.temporal_max_right_jump_mm, 42.0, 1e-12,
              "temporal right threshold must round-trip through JSON");
  requireNear(loaded.temporal_max_yaw_jump_deg, 8.0, 1e-12,
              "temporal yaw threshold must round-trip through JSON");
  require(loaded.temporal_reset_after_ms == 450,
          "temporal reset period must round-trip through JSON");
}

void testTemporalPoseGate() {
  kfs::PlaneFitConfig config;
  config.temporal_max_forward_jump_mm = 30.0;
  config.temporal_max_right_jump_mm = 30.0;
  config.temporal_max_yaw_jump_deg = 7.0;
  kfs::HorizontalPose previous;
  previous.z_forward_mm = 787.0;
  previous.x_right_mm = -220.0;
  previous.yaw_deg = -8.5;

  kfs::HorizontalPose stable = previous;
  stable.z_forward_mm += 20.0;
  stable.x_right_mm -= 15.0;
  stable.yaw_deg += 6.0;
  require(kfs::checkTemporalPose(stable, previous, config).accepted,
          "pose changes within all temporal limits must pass");

  kfs::HorizontalPose outlier = previous;
  outlier.z_forward_mm += 35.0;
  outlier.x_right_mm -= 40.0;
  outlier.yaw_deg += 20.0;
  const kfs::TemporalPoseCheck rejected =
      kfs::checkTemporalPose(outlier, previous, config);
  require(!rejected.accepted && rejected.forward_delta_mm == 35.0 &&
              rejected.right_delta_mm == 40.0 && rejected.yaw_delta_deg == 20.0,
          "pose jump beyond temporal limits must be rejected with measured deltas");

  config.temporal_max_yaw_jump_deg = 0.0;
  outlier = previous;
  outlier.yaw_deg = 351.5;
  require(kfs::checkTemporalPose(outlier, previous, config).accepted,
          "a zero temporal limit must disable that component gate");
}

kfs::SampledPoints twoPlaneCloud() {
  kfs::SampledPoints samples;
  for (int row = -2; row <= 2; ++row) {
    for (int column = -4; column <= 4; ++column) {
      samples.cloud->emplace_back(column * 35.0F, row * 35.0F, 700.0F);
      samples.xs.push_back(column + 100);
      samples.ys.push_back(row + 100);
    }
  }
  for (int row = -2; row <= 2; ++row) {
    for (int depth_index = 0; depth_index < 6; ++depth_index) {
      samples.cloud->emplace_back(180.0F, row * 35.0F, 735.0F + depth_index * 30.0F);
      samples.xs.push_back(depth_index + 200);
      samples.ys.push_back(row + 100);
    }
  }
  samples.cloud->width = static_cast<std::uint32_t>(samples.cloud->size());
  samples.cloud->height = 1;
  return samples;
}

void testFrontAndSecondaryPlaneChoice() {
  kfs::PlaneFitConfig config;
  config.ransac_iterations = 1000;
  config.inlier_threshold_mm = 1.0;
  config.min_inliers = 20;
  kfs::FrontPlaneEstimator estimator;
  const kfs::SampledPoints samples = twoPlaneCloud();
  const auto front = estimator.estimate(samples, config);
  require(front.has_value(), "front plane must be found");
  requireNear(front->normal.z(), 1.0, 1e-5, "most camera-facing plane normal");
  requireNear(front->offset, -700.0, 1e-3, "front plane offset");
  require(front->inlier_count == 45, "front plane support must contain the 45 front points");

  const auto side = estimator.estimateSecondaryPlane(samples, config, *front);
  require(side.has_value(), "secondary plane must be found");
  requireNear(std::abs(side->normal.x()), 1.0, 1e-5, "secondary plane normal");
  require(side->inlier_count == 30, "secondary plane support must contain the 30 side points");
}

void testDensePose() {
  constexpr int width = 640;
  constexpr int height = 480;
  kfs::Intrinsics intrinsics{600.0, 600.0, 320.0, 240.0};
  kfs::PlaneFitConfig config;
  config.min_depth_mm = 150;
  config.max_depth_mm = 1000;
  config.inlier_threshold_mm = 17.0;
  config.known_width_mm = 350.0;
  config.width_tolerance_mm = 1.0;
  config.center_band_mm = 12.0;

  cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
  mask(cv::Rect(170, 180, 301, 121)).setTo(255);
  cv::Mat depth(height, width, CV_32FC1, cv::Scalar(700.0F));
  cv::Mat positive;
  cv::Mat in_range;
  kfs::depthValidityMasks(depth, config, positive, in_range);

  kfs::PlaneModel front;
  front.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  front.offset = -700.0;
  const cv::Rect known_bounds = cv::boundingRect(mask);
  const int known_pixels = cv::countNonZero(mask);
  kfs::FrontPlaneEstimator sampler;
  const kfs::SampledPoints sampled = sampler.samplePoints(
      mask, depth, intrinsics, config, &in_range, &known_bounds);
  const kfs::SampledPoints reference_sampled = sampler.samplePoints(
      mask, depth, intrinsics, config, &in_range);
  require(sampled.xs == reference_sampled.xs && sampled.ys == reference_sampled.ys,
          "known mask bounds must preserve sampled pixel coordinates");
  require(sampled.cloud->size() == reference_sampled.cloud->size(),
          "known mask bounds must preserve the sampled cloud size");
  for (std::size_t index = 0; index < sampled.cloud->size(); ++index) {
    const auto& actual = (*sampled.cloud)[index];
    const auto& expected = (*reference_sampled.cloud)[index];
    require(actual.x == expected.x && actual.y == expected.y && actual.z == expected.z,
            "known mask bounds must preserve sampled 3D coordinates");
  }
  const kfs::DensePlaneResult dense = kfs::densePlaneInliers(
      mask, depth, intrinsics, config, front, std::nullopt, in_range,
      &known_bounds, known_pixels);
  const kfs::DensePlaneResult reference_dense = kfs::densePlaneInliers(
      mask, depth, intrinsics, config, front, std::nullopt, in_range);
  require(cv::countNonZero(dense.mask != reference_dense.mask) == 0,
          "known mask metadata must preserve the dense inlier mask");
  require(dense.pixels == reference_dense.pixels,
          "known mask metadata must preserve dense pixel ordering");
  require(dense.depth_mm == reference_dense.depth_mm,
          "known mask metadata must preserve dense depths");
  require(dense.pixels.size() == 301U * 121U,
          "dense classification must keep every true front-plane pixel");
  const kfs::PoseResult result = kfs::estimateHorizontalPose(
      dense, intrinsics, front, config);
  require(result.pose.has_value(), "complete 350 mm front face must produce a pose");
  requireNear(result.pose->x_right_mm, 0.0, 1e-6, "pose X");
  requireNear(result.pose->z_forward_mm, 700.0, 1e-6, "pose Z");
  requireNear(result.pose->yaw_deg, 0.0, 1e-6, "pose yaw");
  requireNear(result.pose->observed_width_mm, 350.0, 1e-6, "observed width");
  require(result.pose->center_pixel.x == 320, "pose center pixel must remain in full-image coordinates");
}

void requireRoiClosingMatchesFull(const cv::Mat& mask, const cv::Rect& support_bounds,
                                  const std::string& scenario) {
  const cv::Mat reference = kfs::fillDensePlaneMaskHoles(mask);
  const cv::Mat optimized = kfs::fillDensePlaneMaskHoles(mask, &support_bounds);
  require(cv::countNonZero(reference != optimized) == 0,
          "ROI morphology must match full-frame morphology: " + scenario);
}

void testRoiMorphologyEquivalence() {
  cv::Mat interior = cv::Mat::zeros(60, 80, CV_8UC1);
  interior(cv::Rect(24, 18, 25, 21)).setTo(255);
  interior(cv::Rect(34, 26, 2, 2)).setTo(0);
  requireRoiClosingMatchesFull(interior, cv::Rect(20, 14, 34, 30),
                              "interior support with a hole");

  cv::Mat top_left = cv::Mat::zeros(60, 80, CV_8UC1);
  top_left(cv::Rect(0, 0, 19, 17)).setTo(255);
  top_left(cv::Rect(7, 7, 2, 2)).setTo(0);
  requireRoiClosingMatchesFull(top_left, cv::Rect(0, 0, 23, 21),
                              "support touching top-left image borders");

  cv::Mat bottom_right = cv::Mat::zeros(60, 80, CV_8UC1);
  bottom_right(cv::Rect(61, 43, 19, 17)).setTo(255);
  bottom_right(cv::Rect(70, 51, 2, 2)).setTo(0);
  requireRoiClosingMatchesFull(bottom_right, cv::Rect(57, 39, 23, 21),
                              "support touching bottom-right image borders");
}

void testReferenceOrigin() {
  const Eigen::Vector3d origin = kfs::rgbFrontHousingOrigin(
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), 4.930);
  requireNear(origin.x(), 0.0, 1e-12, "reference origin X");
  requireNear(origin.y(), 0.0, 1e-12, "reference origin Y");
  requireNear(origin.z(), 4.930, 1e-12, "reference origin Z");
}

}  // namespace

int main() {
  try {
    testDepthGate();
    testDepthGateConfigRoundTrip();
    testTemporalPoseGate();
    testFrontAndSecondaryPlaneChoice();
    testDensePose();
    testRoiMorphologyEquivalence();
    testReferenceOrigin();
    std::cout << "geometry_test: all checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "geometry_test failed: " << error.what() << '\n';
    return 1;
  }
}
