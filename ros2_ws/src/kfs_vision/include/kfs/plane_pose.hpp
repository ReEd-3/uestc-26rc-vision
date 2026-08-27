#pragma once

#include <optional>
#include <random>

#include "kfs/types.hpp"

namespace kfs {

class FrontPlaneEstimator {
 public:
  FrontPlaneEstimator();

  SampledPoints samplePoints(const cv::Mat& mask, const cv::Mat& depth_mm,
                             const Intrinsics& intrinsics, const PlaneFitConfig& config,
                             const cv::Mat* valid_depth_in_range = nullptr,
                             const cv::Rect* known_mask_bounds = nullptr) const;

  std::optional<PlaneModel> estimate(const SampledPoints& samples,
                                     const PlaneFitConfig& config);

  std::optional<PlaneModel> estimateSecondaryPlane(const SampledPoints& samples,
                                                   const PlaneFitConfig& config,
                                                   const PlaneModel& front_plane);

 private:
  std::mt19937 rng_;
};

void depthValidityMasks(const cv::Mat& depth_mm, const PlaneFitConfig& config,
                        cv::Mat& positive_depth, cv::Mat& in_range_depth);

DepthGateStats inspectDepthGate(const cv::Mat& mask, const cv::Mat& depth_mm,
                                const PlaneFitConfig& config,
                                const cv::Rect* known_mask_bounds = nullptr);

bool keepInstanceByDepth(const cv::Mat& mask, const cv::Mat& positive_depth,
                         const cv::Mat& in_range_depth, const PlaneFitConfig& config,
                         const cv::Rect* known_mask_bounds = nullptr);

DensePlaneResult densePlaneInliers(const cv::Mat& mask, const cv::Mat& depth_mm,
                                   const Intrinsics& intrinsics,
                                   const PlaneFitConfig& config,
                                   const PlaneModel& front_plane,
                                   const std::optional<PlaneModel>& competing_plane,
                                   const cv::Mat& valid_depth_in_range,
                                   const cv::Rect* known_mask_bounds = nullptr,
                                   int known_mask_pixels = -1);

cv::Mat fillDensePlaneMaskHoles(const cv::Mat& dense_mask,
                                const cv::Rect* known_support_bounds = nullptr);

PoseResult estimateHorizontalPose(const DensePlaneResult& front_inliers,
                                  const Intrinsics& intrinsics,
                                  const PlaneModel& plane,
                                  const PlaneFitConfig& config);

Eigen::Vector3d rgbFrontHousingOrigin(const Eigen::Matrix3d& depth_to_rgb_rotation,
                                     const Eigen::Vector3d& depth_to_rgb_translation_mm,
                                     double depth_surface_z_mm);

double planeDistanceFromReference(const PlaneModel& plane,
                                  const Eigen::Vector3d& reference_point_rgb);

}  // namespace kfs
