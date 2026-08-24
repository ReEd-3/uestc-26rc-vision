#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "kfs_vision/target_message.hpp"

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void requireNear(float actual, float expected, float tolerance,
                 const std::string& message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(message + ": got " + std::to_string(actual) +
                             ", expected " + std::to_string(expected));
  }
}

kfs::Measurement validMeasurement() {
  kfs::Measurement measurement;
  measurement.name = "Blue KFS";
  measurement.plane = kfs::PlaneModel{};
  measurement.pose = kfs::HorizontalPose{};
  measurement.pose->x_right_mm = 1000.0;
  measurement.pose->z_forward_mm = 2500.0;
  measurement.pose->yaw_deg = 180.0;
  return measurement;
}

void testBlueMessageAndUnits() {
  const kfs::SegmentationDetection detection{
      0, 0.9F, cv::Mat{}, 0, cv::Rect{}};
  const auto message =
      kfs_vision::makeTargetMessage(detection, validMeasurement());
  require(message.has_value(), "valid blue result must create a message");
  require(message->color == custom_msgs::msg::KfsTarget::BLUE,
          "class 0 must map to BLUE");
  requireNear(message->x_m, 1.0F, 1e-6F, "x millimetres to metres");
  requireNear(message->y_m, 2.5F, 1e-6F, "y millimetres to metres");
  requireNear(message->yaw_rad, 3.1415927F, 1e-6F,
              "yaw degrees to radians");
}

void testRedMessage() {
  const kfs::SegmentationDetection detection{
      1, 0.9F, cv::Mat{}, 0, cv::Rect{}};
  const auto message =
      kfs_vision::makeTargetMessage(detection, validMeasurement());
  require(message.has_value(), "valid red result must create a message");
  require(message->color == custom_msgs::msg::KfsTarget::RED,
          "class 1 must map to RED");
}

void testMissingPlaneOrPose() {
  const kfs::SegmentationDetection detection{
      0, 0.9F, cv::Mat{}, 0, cv::Rect{}};
  kfs::Measurement missing_plane = validMeasurement();
  missing_plane.plane.reset();
  require(!kfs_vision::makeTargetMessage(detection, missing_plane),
          "missing plane must suppress publication");

  kfs::Measurement missing_pose = validMeasurement();
  missing_pose.pose.reset();
  require(!kfs_vision::makeTargetMessage(detection, missing_pose),
          "missing pose must suppress publication");
}

void testInvalidClass() {
  const kfs::SegmentationDetection detection{
      2, 0.9F, cv::Mat{}, 0, cv::Rect{}};
  require(!kfs_vision::makeTargetMessage(detection, validMeasurement()),
          "unknown class must suppress publication");
}

void testNonFinitePose() {
  const kfs::SegmentationDetection detection{
      0, 0.9F, cv::Mat{}, 0, cv::Rect{}};

  kfs::Measurement measurement = validMeasurement();
  measurement.pose->x_right_mm = std::numeric_limits<double>::quiet_NaN();
  require(!kfs_vision::makeTargetMessage(detection, measurement),
          "NaN x must suppress publication");

  measurement = validMeasurement();
  measurement.pose->z_forward_mm = std::numeric_limits<double>::infinity();
  require(!kfs_vision::makeTargetMessage(detection, measurement),
          "infinite y must suppress publication");

  measurement = validMeasurement();
  measurement.pose->yaw_deg = std::numeric_limits<double>::quiet_NaN();
  require(!kfs_vision::makeTargetMessage(detection, measurement),
          "NaN yaw must suppress publication");
}

}  // namespace

int main() {
  try {
    testBlueMessageAndUnits();
    testRedMessage();
    testMissingPlaneOrPose();
    testInvalidClass();
    testNonFinitePose();
    std::cout << "target_message_test passed\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "target_message_test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}

