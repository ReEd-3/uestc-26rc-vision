#include "kfs_vision/kfs_vision_node.hpp"

#include <libobsensor/ObSensor.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "kfs/debug_view.hpp"
#include "kfs/plane_pose.hpp"
#include "kfs/yolo_seg.hpp"
#include "kfs_vision/target_message.hpp"

namespace kfs_vision {
namespace {

using Clock = std::chrono::steady_clock;

class PipelineGuard {
 public:
  explicit PipelineGuard(std::shared_ptr<ob::Pipeline> pipeline)
      : pipeline_(std::move(pipeline)) {}

  ~PipelineGuard() {
    if (!started_) return;
    try {
      pipeline_->stop();
    } catch (...) {
    }
  }

  void markStarted() { started_ = true; }

  void stop() {
    if (!started_) return;
    pipeline_->stop();
    started_ = false;
  }

 private:
  std::shared_ptr<ob::Pipeline> pipeline_;
  bool started_ = false;
};

class ProcessingRateTracker {
 public:
  explicit ProcessingRateTracker(std::chrono::milliseconds reporting_period)
      : reporting_period_(reporting_period) {}

  bool observeFrame(Clock::time_point completed_at) {
    if (!window_started_) {
      window_started_ = completed_at;
      return false;
    }

    ++completed_intervals_;
    const auto elapsed = completed_at - *window_started_;
    if (elapsed < reporting_period_) return false;

    const double elapsed_seconds =
        std::chrono::duration<double>(elapsed).count();
    processing_fps_ = static_cast<double>(completed_intervals_) / elapsed_seconds;
    window_started_ = completed_at;
    completed_intervals_ = 0;
    return true;
  }

  [[nodiscard]] double processingFps() const { return processing_fps_; }

 private:
  std::chrono::milliseconds reporting_period_;
  std::optional<Clock::time_point> window_started_;
  std::size_t completed_intervals_ = 0;
  double processing_fps_ = 0.0;
};

void recordStage(std::map<std::string, double>& stage_ema,
                 const std::string& name,
                 Clock::time_point started) {
  const double elapsed_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - started).count();
  const auto it = stage_ema.find(name);
  stage_ema[name] =
      it == stage_ema.end() ? elapsed_ms : 0.9 * it->second + 0.1 * elapsed_ms;
}

void recordDuration(std::map<std::string, double>& stage_ema,
                    const std::string& name,
                    double elapsed_ms) {
  const auto it = stage_ema.find(name);
  stage_ema[name] =
      it == stage_ema.end() ? elapsed_ms : 0.9 * it->second + 0.1 * elapsed_ms;
}

double timingValue(const std::map<std::string, double>& timings,
                   const char* name) {
  const auto it = timings.find(name);
  return it == timings.end() ? 0.0 : it->second;
}

std::shared_ptr<ob::VideoStreamProfile> getColorProfile(
    const std::shared_ptr<ob::Pipeline>& pipeline,
    const kfs::AppConfig& config) {
  const auto profiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
  try {
    return profiles->getVideoStreamProfile(
        config.frame_width, config.frame_height, OB_FORMAT_RGB, config.fps);
  } catch (const ob::Error&) {
    try {
      return profiles->getVideoStreamProfile(
          OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_RGB, OB_FPS_ANY);
    } catch (const ob::Error&) {
      throw std::runtime_error("camera has no RGB color stream profile");
    }
  }
}

bool deviceListContainsUid(const std::shared_ptr<ob::DeviceList>& devices,
                           const std::string& expected_uid) {
  if (!devices) return false;
  for (std::uint32_t index = 0; index < devices->getCount(); ++index) {
    const char* uid = devices->getUid(index);
    if (uid && expected_uid == uid) return true;
  }
  return false;
}

Eigen::Matrix3d rotationMatrix(const OBD2CTransform& transform) {
  Eigen::Matrix3d rotation;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rotation(row, column) = transform.rot[row * 3 + column];
    }
  }
  return rotation;
}

const char* messageColorName(std::uint8_t color) {
  if (color == custom_msgs::msg::KfsTarget::BLUE) return "BLUE";
  if (color == custom_msgs::msg::KfsTarget::RED) return "RED";
  return "UNKNOWN";
}

std::string terminalStatusLine(
    double processing_fps,
    const kfs::RuntimeDebug& debug,
    const std::vector<kfs::Measurement>& measurements,
    const std::map<std::string, double>& timings,
    const std::optional<custom_msgs::msg::KfsTarget>& published_message,
    const std::string& failure_reason) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(1)
         << "process_fps=" << processing_fps;

  if (published_message) {
    stream << " target=valid"
           << " color=" << messageColorName(published_message->color)
           << std::setprecision(3)
           << " x=" << published_message->x_m << "m"
           << " y=" << published_message->y_m << "m"
           << " yaw=" << published_message->yaw_rad << "rad";
  } else {
    stream << " target=" << debug.target_state;
    if (!failure_reason.empty()) stream << " reason=" << failure_reason;
    stream << " class=" << debug.class_name
           << std::setprecision(3) << " conf=" << debug.confidence;
  }

  stream << std::setprecision(1)
         << " mask=" << debug.mask_pixels
         << " bbox=" << debug.bbox_w << 'x' << debug.bbox_h
         << " samples=" << debug.sample_count
         << " plane=" << debug.plane_state;

  if (!measurements.empty() && measurements.front().plane) {
    const kfs::PlaneModel& plane = *measurements.front().plane;
    stream << " angle=" << plane.angle_deg << "deg"
           << " rms=" << plane.rms_mm << "mm"
           << " inliers=" << plane.inlier_count << '/' << plane.sample_count;
  }

  if (!measurements.empty() && measurements.front().pose) {
    const kfs::HorizontalPose& pose = *measurements.front().pose;
    stream << " pose_x=" << pose.x_right_mm << "mm"
           << " pose_z=" << pose.z_forward_mm << "mm"
           << " pose_yaw=" << pose.yaw_deg << "deg"
           << " width=" << pose.observed_width_mm << "mm";
  }

  stream << " | yolo=" << timingValue(timings, "yolo") << "ms"
         << " infer=" << timingValue(timings, "yolo_infer") << "ms"
         << " align=" << timingValue(timings, "align_depth") << "ms"
         << " dense=" << timingValue(timings, "dense_pose") << "ms"
         << " frame_cv=" << timingValue(timings, "frame_to_cv") << "ms";
  return stream.str();
}

std::filesystem::path requiredInputPath(
    const std::string& configured_path,
    const std::filesystem::path& default_path,
    const char* parameter_name) {
  std::filesystem::path result =
      configured_path.empty() ? default_path : std::filesystem::path(configured_path);
  if (!result.is_absolute()) {
    throw std::invalid_argument(std::string(parameter_name) +
                                " must be an absolute path when overridden");
  }
  if (!std::filesystem::is_regular_file(result)) {
    throw std::runtime_error(std::string(parameter_name) +
                             " is not a regular file: " + result.string());
  }
  return result;
}

std::optional<std::filesystem::path> optionalOutputPath(
    const std::string& configured_path,
    const char* parameter_name) {
  if (configured_path.empty()) return std::nullopt;
  std::filesystem::path result(configured_path);
  if (!result.is_absolute()) {
    throw std::invalid_argument(std::string(parameter_name) +
                                " must be an absolute path when set");
  }
  return result;
}

}  // namespace

KfsVisionNode::KfsVisionNode() : rclcpp::Node("kfs_vision_node") {
  const std::filesystem::path package_share =
      ament_index_cpp::get_package_share_directory("kfs_vision");

  const std::string model_path =
      declare_parameter<std::string>("model_path", "");
  const std::string plane_config_path =
      declare_parameter<std::string>("plane_config_path", "");
  const std::string plane_config_output_path =
      declare_parameter<std::string>("plane_config_output_path", "");
  const std::string target_topic =
      declare_parameter<std::string>("target_topic", "kfs/target");

  app_config_.frame_width = declare_parameter<int>("frame_width", 1280);
  app_config_.frame_height = declare_parameter<int>("frame_height", 720);
  app_config_.fps = declare_parameter<int>("camera_fps", 30);
  app_config_.yolo_confidence = static_cast<float>(
      declare_parameter<double>("yolo_confidence", 0.50));
  app_config_.front_housing_from_depth_zero_mm = declare_parameter<double>(
      "front_housing_from_depth_zero_mm", 4.930);
  show_gui_ = declare_parameter<bool>("show_gui", false);
  const int terminal_period_ms =
      declare_parameter<int>("terminal_period_ms", 1000);

  if (app_config_.frame_width <= 0 || app_config_.frame_height <= 0 ||
      app_config_.fps <= 0) {
    throw std::invalid_argument(
        "frame_width, frame_height and camera_fps must be greater than zero");
  }
  if (!(app_config_.yolo_confidence >= 0.0F &&
        app_config_.yolo_confidence <= 1.0F)) {
    throw std::invalid_argument("yolo_confidence must be in [0, 1]");
  }
  if (terminal_period_ms <= 0) {
    throw std::invalid_argument("terminal_period_ms must be greater than zero");
  }
  if (target_topic.empty()) {
    throw std::invalid_argument("target_topic must not be empty");
  }

  terminal_period_ = std::chrono::milliseconds(terminal_period_ms);
  app_config_.model_path = requiredInputPath(
      model_path, package_share / "models" / "exp.onnx", "model_path");
  app_config_.plane_config_path = requiredInputPath(
      plane_config_path, package_share / "config" / "kfs_plane_fit.json",
      "plane_config_path");
  plane_config_output_path_ = optionalOutputPath(
      plane_config_output_path, "plane_config_output_path");

  target_publisher_ = create_publisher<custom_msgs::msg::KfsTarget>(
      target_topic, rclcpp::QoS(1).reliable().durability_volatile());

  RCLCPP_INFO(get_logger(), "model=%s", app_config_.model_path.c_str());
  RCLCPP_INFO(get_logger(), "plane_config=%s",
              app_config_.plane_config_path.c_str());
  if (plane_config_output_path_) {
    RCLCPP_INFO(get_logger(), "plane_config_output=%s",
                plane_config_output_path_->c_str());
  } else if (show_gui_) {
    RCLCPP_INFO(get_logger(),
                "GUI plane controls are runtime-only; plane_config_output_path is empty");
  }
  RCLCPP_INFO(get_logger(), "color=%dx%d@%d depth=aligned-to-color",
              app_config_.frame_width, app_config_.frame_height, app_config_.fps);
  RCLCPP_INFO(get_logger(), "topic=%s", target_publisher_->get_topic_name());
}

KfsVisionNode::~KfsVisionNode() { stop(); }

void KfsVisionNode::start() {
  if (processing_thread_.joinable()) {
    throw std::logic_error("kfs_vision processing thread is already running");
  }
  stop_requested_.store(false);
  runtime_failed_.store(false);
  processing_thread_ = std::thread(&KfsVisionNode::processingLoop, this);
}

void KfsVisionNode::stop() {
  stop_requested_.store(true);
  if (processing_thread_.joinable()) processing_thread_.join();
}

bool KfsVisionNode::runtimeFailed() const noexcept {
  return runtime_failed_.load();
}

void KfsVisionNode::processingLoop() noexcept {
  try {
    runPipeline();
  } catch (const ob::Error& error) {
    runtime_failed_.store(true);
    RCLCPP_ERROR(get_logger(), "OrbbecSDK error: %s", error.what());
  } catch (const Ort::Exception& error) {
    runtime_failed_.store(true);
    RCLCPP_ERROR(get_logger(), "ONNX Runtime error: %s", error.what());
  } catch (const std::exception& error) {
    runtime_failed_.store(true);
    RCLCPP_ERROR(get_logger(), "vision pipeline error: %s", error.what());
  } catch (...) {
    runtime_failed_.store(true);
    RCLCPP_ERROR(get_logger(), "vision pipeline failed with an unknown error");
  }

  cv::destroyAllWindows();
  if (!stop_requested_.load() && rclcpp::ok()) rclcpp::shutdown();
}

void KfsVisionNode::runPipeline() {
  app_config_.plane = kfs::loadPlaneConfig(app_config_.plane_config_path);

  RCLCPP_INFO(get_logger(), "loading ONNX model");
  kfs::OnnxYoloSegmenter model(
      app_config_.model_path, app_config_.yolo_confidence);
  RCLCPP_INFO(get_logger(), "inference_provider=%s",
              model.executionProvider().c_str());

  // OrbbecSDK otherwise writes ./Log/OrbbecSDK.log.txt relative to the
  // caller's working directory. Diagnostics from this node stay in the ROS
  // terminal, so disable only the SDK file sink before creating its context.
  ob::Context::setLoggerToFile(OB_LOG_SEVERITY_OFF, "");
  ob::Context context;
  if (context.queryDeviceList()->getCount() == 0) {
    throw std::runtime_error("no Orbbec camera detected");
  }

  auto pipeline = std::make_shared<ob::Pipeline>();
  const auto selected_device = pipeline->getDevice();
  const auto selected_device_info = selected_device->getDeviceInfo();
  const char* selected_uid_value = selected_device_info->getUid();
  if (!selected_uid_value || selected_uid_value[0] == '\0') {
    throw std::runtime_error("Orbbec camera has no device UID");
  }
  const std::string selected_uid(selected_uid_value);
  const char* selected_serial_value = selected_device_info->getSerialNumber();
  const std::string selected_serial =
      selected_serial_value ? selected_serial_value : "unknown";
  auto camera_disconnected = std::make_shared<std::atomic_bool>(false);
  context.registerDeviceChangedCallback(
      [camera_disconnected, selected_uid](
          const std::shared_ptr<ob::DeviceList>& removed_devices,
          const std::shared_ptr<ob::DeviceList>&) {
        try {
          if (deviceListContainsUid(removed_devices, selected_uid)) {
            camera_disconnected->store(true);
          }
        } catch (...) {
          camera_disconnected->store(true);
        }
      });
  RCLCPP_INFO(get_logger(), "camera_uid=%s serial=%s", selected_uid.c_str(),
              selected_serial.c_str());

  auto camera_config = std::make_shared<ob::Config>();
  camera_config->enableStream(getColorProfile(pipeline, app_config_));
  camera_config->enableStream(
      pipeline->getStreamProfileList(OB_SENSOR_DEPTH)->getProfile(0));
  camera_config->setFrameAggregateOutputMode(
      OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
  auto align_filter = std::make_shared<ob::Align>(OB_STREAM_COLOR);
  align_filter->setMatchTargetResolution(true);
  auto hole_filler = std::make_shared<ob::HoleFillingFilter>();

  PipelineGuard pipeline_guard(pipeline);
  pipeline->start(camera_config);
  pipeline_guard.markStarted();

  constexpr const char* kWindowName = "Depth Seg";
  std::optional<kfs::OpenCvControls> controls;
  if (show_gui_) {
    cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(kWindowName, app_config_.frame_width,
                     app_config_.frame_height / 2);
    controls.emplace(app_config_.plane, plane_config_output_path_);
    controls->create();
  }

  kfs::FrontPlaneEstimator plane_estimator;
  std::optional<kfs::Intrinsics> intrinsics;
  std::optional<Eigen::Vector3d> front_housing_origin_rgb;
  std::map<std::string, double> stage_ema;
  ProcessingRateTracker processing_rate_tracker(terminal_period_);
  auto next_debug_refresh = Clock::now();
  constexpr auto kDebugRefreshPeriod =
      std::chrono::milliseconds(67);  // About 15 Hz.
  constexpr int kMaximumConsecutiveFrameTimeouts = 3;
  int consecutive_frame_timeouts = 0;

  while (rclcpp::ok() && !stop_requested_.load()) {
    if (controls) controls->pollAndSave();

    auto stage_started = Clock::now();
    auto frame_set = pipeline->waitForFrames(1000);
    recordStage(stage_ema, "wait_frames", stage_started);
    if (camera_disconnected->load()) {
      throw std::runtime_error("Orbbec camera disconnected");
    }
    if (!frame_set) {
      if (!deviceListContainsUid(context.queryDeviceList(), selected_uid)) {
        throw std::runtime_error("Orbbec camera disconnected");
      }
      ++consecutive_frame_timeouts;
      if (consecutive_frame_timeouts >= kMaximumConsecutiveFrameTimeouts) {
        throw std::runtime_error(
            "Orbbec camera produced no frames for 3 consecutive seconds");
      }
      RCLCPP_WARN(get_logger(), "camera frame timeout (%d/%d)",
                  consecutive_frame_timeouts,
                  kMaximumConsecutiveFrameTimeouts);
      continue;
    }
    consecutive_frame_timeouts = 0;

    stage_started = Clock::now();
    auto aligned_frame = align_filter->process(frame_set);
    recordStage(stage_ema, "align_depth", stage_started);
    if (!aligned_frame) {
      throw std::runtime_error("depth alignment returned no frame");
    }
    auto aligned_set = aligned_frame->as<ob::FrameSet>();
    if (!aligned_set) {
      throw std::runtime_error("depth alignment returned a non-frameset result");
    }
    auto color_frame = aligned_set->getColorFrame();
    auto depth_frame = aligned_set->getDepthFrame();
    if (!color_frame || !depth_frame) {
      throw std::runtime_error(
          "aligned frameset is missing color or depth data");
    }

    stage_started = Clock::now();
    auto filled_frame = hole_filler->process(depth_frame);
    if (filled_frame) depth_frame = filled_frame->as<ob::DepthFrame>();
    recordStage(stage_ema, "fill_depth", stage_started);

    stage_started = Clock::now();
    const int width = static_cast<int>(color_frame->getWidth());
    const int height = static_cast<int>(color_frame->getHeight());
    if (color_frame->getFormat() != OB_FORMAT_RGB) {
      throw std::runtime_error("color frame format changed from RGB");
    }
    cv::Mat rgb_view(height, width, CV_8UC3, color_frame->getData());
    cv::Mat frame;
    if (show_gui_) cv::cvtColor(rgb_view, frame, cv::COLOR_RGB2BGR);

    const int depth_width = static_cast<int>(depth_frame->getWidth());
    const int depth_height = static_cast<int>(depth_frame->getHeight());
    if (depth_width != width || depth_height != height) {
      throw std::runtime_error(
          "aligned depth size does not match the color frame");
    }
    cv::Mat depth_raw(
        depth_height, depth_width, CV_16UC1, depth_frame->getData());
    cv::Mat depth_mm;
    depth_raw.convertTo(
        depth_mm, CV_32FC1, depth_frame->getValueScale());
    cv::Mat positive_depth;
    cv::Mat in_range_depth;
    kfs::depthValidityMasks(
        depth_mm, app_config_.plane, positive_depth, in_range_depth);
    recordStage(stage_ema, "frame_to_cv", stage_started);

    if (!intrinsics) {
      const OBCameraParam camera_parameters = pipeline->getCameraParam();
      intrinsics = kfs::Intrinsics{
          camera_parameters.rgbIntrinsic.fx,
          camera_parameters.rgbIntrinsic.fy,
          camera_parameters.rgbIntrinsic.cx,
          camera_parameters.rgbIntrinsic.cy};
      const Eigen::Matrix3d rotation =
          rotationMatrix(camera_parameters.transform);
      const Eigen::Vector3d translation(
          camera_parameters.transform.trans[0],
          camera_parameters.transform.trans[1],
          camera_parameters.transform.trans[2]);
      front_housing_origin_rgb = kfs::rgbFrontHousingOrigin(
          rotation, translation,
          app_config_.front_housing_from_depth_zero_mm);
      RCLCPP_INFO(
          get_logger(),
          "RGB intrinsics: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
          intrinsics->fx, intrinsics->fy, intrinsics->cx, intrinsics->cy);
      RCLCPP_INFO(
          get_logger(),
          "reference_origin_rgb_mm: x=%.3f y=%.3f z=%.3f",
          front_housing_origin_rgb->x(), front_housing_origin_rgb->y(),
          front_housing_origin_rgb->z());
    }

    stage_started = Clock::now();
    const auto detection = model.inferBestRgb(rgb_view);
    recordStage(stage_ema, "yolo", stage_started);
    const kfs::YoloTimings& yolo_timings = model.lastTimings();
    recordDuration(stage_ema, "yolo_pre", yolo_timings.preprocess_ms);
    recordDuration(stage_ema, "yolo_infer", yolo_timings.inference_ms);
    recordDuration(stage_ema, "yolo_decode", yolo_timings.decode_ms);
    recordDuration(stage_ema, "yolo_mask", yolo_timings.mask_ms);

    cv::Mat combined_inliers;
    std::vector<kfs::Measurement> measurements;
    kfs::RuntimeDebug runtime_debug;
    runtime_debug.output_fps = processing_rate_tracker.processingFps();
    runtime_debug.timings = &stage_ema;
    runtime_debug.plane_cfg = &app_config_.plane;
    runtime_debug.target_state = "none";
    std::string failure_reason = "no detection";
    std::optional<custom_msgs::msg::KfsTarget> published_message;

    if (detection) {
      runtime_debug.target_state = "invalid";
      runtime_debug.class_name = kfs::className(detection->class_id);
      runtime_debug.confidence = detection->confidence;
      const int mask_pixels = detection->mask_pixels;
      const cv::Rect& mask_bounds = detection->mask_bounds;
      runtime_debug.mask_pixels = mask_pixels;
      runtime_debug.bbox_w = mask_bounds.width;
      runtime_debug.bbox_h = mask_bounds.height;

      stage_started = Clock::now();
      if (!kfs::keepInstanceByDepth(
              detection->mask, positive_depth, in_range_depth, &mask_bounds)) {
        failure_reason = "depth rejected";
        recordStage(stage_ema, "mask_and_gate", stage_started);
      } else {
        recordStage(stage_ema, "mask_and_gate", stage_started);
        stage_started = Clock::now();
        const kfs::SampledPoints samples = plane_estimator.samplePoints(
            detection->mask, depth_mm, *intrinsics, app_config_.plane,
            &in_range_depth, &mask_bounds);
        auto front_plane =
            plane_estimator.estimate(samples, app_config_.plane);
        runtime_debug.sample_count = samples.cloud->size();
        runtime_debug.plane_state = front_plane ? "OK" : "FAILED";
        recordStage(stage_ema, "front_plane", stage_started);

        kfs::Measurement measurement;
        measurement.name = kfs::className(detection->class_id);
        measurement.plane = front_plane;
        measurement.pose_reason = "front plane fit failed";
        failure_reason = measurement.pose_reason;

        if (front_plane) {
          stage_started = Clock::now();
          measurement.side_plane = plane_estimator.estimateSecondaryPlane(
              samples, app_config_.plane, *front_plane);
          recordStage(stage_ema, "side_plane", stage_started);

          stage_started = Clock::now();
          auto dense_stage_started = Clock::now();
          const kfs::DensePlaneResult dense = kfs::densePlaneInliers(
              detection->mask, depth_mm, *intrinsics, app_config_.plane,
              *front_plane, measurement.side_plane, in_range_depth,
              &mask_bounds, mask_pixels);
          recordStage(stage_ema, "dense_classify", dense_stage_started);

          dense_stage_started = Clock::now();
          const kfs::PoseResult pose = kfs::estimateHorizontalPose(
              dense, *intrinsics, *front_plane, app_config_.plane);
          recordStage(stage_ema, "dense_estimate", dense_stage_started);
          measurement.pose = pose.pose;
          measurement.pose_reason = pose.reason;
          failure_reason = pose.reason;

          dense_stage_started = Clock::now();
          if (show_gui_) {
            combined_inliers =
                kfs::fillDensePlaneMaskHoles(dense.mask, &mask_bounds);
          }
          recordStage(stage_ema, "dense_close", dense_stage_started);
          recordStage(stage_ema, "dense_pose", stage_started);
        }

        measurements.push_back(std::move(measurement));
        published_message =
            makeTargetMessage(*detection, measurements.front());
        if (published_message) {
          runtime_debug.target_state = "valid";
          failure_reason.clear();
          target_publisher_->publish(*published_message);
        } else if (measurements.front().pose) {
          failure_reason = "message fields are invalid";
        }
      }
    }

    if (show_gui_) {
      if (combined_inliers.empty()) {
        combined_inliers = cv::Mat::zeros(height, width, CV_8UC1);
      }
      const auto display_started = Clock::now();
      if (display_started >= next_debug_refresh) {
        stage_started = Clock::now();
        cv::Mat debug_view = kfs::buildDebugView(
            frame, combined_inliers, measurements, runtime_debug);
        recordStage(stage_ema, "build_mosaic", stage_started);
        cv::imshow(kWindowName, debug_view);
        stage_started = Clock::now();
        const int key = cv::pollKey();
        recordStage(stage_ema, "display", stage_started);
        next_debug_refresh = display_started + kDebugRefreshPeriod;
        if ((key & 0xff) == 'q') {
          RCLCPP_INFO(get_logger(), "q pressed; shutting down");
          rclcpp::shutdown();
        }
      } else {
        recordDuration(stage_ema, "build_mosaic", 0.0);
        recordDuration(stage_ema, "display", 0.0);
      }
    }

    if (processing_rate_tracker.observeFrame(Clock::now())) {
      const std::string line = terminalStatusLine(
          processing_rate_tracker.processingFps(), runtime_debug, measurements,
          stage_ema, published_message, failure_reason);
      RCLCPP_INFO(get_logger(), "%s", line.c_str());
    }
  }

  pipeline_guard.stop();
  if (show_gui_) cv::destroyAllWindows();
}

}  // namespace kfs_vision
