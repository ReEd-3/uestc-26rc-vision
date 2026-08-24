#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core.hpp>

#include "kfs/yolo_seg.hpp"

namespace {

std::filesystem::path parseModelPath(int argc, char** argv) {
  if (argc == 1) {
    return std::filesystem::path(
               ament_index_cpp::get_package_share_directory("kfs_vision")) /
           "models" / "exp.onnx";
  }
  if (argc == 3 && std::string(argv[1]) == "--model") return argv[2];
  throw std::invalid_argument(
      std::string("usage: ") + argv[0] + " [--model /absolute/path/to/model.onnx]");
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const std::filesystem::path model_path = parseModelPath(argc, argv);
    if (!model_path.is_absolute()) {
      throw std::invalid_argument("model override path must be absolute");
    }
    std::cout << "Loading ONNX model: " << model_path << '\n';
    kfs::OnnxYoloSegmenter model(model_path, 0.50F);
    const cv::Mat black_frame(720, 1280, CV_8UC3, cv::Scalar::all(0));
    (void)model.inferBest(black_frame);
    std::cout << "Inference provider: " << model.executionProvider() << '\n'
              << "CUDA ONNX model check passed (session creation + one inference).\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Model check failed: " << error.what() << '\n';
    return 1;
  }
}

