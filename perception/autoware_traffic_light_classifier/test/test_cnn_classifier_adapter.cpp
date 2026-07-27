// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// ROS adapter test for CNNClassifier.
//
// Classification behavior is covered ROS-free in test_cnn_classifier (against
// CNNClassifierCore::classify). This file pins the one adapter concern that cannot move
// to the core because CNNClassifier::getTrafficSignals takes a caller-owned signals
// array: the images/signals size guard. The guard protects the element-merge loop from
// an out-of-range access when the caller's signal count disagrees with the images, so a
// regression here is a memory-safety bug, not just a wrong return value.
//
// Unlike test_color_classifier_adapter, constructing CNNClassifier builds the TensorRT
// engine (its ctor needs a GPU + the ONNX model downloaded under autoware_data), so this
// test self-skips (GTEST_SKIP) when the GPU or model is unavailable.
//
// Tests follow Arrange-Act-Assert.
//

#include "../src/classifier/cnn_classifier.hpp"
#include "../src/classifier_params.hpp"

#include <autoware/cuda_utils/cuda_gtest_utils.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_perception_msgs/msg/traffic_light_array.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace
{
namespace tl = autoware::traffic_light;

using tier4_perception_msgs::msg::TrafficLightArray;

// CAR CNN classifier artifacts, downloaded under autoware_data (see the ansible
// artifacts role).
constexpr char model_filename[] = "traffic_light_classifier_mobilenetv2_batch_1.onnx";
constexpr char label_filename[] = "lamp_labels.txt";

// Normalization from config/car_traffic_light_classifier.param.yaml (ROS params load as
// std::vector<double>).
const std::vector<double> default_mean{123.675, 116.28, 103.53};
const std::vector<double> default_std{58.395, 57.12, 57.375};

// Resolve a file shipped under autoware_data. The on-disk layout differs between setups
// (canonical autoware_data/ml_models/traffic_light_classifier vs. the legacy flat
// autoware_data/traffic_light_classifier); an explicit TLC_TEST_DATA_DIR override takes
// precedence. Returns "" when not found.
std::string resolve_autoware_data_file(const std::string & filename)
{
  std::vector<std::string> candidate_dirs;
  if (const char * override_dir = std::getenv("TLC_TEST_DATA_DIR")) {
    candidate_dirs.emplace_back(override_dir);
  }
  if (const char * home = std::getenv("HOME")) {
    candidate_dirs.emplace_back(
      std::string(home) + "/autoware_data/ml_models/traffic_light_classifier");
    candidate_dirs.emplace_back(std::string(home) + "/autoware_data/traffic_light_classifier");
  }
  for (const auto & dir : candidate_dirs) {
    const std::string candidate = dir + "/" + filename;
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return "";
}

// Builds the real CNNClassifier once for the whole suite (the TensorRT engine build is
// minutes-long). When the model or a usable GPU is missing, classifier_ stays null and
// skip_reason_ explains why; the test then GTEST_SKIPs.
class CnnClassifierAdapterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    const std::string model_path = resolve_autoware_data_file(model_filename);
    const std::string label_path = resolve_autoware_data_file(label_filename);
    if (model_path.empty() || label_path.empty()) {
      skip_reason_ = "CNN model/label not found under autoware_data";
      return;
    }
    if (!autoware::cuda_utils::is_cuda_runtime_available()) {
      skip_reason_ = "CUDA runtime / GPU not available";
      return;
    }

    rclcpp::NodeOptions options;
    options.append_parameter_override("precision", std::string("fp16"));
    options.append_parameter_override("model_path", model_path);
    options.append_parameter_override("label_path", label_path);
    options.append_parameter_override("mean", default_mean);
    options.append_parameter_override("std", default_std);

    // Node creation can fail on a misconfigured RMW/DDS environment, and the CNNClassifier
    // ctor builds the TensorRT engine and throws on GPU/TRT failure. Treat all of these as
    // "environment unavailable" -> skip (not fail), with a reason the test reports.
    try {
      node_ = std::make_shared<rclcpp::Node>("cnn_classifier_adapter_test", options);
      classifier_ = std::make_shared<tl::CNNClassifier>(tl::declare_cnn_config(node_.get()));
    } catch (const std::exception & e) {
      skip_reason_ = std::string("CNNClassifier environment unavailable: ") + e.what();
      classifier_.reset();
      node_.reset();
    }
  }

  static void TearDownTestSuite()
  {
    classifier_.reset();
    node_.reset();
  }

  static inline std::string skip_reason_;
  static inline std::shared_ptr<rclcpp::Node> node_;
  static inline std::shared_ptr<tl::CNNClassifier> classifier_;
};

// A mismatched images/signals count is rejected before any inference (and before any
// per-slot access in the merge loop). Mirrors the equivalent guard test in
// test_color_classifier_adapter.cpp.
TEST_F(CnnClassifierAdapterTest, MismatchedImageSignalCountReturnsFalse)
{
  if (!classifier_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange -- one image but two signal slots. The guard returns before inference, so a
  // dummy image (never classified) is enough.
  const std::vector<cv::Mat> images{cv::Mat(4, 4, CV_8UC3, cv::Scalar(0, 0, 0))};
  TrafficLightArray signals;
  signals.signals.resize(2);

  // Act
  const bool ok = classifier_->getTrafficSignals(images, signals);

  // Assert
  EXPECT_FALSE(ok);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
