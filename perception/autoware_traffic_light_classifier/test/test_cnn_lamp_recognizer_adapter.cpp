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

// cspell:ignore comlops

//
// ROS adapter test for CnnLampRecognizer.
//
// Recognition/decode behavior is covered ROS-free in test_cnn_lamp_recognizer (against
// CnnLampRecognizerCore). This file pins the concerns that live in the ROS adapter and cannot
// move to the core:
//   - the images/signals size guard (getTrafficSignals takes a caller-owned signals array), and
//   - the per-image -> per-signal-slot scatter of the assembled getTrafficSignals path.
// Parameter declaration is exercised implicitly: the adapter ctor reads every model_params.*
// parameter, so a wrong name/type would fail construction here (-> skip with a reason).
//
// Constructing CnnLampRecognizer builds the TensorRT engine (its ctor needs a GPU + the ONNX
// model under autoware_data), so this test self-skips (GTEST_SKIP) when the GPU or model is
// unavailable, and builds the engine once per suite (SetUpTestSuite; that build takes minutes).
// Model outputs are NOT pinned (TensorRT output is not bit-reproducible); assertions are limited
// to model-stable structure, with a pure-black slot standing in for a guaranteed no-detection
// (no model detects a lamp in black).
//
// Tests follow Arrange-Act-Assert.
//

#include "../src/classifier/cnn_lamp_recognizer.hpp"
#include "../src/classifier_params.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/cuda_utils/cuda_gtest_utils.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
namespace tl = autoware::traffic_light;

using tier4_perception_msgs::msg::TrafficLightArray;
using tier4_perception_msgs::msg::TrafficLightElement;

// Lamp recognizer model, downloaded under autoware_data by the ansible artifacts role:
// https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v4/traffic_light_lamp_recognizer_comlops.onnx
constexpr char model_filename[] = "traffic_light_lamp_recognizer_comlops.onnx";

// YOLO-head layout from lamp_recognizer_ml.param.yaml, the artifact the launch stack feeds the
// node (same ansible role as the model above):
// https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v4/lamp_recognizer_ml.param.yaml
// The ctor declares each of these without a default, so all must be present.
struct LampModelParam
{
  const char * name;
  int value;
};
const std::vector<LampModelParam> lamp_model_params{
  {"model_params.num_anchors", 3}, {"model_params.chans_per_anchor", 16},
  {"model_params.x_index", 0},     {"model_params.y_index", 1},
  {"model_params.w_index", 2},     {"model_params.h_index", 3},
  {"model_params.obj_index", 4},   {"model_params.color_start", 5},
  {"model_params.type_start", 8},  {"model_params.num_types", 6},
  {"model_params.num_colors", 3},  {"model_params.cos_index", 14},
  {"model_params.sin_index", 15}};
constexpr double scale_x_y = 2.0;
// (w, h) per anchor; size must equal 2 * num_anchors.
const std::vector<double> anchors{7.0, 7.0, 14.0, 14.0, 42.0, 42.0};

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

// Load the normal-traffic-light test crop as RGB (imread yields BGR; the node feeds the
// classifier RGB). Throws on a missing file so a setup error fails loudly instead of classifying
// an empty Mat.
cv::Mat load_rgb_crop()
{
  const std::string path =
    ament_index_cpp::get_package_share_directory("autoware_traffic_light_classifier") +
    "/test_data/traffic_light_normal.png";
  const cv::Mat bgr = cv::imread(path, cv::IMREAD_COLOR);
  if (bgr.empty()) {
    throw std::runtime_error("failed to load test image: " + path);
  }
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return rgb;
}

// Builds the real CnnLampRecognizer adapter once for the whole suite (the TensorRT engine build
// is minutes-long), through a node + the real ROS parameters -- so the ctor's parameter
// declaration is exercised end to end. When the model or a usable GPU is missing, recognizer_
// stays null and skip_reason_ explains why; each test then GTEST_SKIPs.
class CnnLampRecognizerAdapterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    const std::string model_path = resolve_autoware_data_file(model_filename);
    if (model_path.empty()) {
      skip_reason_ = "lamp recognizer model not found under autoware_data";
      return;
    }
    if (!autoware::cuda_utils::is_cuda_runtime_available()) {
      skip_reason_ = "CUDA runtime / GPU not available";
      return;
    }

    rclcpp::NodeOptions options;
    options.append_parameter_override("model_path", model_path);
    options.append_parameter_override("precision", std::string("fp16"));
    options.append_parameter_override("score_threshold", 0.2);
    options.append_parameter_override("nms_threshold", 0.2);
    options.append_parameter_override("max_batch_size", 64);
    for (const auto & p : lamp_model_params) {
      options.append_parameter_override(p.name, p.value);
    }
    options.append_parameter_override("model_params.scale_x_y", scale_x_y);
    options.append_parameter_override("model_params.anchors", anchors);

    // Node creation can fail on a misconfigured RMW/DDS environment, and the CnnLampRecognizer
    // ctor builds the TensorRT engine and throws on GPU/TRT failure. Treat all of these as
    // "environment unavailable" -> skip (not fail), with a reason each test reports via
    // GTEST_SKIP.
    try {
      node_ = std::make_shared<rclcpp::Node>("cnn_lamp_recognizer_adapter_test", options);
      recognizer_ =
        std::make_shared<tl::CnnLampRecognizer>(node_.get(), tl::declare_lamp_config(node_.get()));
    } catch (const std::exception & e) {
      skip_reason_ = std::string("CnnLampRecognizer environment unavailable: ") + e.what();
      recognizer_.reset();
      node_.reset();
    }
  }

  static void TearDownTestSuite()
  {
    recognizer_.reset();
    node_.reset();
  }

  static inline std::string skip_reason_;
  static inline std::shared_ptr<rclcpp::Node> node_;
  static inline std::shared_ptr<tl::CnnLampRecognizer> recognizer_;
};

// A mismatched images/signals count is rejected before any inference (the guard stays in the ROS
// adapter after the core/adapter split; getTrafficSignals writes into the caller-owned slots, so
// a count mismatch would otherwise be an out-of-range access). Mirrors the equivalent guard test
// in test_cnn_classifier_adapter.cpp.
TEST_F(CnnLampRecognizerAdapterTest, MismatchedImageSignalCountReturnsFalse)
{
  if (!recognizer_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange -- one image but two signal slots. The guard returns before inference, so a dummy
  // image (never classified) is enough.
  const std::vector<cv::Mat> images{cv::Mat(4, 4, CV_8UC3, cv::Scalar(0, 0, 0))};
  TrafficLightArray signals;
  signals.signals.resize(2);

  // Act
  const bool ok = recognizer_->getTrafficSignals(images, signals);

  // Assert
  EXPECT_FALSE(ok);
}

// getTrafficSignals must scatter each image's result into its own signal slot. A detected crop in
// slot 0 and a black (guaranteed no-detection) image in slot 1 make the slots asymmetric, so a
// swapped or collapsed mapping is caught: slot 0 keeps its detection while slot 1 falls back to
// the single UNKNOWN placeholder. Colors/counts of the detection are left to the ROS-free core
// tests -- only the model-stable structure and the empty-slot placeholder are asserted here.
TEST_F(CnnLampRecognizerAdapterTest, BatchClassificationScattersPerImageResults)
{
  if (!recognizer_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange -- detected crop in slot 0, a black (featureless) image in slot 1.
  const cv::Mat detected{load_rgb_crop()};
  const cv::Mat black{cv::Mat::zeros(detected.size(), CV_8UC3)};
  TrafficLightArray signals;
  signals.signals.resize(2);

  // Act
  const bool ok = recognizer_->getTrafficSignals({detected, black}, signals);

  // Assert -- one signal per input, scattered to the correct slot.
  ASSERT_TRUE(ok);
  ASSERT_EQ(signals.signals.size(), 2u);
  // Slot 0: the crop detects, so at least one element (non-vacuous; contents are the core's job).
  EXPECT_FALSE(signals.signals[0].elements.empty());
  // Slot 1: black yields no detection -> exactly one UNKNOWN placeholder with zero confidence.
  ASSERT_EQ(signals.signals[1].elements.size(), 1u);
  EXPECT_EQ(signals.signals[1].elements[0].color, TrafficLightElement::UNKNOWN);
  EXPECT_EQ(signals.signals[1].elements[0].shape, TrafficLightElement::UNKNOWN);
  EXPECT_FLOAT_EQ(signals.signals[1].elements[0].confidence, 0.0f);
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
