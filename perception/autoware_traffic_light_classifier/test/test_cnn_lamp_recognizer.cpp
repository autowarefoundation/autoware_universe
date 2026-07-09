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
// Characterization test for CnnLampRecognizer.
//
// CnnLampRecognizer is a YOLO-style TensorRT lamp detector, so it needs a GPU and the ONNX
// model (traffic_light_lamp_recognizer_comlops.onnx, under autoware_data). This pins the
// CURRENT end-to-end behavior of getTrafficSignals against the real model, as a safety net
// before the planned Node/core split of cnn_lamp_recognizer.{hpp,cpp}. It self-skips
// (GTEST_SKIP) when the GPU or model is unavailable, builds the engine once per suite
// (SetUpTestSuite; that build takes minutes), and asserts coarsely because TensorRT output
// is not bit-reproducible across GPU / TRT versions. The decode geometry comes from
// model_params.*, which the launch stack feeds from lamp_recognizer_ml.param.yaml; the same
// values are hard-coded below (lamp_model_params) so the ctor's declare_parameter calls
// succeed.
//
// SCOPE: the green-only test_data crops exercise only the simplest outputs -- one element per
// ROI, either a green/amber CIRCLE when the lamp is detected or the UNKNOWN no-detection
// placeholder when it is not. The remaining decode branches (arrow directions and the
// near-zero-vector guard, pedestrian / cross / red, NMS de-duplication) are NOT covered here;
// after the split they belong in a GPU-free core unit test.
//
// Inputs are the green ROI crops in test/test_data, converted BGR->RGB to match the node's RGB
// input. Per-crop expectations are pinned as OBSERVED, not as semantically correct; each test's
// comment records what its crop decodes to and how the assertion is shaped.
//
// Tests follow Arrange-Act-Assert.
//

#include "../src/classifier/cnn_lamp_recognizer.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/cuda_utils/cuda_gtest_utils.hpp>
#include <opencv2/opencv.hpp>
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

// YOLO-head layout from lamp_recognizer_ml.param.yaml, the artifact the launch stack
// feeds the node (same ansible role as the model above):
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

// Resolve a file shipped under autoware_data. The on-disk layout differs between
// setups (canonical autoware_data/ml_models/traffic_light_classifier vs. the legacy
// flat autoware_data/traffic_light_classifier); an explicit TLC_TEST_DATA_DIR override
// takes precedence. Returns "" when not found.
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

// Load a test_data ROI crop as RGB (cv::imread yields BGR; the node feeds the
// classifier RGB). Throws on a missing file so a setup error fails the test clearly
// rather than silently classifying an empty Mat.
cv::Mat load_rgb_crop(const std::string & name)
{
  const std::string path =
    ament_index_cpp::get_package_share_directory("autoware_traffic_light_classifier") +
    "/test_data/" + name;
  const cv::Mat bgr = cv::imread(path, cv::IMREAD_COLOR);
  if (bgr.empty()) {
    throw std::runtime_error("failed to load test image: " + path);
  }
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return rgb;
}

// Builds the real CnnLampRecognizer once for the whole suite (the TensorRT engine build
// is minutes-long). When the model or a usable GPU is missing, recognizer_ stays null
// and skip_reason_ explains why; each test then GTEST_SKIPs.
class CnnLampRecognizerCharacterizationTest : public ::testing::Test
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

    // Node creation can fail on a misconfigured RMW/DDS environment, and the
    // CnnLampRecognizer ctor builds the TensorRT engine and throws on GPU/TRT failure.
    // Treat all of these as "environment unavailable" -> skip (not fail), with a reason
    // each test reports via GTEST_SKIP.
    try {
      node_ = std::make_shared<rclcpp::Node>("cnn_lamp_recognizer_characterization_test", options);
      recognizer_ = std::make_shared<tl::CnnLampRecognizer>(node_.get());
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

// Each single-crop test below pins the CURRENT observed decode of one green traffic-light
// ROI through the full path: preprocess -> TRT inference -> YOLO decode -> NMS -> element.
// A refactor that perturbs the pipeline is caught by the pinned outputs. SHAPE is the robust
// axis: every DETECTED crop decodes to a CIRCLE (a stable argmax), so shape is asserted exactly
// (the strongly-dimmed crop is the exception -- no detection -> UNKNOWN, see its test). COLOR
// is NOT robust -- the per-lamp color head reads the same physically-green lamp as amber at
// full brightness but green once dimmed (see each test's value), so the pinned color is an
// as-observed value, not a wide-margin argmax; under fp16 it is the assertion most likely to
// shift across GPU / TRT versions. Confidence is only bounded, for the same reason.

// The bright, un-dimmed crop decodes to a single AMBER CIRCLE element (the color head reads
// this green lamp as amber; pinned as-observed, observed confidence ~0.98).
TEST_F(CnnLampRecognizerCharacterizationTest, ClassifiesNormalGreenCrop)
{
  if (!recognizer_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange
  const std::vector<cv::Mat> images{load_rgb_crop("traffic_light_normal.png")};
  TrafficLightArray signals;
  signals.signals.resize(images.size());

  // Act
  const bool ok = recognizer_->getTrafficSignals(images, signals);

  // Assert
  ASSERT_TRUE(ok);
  ASSERT_EQ(signals.signals.size(), 1u);
  ASSERT_EQ(signals.signals[0].elements.size(), 1u);
  const auto & element = signals.signals[0].elements[0];
  EXPECT_EQ(element.color, TrafficLightElement::AMBER);
  EXPECT_EQ(element.shape, TrafficLightElement::CIRCLE);
  EXPECT_GT(element.confidence, 0.0f);
  EXPECT_LE(element.confidence, 1.0f);
}

// The weakly-dimmed crop decodes to a single GREEN CIRCLE element (observed confidence ~0.86).
TEST_F(CnnLampRecognizerCharacterizationTest, ClassifiesWeaklyDimmedGreenCrop)
{
  if (!recognizer_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange
  const std::vector<cv::Mat> images{load_rgb_crop("traffic_light_dimmed_weak.png")};
  TrafficLightArray signals;
  signals.signals.resize(images.size());

  // Act
  const bool ok = recognizer_->getTrafficSignals(images, signals);

  // Assert
  ASSERT_TRUE(ok);
  ASSERT_EQ(signals.signals.size(), 1u);
  ASSERT_EQ(signals.signals[0].elements.size(), 1u);
  const auto & element = signals.signals[0].elements[0];
  EXPECT_EQ(element.color, TrafficLightElement::GREEN);
  EXPECT_EQ(element.shape, TrafficLightElement::CIRCLE);
  EXPECT_GT(element.confidence, 0.0f);
  EXPECT_LE(element.confidence, 1.0f);
}

// The medium-dimmed crop decodes to a single GREEN CIRCLE element (observed confidence ~0.61).
TEST_F(CnnLampRecognizerCharacterizationTest, ClassifiesMediumDimmedGreenCrop)
{
  if (!recognizer_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange
  const std::vector<cv::Mat> images{load_rgb_crop("traffic_light_dimmed_medium.png")};
  TrafficLightArray signals;
  signals.signals.resize(images.size());

  // Act
  const bool ok = recognizer_->getTrafficSignals(images, signals);

  // Assert
  ASSERT_TRUE(ok);
  ASSERT_EQ(signals.signals.size(), 1u);
  ASSERT_EQ(signals.signals[0].elements.size(), 1u);
  const auto & element = signals.signals[0].elements[0];
  EXPECT_EQ(element.color, TrafficLightElement::GREEN);
  EXPECT_EQ(element.shape, TrafficLightElement::CIRCLE);
  EXPECT_GT(element.confidence, 0.0f);
  EXPECT_LE(element.confidence, 1.0f);
}

// The strongly-dimmed crop drops below score_threshold, so nothing is detected. The
// empty-detection branch of updateTrafficSignals then emits a single UNKNOWN element
// with zero confidence -- pinning that the "no lamp found" path yields exactly one
// placeholder element rather than an empty list.
TEST_F(CnnLampRecognizerCharacterizationTest, StronglyDimmedCropYieldsUnknownNoDetection)
{
  if (!recognizer_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange
  const std::vector<cv::Mat> images{load_rgb_crop("traffic_light_dimmed_strong.png")};
  TrafficLightArray signals;
  signals.signals.resize(images.size());

  // Act
  const bool ok = recognizer_->getTrafficSignals(images, signals);

  // Assert
  ASSERT_TRUE(ok);
  ASSERT_EQ(signals.signals.size(), 1u);
  ASSERT_EQ(signals.signals[0].elements.size(), 1u);
  const auto & element = signals.signals[0].elements[0];
  EXPECT_EQ(element.color, TrafficLightElement::UNKNOWN);
  EXPECT_EQ(element.shape, TrafficLightElement::UNKNOWN);
  EXPECT_FLOAT_EQ(element.confidence, 0.0f);
}

// Two ROIs in one call must scatter to their own signal slots (getTrafficSignals loops
// images into batches and writes signals[signal_i + i] per image). Pairing the normal
// crop (detected -> AMBER CIRCLE) with the strongly-dimmed crop (no detection -> UNKNOWN)
// gives the slots distinct outputs, so a swapped mapping or a collapse to one image is
// caught directly by the per-slot color/shape (not just by element counts).
TEST_F(CnnLampRecognizerCharacterizationTest, BatchClassificationScattersPerImageResults)
{
  if (!recognizer_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange -- normal (detected) in slot 0, strongly-dimmed (undetected) in slot 1.
  const cv::Mat detected = load_rgb_crop("traffic_light_normal.png");
  const cv::Mat undetected = load_rgb_crop("traffic_light_dimmed_strong.png");
  TrafficLightArray batched;
  batched.signals.resize(2);

  // Act
  const bool ok = recognizer_->getTrafficSignals({detected, undetected}, batched);

  // Assert -- slot 0 keeps the AMBER CIRCLE detection, slot 1 the UNKNOWN placeholder.
  ASSERT_TRUE(ok);
  ASSERT_EQ(batched.signals.size(), 2u);
  ASSERT_EQ(batched.signals[0].elements.size(), 1u);
  ASSERT_EQ(batched.signals[1].elements.size(), 1u);
  EXPECT_EQ(batched.signals[0].elements[0].color, TrafficLightElement::AMBER);
  EXPECT_EQ(batched.signals[0].elements[0].shape, TrafficLightElement::CIRCLE);
  EXPECT_EQ(batched.signals[1].elements[0].color, TrafficLightElement::UNKNOWN);
  EXPECT_EQ(batched.signals[1].elements[0].shape, TrafficLightElement::UNKNOWN);
}

// A mismatched images/signals count is rejected before any inference (the guard stays
// in the ROS adapter after the core/adapter split).
TEST_F(CnnLampRecognizerCharacterizationTest, MismatchedImageSignalCountReturnsFalse)
{
  if (!recognizer_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange -- one image but two signal slots.
  const std::vector<cv::Mat> images{load_rgb_crop("traffic_light_normal.png")};
  TrafficLightArray signals;
  signals.signals.resize(2);

  // Act
  const bool ok = recognizer_->getTrafficSignals(images, signals);

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
