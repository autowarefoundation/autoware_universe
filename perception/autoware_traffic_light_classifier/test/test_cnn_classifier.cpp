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
// Tests for CNNClassifierCore, the Node-free CNN classification core.
//
// Two groups, both ROS-free (constructed directly from a CNNConfig, no node):
//
//  - The static helpers decode_label() and make_debug_image() depend only on the
//    perception message types + OpenCV, so they run everywhere -- no GPU, no model.
//
//  - The ctor builds a TensorRT engine and classify() runs inference, so both need
//    a GPU + the ONNX model (downloaded under autoware_data). The
//    CnnClassifierCoreClassifyTest fixture below exercises that path against the real
//    model and self-skips (GTEST_SKIP) when the GPU or model is unavailable. Asserts
//    are coarse because TensorRT output is not bit-reproducible across GPU / TRT
//    versions, and the engine is built once per suite (SetUpTestSuite) since that build
//    takes minutes.
//
// Tests follow Arrange-Act-Assert.
//

#include "../src/classifier/cnn_classifier.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/cuda_utils/cuda_gtest_utils.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <tier4_perception_msgs/msg/traffic_light.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
namespace tl = autoware::traffic_light;

using tier4_perception_msgs::msg::TrafficLight;
using tier4_perception_msgs::msg::TrafficLightElement;

// ===================== static helpers (no GPU, no model) =====================

// A single "color-shape" token decodes to one element with both fields mapped.
TEST(CnnClassifierCoreDecodeLabelTest, ColorShapeToken)
{
  // Arrange / Act
  const auto elements = tl::CNNClassifierCore::decode_label("red-cross", 0.5f);

  // Assert
  ASSERT_EQ(elements.size(), 1u);
  EXPECT_EQ(elements[0].color, TrafficLightElement::RED);
  EXPECT_EQ(elements[0].shape, TrafficLightElement::CROSS);
}

// Comma-separated tokens decode to one element each, and the per-token branches differ:
// a bare color implies a CIRCLE shape while a "color-shape" token maps the shape. This
// pins both the comma loop and the per-token branch selection in one case.
TEST(CnnClassifierCoreDecodeLabelTest, CommaSeparatedLampsDecodePerToken)
{
  // Arrange / Act
  const auto elements = tl::CNNClassifierCore::decode_label("red,red-right", 0.5f);

  // Assert
  ASSERT_EQ(elements.size(), 2u);
  EXPECT_EQ(elements[0].color, TrafficLightElement::RED);
  EXPECT_EQ(elements[0].shape, TrafficLightElement::CIRCLE);
  EXPECT_EQ(elements[1].color, TrafficLightElement::RED);
  EXPECT_EQ(elements[1].shape, TrafficLightElement::RIGHT_ARROW);
}

// A bare "unknown" token maps both color and shape to UNKNOWN.
TEST(CnnClassifierCoreDecodeLabelTest, UnknownToken)
{
  // Arrange / Act
  const auto elements = tl::CNNClassifierCore::decode_label("unknown", 0.5f);

  // Assert
  ASSERT_EQ(elements.size(), 1u);
  EXPECT_EQ(elements[0].color, TrafficLightElement::UNKNOWN);
  EXPECT_EQ(elements[0].shape, TrafficLightElement::UNKNOWN);
}

// A bare color token implies a CIRCLE shape.
TEST(CnnClassifierCoreDecodeLabelTest, BareColorImpliesCircle)
{
  // Arrange / Act
  const auto elements = tl::CNNClassifierCore::decode_label("green", 0.5f);

  // Assert
  ASSERT_EQ(elements.size(), 1u);
  EXPECT_EQ(elements[0].color, TrafficLightElement::GREEN);
  EXPECT_EQ(elements[0].shape, TrafficLightElement::CIRCLE);
}

// A bare shape token (no color prefix) defaults the color to GREEN -- the surprising
// fall-through branch worth pinning.
TEST(CnnClassifierCoreDecodeLabelTest, BareShapeDefaultsToGreen)
{
  // Arrange / Act
  const auto elements = tl::CNNClassifierCore::decode_label("right", 0.5f);

  // Assert
  ASSERT_EQ(elements.size(), 1u);
  EXPECT_EQ(elements[0].color, TrafficLightElement::GREEN);
  EXPECT_EQ(elements[0].shape, TrafficLightElement::RIGHT_ARROW);
}

// The passed confidence is propagated to every decoded lamp element.
TEST(CnnClassifierCoreDecodeLabelTest, ConfidencePropagatedToEveryLamp)
{
  // Arrange
  const float confidence = 0.75f;

  // Act
  const auto elements = tl::CNNClassifierCore::decode_label("red,green-right,unknown", confidence);

  // Assert
  ASSERT_EQ(elements.size(), 3u);
  for (const auto & element : elements) {
    EXPECT_FLOAT_EQ(element.confidence, confidence);
  }
}

// make_debug_image lays out the ROI resized to a fixed 200px width above a 50px text
// strip. A non-square ROI makes an accidental rows/cols swap detectable.
TEST(CnnClassifierCoreDebugImageTest, MosaicGeometry)
{
  // Arrange -- distinct width and height; one element so the label strip is exercised.
  const int roi_width = 100;
  const int roi_height = 50;
  const cv::Mat roi(roi_height, roi_width, CV_8UC3, cv::Scalar(0, 255, 0));
  TrafficLight signal;
  TrafficLightElement element;
  element.color = TrafficLightElement::GREEN;
  element.shape = TrafficLightElement::CIRCLE;
  element.confidence = 0.9f;
  signal.elements.push_back(element);

  // Act
  const cv::Mat debug_image = tl::CNNClassifierCore::make_debug_image(roi, signal);

  // Assert -- width fixed at 200, height = scaled ROI (200*50/100 == 100) + 50px strip.
  EXPECT_EQ(debug_image.cols, 200);
  EXPECT_EQ(debug_image.rows, 100 + 50);
  EXPECT_EQ(debug_image.type(), CV_8UC3);
}

// make_debug_image returns a fresh Mat and does not mutate the input ROI.
TEST(CnnClassifierCoreDebugImageTest, DoesNotMutateInput)
{
  // Arrange
  const int roi_width = 100;
  const int roi_height = 50;
  cv::Mat roi(roi_height, roi_width, CV_8UC3, cv::Scalar(0, 255, 0));
  TrafficLight signal;
  TrafficLightElement element;
  element.color = TrafficLightElement::GREEN;
  element.shape = TrafficLightElement::CIRCLE;
  element.confidence = 0.9f;
  signal.elements.push_back(element);

  // Act
  tl::CNNClassifierCore::make_debug_image(roi, signal);

  // Assert
  EXPECT_EQ(roi.cols, roi_width);
  EXPECT_EQ(roi.rows, roi_height);
}

// ================== GPU-gated tests of the TensorRT classify() path ==================

// CAR CNN classifier artifacts, downloaded under autoware_data (see the ansible
// artifacts role). The model's static batch size is 1.
constexpr char model_filename[] = "traffic_light_classifier_mobilenetv2_batch_1.onnx";
constexpr char label_filename[] = "lamp_labels.txt";

// Normalization from config/car_traffic_light_classifier.param.yaml. CNNConfig takes
// std::vector<float> directly (the ROS adapter narrows from the double params).
const std::vector<float> default_mean{123.675f, 116.28f, 103.53f};
const std::vector<float> default_std{58.395f, 57.12f, 57.375f};

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

// Read the label file into per-line labels (the core takes the pre-read lines so it does
// no file I/O of its own -- the ROS adapter normally does this read).
std::vector<std::string> read_label_lines(const std::string & path)
{
  std::ifstream labels_file(path);
  std::vector<std::string> labels;
  std::string label;
  while (std::getline(labels_file, label)) {
    labels.push_back(label);
  }
  return labels;
}

// Load the green traffic-light ROI crop from test_data as RGB -- the only crop the tests
// use (cv::imread yields BGR; the node feeds the classifier RGB). Throws on a missing
// file so a setup error fails the test clearly rather than silently classifying an empty
// Mat.
cv::Mat load_test_data()
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

// Builds the real CNNClassifierCore once for the whole suite (the TensorRT engine build
// is minutes-long). When the model or a usable GPU is missing, core_ stays null and
// skip_reason_ explains why; each test then GTEST_SKIPs. No node is involved -- the core
// is constructed straight from a CNNConfig.
class CnnClassifierCoreClassifyTest : public ::testing::Test
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

    tl::CNNConfig config;
    config.model_path = model_path;
    config.precision = "fp16";
    config.labels = read_label_lines(label_path);
    config.mean = default_mean;
    config.std = default_std;

    // The core ctor builds the TensorRT engine and throws on GPU/TRT failure. Treat that
    // as "environment unavailable" -> skip (not fail), with a reason each test reports.
    try {
      core_ = std::make_unique<tl::CNNClassifierCore>(config);
    } catch (const std::exception & e) {
      skip_reason_ = std::string("CNNClassifierCore environment unavailable: ") + e.what();
      core_.reset();
    }
  }

  static void TearDownTestSuite() { core_.reset(); }

  static inline std::string skip_reason_;
  static inline std::unique_ptr<tl::CNNClassifierCore> core_;
};

// Drives the full classify() path on a real ROI crop -- preprocess -> TRT inference ->
// label decode -> element -- and checks the core's output contract: one signal carrying
// one element with a confidence in (0, 1]. The decoded color/shape is deliberately NOT
// pinned here; that label mapping is covered model-free by the decode_label tests above,
// and asserting a specific class against the live model would make this test brittle to a
// model-parameter update. Only the confidence range is asserted -- softmax normalization
// is the core's own guarantee and is stable across models, while the exact value is not
// reproducible across GPU / TRT versions.
TEST_F(CnnClassifierCoreClassifyTest, ClassifiesNormalCropToWellFormedElement)
{
  if (!core_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange
  const cv::Mat image{load_test_data()};

  // Act
  const auto result = core_->classify({image});

  // Assert
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.signals.signals.size(), 1u);
  ASSERT_EQ(result.signals.signals[0].elements.size(), 1u);
  const auto & element = result.signals.signals[0].elements[0];
  EXPECT_GT(element.confidence, 0.0f);
  EXPECT_LE(element.confidence, 1.0f);
}

// classify() runs the images through the model in batches of its static size and
// scatters one result back per input. Feeding the same crop into both slots pins the
// per-image multiplicity -- two inputs must yield two results -- and, because identical
// inputs must produce identical outputs whatever the model predicts, the two confidences
// must be equal. Both properties hold across model updates, so no specific class is
// pinned. (Swap detection is not the goal here; with identical inputs it is a no-op.)
TEST_F(CnnClassifierCoreClassifyTest, BatchClassificationYieldsResultPerImage)
{
  if (!core_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange -- the same crop in both batch slots.
  const cv::Mat image{load_test_data()};

  // Act
  const auto result = core_->classify({image, image});

  // Assert -- two inputs yield two results, and identical inputs give an identical
  // confidence in each slot.
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.signals.signals.size(), 2u);
  ASSERT_EQ(result.signals.signals[0].elements.size(), 1u);
  ASSERT_EQ(result.signals.signals[1].elements.size(), 1u);
  EXPECT_FLOAT_EQ(
    result.signals.signals[0].elements[0].confidence,
    result.signals.signals[1].elements[0].confidence);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
