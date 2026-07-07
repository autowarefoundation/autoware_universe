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
// Characterization test for CNNClassifier.
//
// CNNClassifier runs a TensorRT MobileNet-v2 engine, so -- unlike the pure-OpenCV
// ColorClassifierCore -- it cannot be exercised without a GPU and the ONNX model
// (downloaded under autoware_data). Following the approach of the
// autoware_tensorrt_yolox node test, this pins the CURRENT end-to-end behavior of
// getTrafficSignals against the real model, as a safety net before the planned
// core/adapter split of cnn_classifier.{hpp,cpp}. It loads the real model,
// self-skips (GTEST_SKIP) when the GPU or model is unavailable, and asserts
// coarsely because TensorRT output is not bit-reproducible across GPU / TRT
// versions.
//
// The engine is built once per suite (SetUpTestSuite) since that build takes
// minutes. Inputs are the green traffic-light ROI crops shipped in
// test/test_data, converted BGR->RGB to match the node's RGB classifier input.
//
// Tests follow Arrange-Act-Assert.
//

#include "../src/classifier/cnn_classifier.hpp"

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

// CAR CNN classifier artifacts, downloaded under autoware_data (see the ansible
// artifacts role). The model's static batch size is 6.
constexpr char model_filename[] = "traffic_light_classifier_mobilenetv2_batch_6.onnx";
constexpr char label_filename[] = "lamp_labels.txt";

// Normalization from config/car_traffic_light_classifier.param.yaml.
const std::vector<double> default_mean{123.675, 116.28, 103.53};
const std::vector<double> default_std{58.395, 57.12, 57.375};

// Resolve a file shipped under autoware_data. The on-disk layout differs between
// setups (canonical autoware_data/ml_models/traffic_light_classifier vs. the
// legacy flat autoware_data/traffic_light_classifier); an explicit
// TLC_TEST_DATA_DIR override takes precedence. Returns "" when not found.
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
// classifier RGB). Throws on a missing file so a setup error fails the test
// clearly rather than silently classifying an empty Mat.
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

// Builds the real CNNClassifier once for the whole suite (the TensorRT engine
// build is minutes-long). When the model or a usable GPU is missing, classifier_
// stays null and skip_reason_ explains why; each test then GTEST_SKIPs.
class CnnClassifierCharacterizationTest : public ::testing::Test
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

    // Node creation can fail on a misconfigured RMW/DDS environment, and the
    // CNNClassifier ctor builds the TensorRT engine and throws on GPU/TRT failure.
    // Treat all of these as "environment unavailable" -> skip (not fail), with a
    // reason each test reports via GTEST_SKIP.
    try {
      node_ = std::make_shared<rclcpp::Node>("cnn_classifier_characterization_test", options);
      classifier_ = std::make_shared<tl::CNNClassifier>(node_.get());
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

// A bright green ROI decodes to a single GREEN CIRCLE element (label class 0,
// "green"), exercising the full path: preprocess -> TRT inference -> postProcess
// label decode -> element. GREEN/CIRCLE is a stable characterization (the argmax
// class is robust to score jitter); the exact confidence is not pinned. The dimmed
// variants below repeat this per exposure level (one test each so a regression
// names the offending crop); confidence varies non-monotonically with dimming
// (~1.0 / 0.55 / 0.81 / 0.94 for normal / weak / medium / strong here).
TEST_F(CnnClassifierCharacterizationTest, ClassifiesNormalGreenCrop)
{
  if (!classifier_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange
  const std::vector<cv::Mat> images{load_rgb_crop("traffic_light_normal.png")};
  TrafficLightArray signals;
  signals.signals.resize(images.size());

  // Act
  const bool ok = classifier_->getTrafficSignals(images, signals);

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

// A weakly-dimmed green ROI still decodes to a single GREEN CIRCLE element.
TEST_F(CnnClassifierCharacterizationTest, ClassifiesWeaklyDimmedGreenCrop)
{
  if (!classifier_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange
  const std::vector<cv::Mat> images{load_rgb_crop("traffic_light_dimmed_weak.png")};
  TrafficLightArray signals;
  signals.signals.resize(images.size());

  // Act
  const bool ok = classifier_->getTrafficSignals(images, signals);

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

// A medium-dimmed green ROI still decodes to a single GREEN CIRCLE element.
TEST_F(CnnClassifierCharacterizationTest, ClassifiesMediumDimmedGreenCrop)
{
  if (!classifier_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange
  const std::vector<cv::Mat> images{load_rgb_crop("traffic_light_dimmed_medium.png")};
  TrafficLightArray signals;
  signals.signals.resize(images.size());

  // Act
  const bool ok = classifier_->getTrafficSignals(images, signals);

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

// A strongly-dimmed green ROI still decodes to a single GREEN CIRCLE element.
TEST_F(CnnClassifierCharacterizationTest, ClassifiesStronglyDimmedGreenCrop)
{
  if (!classifier_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange
  const std::vector<cv::Mat> images{load_rgb_crop("traffic_light_dimmed_strong.png")};
  TrafficLightArray signals;
  signals.signals.resize(images.size());

  // Act
  const bool ok = classifier_->getTrafficSignals(images, signals);

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

// The batch is padded to the model's static size with copies of the front image, so
// this pins that each ROI is scattered to its own slot: the bright crop (slot 0) must
// be strictly more confident than the dimmed one (slot 1) -- a swap or a collapse to
// one image both break the strict inequality. Pairs the brightest with the weakest
// crop (~1.0 vs ~0.55 here) for the widest confidence margin; revisit if a future
// model equalizes them.
TEST_F(CnnClassifierCharacterizationTest, BatchClassificationScattersPerImageResults)
{
  if (!classifier_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange -- bright in slot 0, weakly-dimmed in slot 1.
  const cv::Mat bright = load_rgb_crop("traffic_light_normal.png");
  const cv::Mat dim = load_rgb_crop("traffic_light_dimmed_weak.png");
  TrafficLightArray batched;
  batched.signals.resize(2);

  // Act
  const bool ok = classifier_->getTrafficSignals({bright, dim}, batched);

  // Assert -- one GREEN element per slot (both crops are green lights), and the
  // bright slot is strictly more confident than the dimmed slot (fails on a swapped
  // mapping or a collapse to one image).
  ASSERT_TRUE(ok);
  ASSERT_EQ(batched.signals.size(), 2u);
  ASSERT_EQ(batched.signals[0].elements.size(), 1u);
  ASSERT_EQ(batched.signals[1].elements.size(), 1u);
  EXPECT_EQ(batched.signals[0].elements[0].color, TrafficLightElement::GREEN);
  EXPECT_EQ(batched.signals[1].elements[0].color, TrafficLightElement::GREEN);
  EXPECT_GT(batched.signals[0].elements[0].confidence, batched.signals[1].elements[0].confidence);
}

// A mismatched images/signals count is rejected before any inference (the guard
// stays in the ROS adapter after the core/adapter split). Mirrors the equivalent
// guard test in test_color_classifier_adapter.cpp.
TEST_F(CnnClassifierCharacterizationTest, MismatchedImageSignalCountReturnsFalse)
{
  if (!classifier_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange -- one image but two signal slots.
  const std::vector<cv::Mat> images{load_rgb_crop("traffic_light_normal.png")};
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
