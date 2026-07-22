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
// Tests for CnnLampRecognizerCore, the Node-free lamp recognition core. Two groups, both
// ROS-free; all Arrange-Act-Assert.
//
//  - Static helpers updateTrafficSignals()/outputDebugImage() need only the message types +
//    OpenCV, so they run (no GPU/model). They pin the non-obvious mapping behavior
//    (empty->placeholder, pedestrian->CIRCLE, confidence zeroing vs keeping, per-lamp order,
//    clear-before-map) and the debug geometry. Plain enum->enum maps (color/arrow/cross) are
//    left unpinned -- a test would only mirror the switch.
//
//  - The ctor builds a TensorRT engine and classify() runs inference, so both need a GPU + the
//    ONNX model (under autoware_data). CnnLampRecognizerCoreClassifyTest self-skips when either
//    is missing, and asserts only model-stable facts (no specific class pinned, since TRT output
//    is not bit-reproducible). Engine built once per suite.
//

#include "../src/classifier/cnn_lamp_recognizer.hpp"

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
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
namespace tl = autoware::traffic_light;

using tier4_perception_msgs::msg::TrafficLight;
using tier4_perception_msgs::msg::TrafficLightElement;

// Confidence for lamps whose value a test does not assert (a valid, non-zero placeholder); tests
// that exercise the confidence path pass an explicit value instead.
constexpr float default_confidence = 1.0f;

// Build one lamp detection. Group A hand-constructs these; updateTrafficSignals maps them
// independently of the model (box stays zero-default, which the mapping ignores).
tl::LampElement make_lamp(
  tl::Color color, tl::Shape shape, tl::ArrowDirection arrow_direction,
  float confidence = default_confidence)
{
  tl::LampElement lamp;
  lamp.color = color;
  lamp.shape = shape;
  lamp.arrow_direction = arrow_direction;
  lamp.confidence = confidence;
  return lamp;
}

// ===================== static helpers (no GPU, no model) =====================

// --- output-list shape (independent of the per-element mapping) ---

// An empty detection list yields a single UNKNOWN/UNKNOWN placeholder with zero confidence,
// so downstream always sees one element per signal.
TEST(CnnLampRecognizerCoreUpdateSignalsTest, EmptyInputYieldsSingleUnknownPlaceholder)
{
  // Arrange
  TrafficLight signal;
  const std::vector<tl::LampElement> no_lamps;

  // Act
  tl::CnnLampRecognizerCore::updateTrafficSignals(no_lamps, signal);

  // Assert
  ASSERT_EQ(signal.elements.size(), 1u);
  EXPECT_EQ(signal.elements[0].color, TrafficLightElement::UNKNOWN);
  EXPECT_EQ(signal.elements[0].shape, TrafficLightElement::UNKNOWN);
  EXPECT_FLOAT_EQ(signal.elements[0].confidence, 0.0f);
}

// updateTrafficSignals emits one output element per input lamp, in input order (it does not
// merge duplicates -- that is classify()'s job). Two distinct lamps must both survive and keep
// their order, so a caller can trust element[i] came from lamp[i].
TEST(CnnLampRecognizerCoreUpdateSignalsTest, MultipleLampsYieldElementPerLampInOrder)
{
  // Arrange -- two lamps that map to distinct shapes so an order swap is visible. Confidence is
  // not asserted here, so both use the default.
  TrafficLight signal;
  const std::vector<tl::LampElement> lamps{
    make_lamp(tl::Color::GREEN, tl::Shape::CIRCLE, tl::ArrowDirection::UNKNOWN),
    make_lamp(tl::Color::RED, tl::Shape::CROSS, tl::ArrowDirection::UNKNOWN)};

  // Act
  tl::CnnLampRecognizerCore::updateTrafficSignals(lamps, signal);

  // Assert
  ASSERT_EQ(signal.elements.size(), 2u);
  EXPECT_EQ(signal.elements[0].color, TrafficLightElement::GREEN);
  EXPECT_EQ(signal.elements[0].shape, TrafficLightElement::CIRCLE);
  EXPECT_EQ(signal.elements[1].color, TrafficLightElement::RED);
  EXPECT_EQ(signal.elements[1].shape, TrafficLightElement::CROSS);
}

// updateTrafficSignals clears the signal's existing elements before mapping, so repeated calls
// replace rather than accumulate. A stale element left in place would be a downstream bug.
TEST(CnnLampRecognizerCoreUpdateSignalsTest, ClearsPreexistingElements)
{
  // Arrange -- a signal already carrying a stale element from a previous frame.
  TrafficLight signal;
  TrafficLightElement stale;
  stale.color = TrafficLightElement::AMBER;
  stale.shape = TrafficLightElement::CIRCLE;
  signal.elements.push_back(stale);
  const std::vector<tl::LampElement> lamps{
    make_lamp(tl::Color::GREEN, tl::Shape::CIRCLE, tl::ArrowDirection::UNKNOWN)};

  // Act
  tl::CnnLampRecognizerCore::updateTrafficSignals(lamps, signal);

  // Assert -- only the freshly mapped element remains; the stale AMBER one is gone.
  ASSERT_EQ(signal.elements.size(), 1u);
  EXPECT_EQ(signal.elements[0].color, TrafficLightElement::GREEN);
}

// --- per-element mapping (pedestrian forcing, then the confidence guard) ---

// A pedestrian signal forces CIRCLE regardless of the lamp's own shape, while the color and
// confidence are preserved (contrast UnknownArrowDirectionZeroesConfidence).
TEST(CnnLampRecognizerCoreUpdateSignalsTest, PedestrianTypeForcesCircle)
{
  // Arrange -- a pedestrian signal whose lamp claims an ARROW shape.
  TrafficLight signal;
  signal.traffic_light_type = TrafficLight::PEDESTRIAN_TRAFFIC_LIGHT;
  const std::vector<tl::LampElement> lamps{
    make_lamp(tl::Color::GREEN, tl::Shape::ARROW, tl::ArrowDirection::UP_ARROW, 0.8f)};

  // Act
  tl::CnnLampRecognizerCore::updateTrafficSignals(lamps, signal);

  // Assert
  ASSERT_EQ(signal.elements.size(), 1u);
  EXPECT_EQ(signal.elements[0].shape, TrafficLightElement::CIRCLE);
  EXPECT_EQ(signal.elements[0].color, TrafficLightElement::GREEN);
  EXPECT_FLOAT_EQ(signal.elements[0].confidence, 0.8f);
}

// A PED-shaped lamp maps to CIRCLE even on a car signal (the other half of the pedestrian OR).
TEST(CnnLampRecognizerCoreUpdateSignalsTest, PedShapeForcesCircle)
{
  // Arrange
  TrafficLight signal;
  signal.traffic_light_type = TrafficLight::CAR_TRAFFIC_LIGHT;
  const std::vector<tl::LampElement> lamps{
    make_lamp(tl::Color::RED, tl::Shape::PED, tl::ArrowDirection::UNKNOWN)};

  // Act
  tl::CnnLampRecognizerCore::updateTrafficSignals(lamps, signal);

  // Assert
  ASSERT_EQ(signal.elements.size(), 1u);
  EXPECT_EQ(signal.elements[0].shape, TrafficLightElement::CIRCLE);
  EXPECT_EQ(signal.elements[0].color, TrafficLightElement::RED);
}

// An unknown color maps to UNKNOWN but, unlike an unknown shape/arrow, does NOT zero the
// confidence. Only the shape switch guards confidence, so a well-shaped lamp of unknown color
// stays trusted (contrast the two ...ZeroesConfidence tests below).
TEST(CnnLampRecognizerCoreUpdateSignalsTest, UnknownColorPreservesConfidence)
{
  // Arrange
  TrafficLight signal;
  const std::vector<tl::LampElement> lamps{
    make_lamp(tl::Color::UNKNOWN, tl::Shape::CIRCLE, tl::ArrowDirection::UNKNOWN, 0.7f)};

  // Act
  tl::CnnLampRecognizerCore::updateTrafficSignals(lamps, signal);

  // Assert
  ASSERT_EQ(signal.elements.size(), 1u);
  EXPECT_EQ(signal.elements[0].color, TrafficLightElement::UNKNOWN);
  EXPECT_EQ(signal.elements[0].shape, TrafficLightElement::CIRCLE);
  EXPECT_FLOAT_EQ(signal.elements[0].confidence, 0.7f);
}

// An ARROW lamp whose direction is uncertain falls through to UNKNOWN, and the confidence is
// zeroed even though the detection carried a positive one.
TEST(CnnLampRecognizerCoreUpdateSignalsTest, UnknownArrowDirectionZeroesConfidence)
{
  // Arrange
  TrafficLight signal;
  const std::vector<tl::LampElement> lamps{
    make_lamp(tl::Color::GREEN, tl::Shape::ARROW, tl::ArrowDirection::UNKNOWN, 0.9f)};

  // Act
  tl::CnnLampRecognizerCore::updateTrafficSignals(lamps, signal);

  // Assert
  ASSERT_EQ(signal.elements.size(), 1u);
  EXPECT_EQ(signal.elements[0].shape, TrafficLightElement::UNKNOWN);
  EXPECT_FLOAT_EQ(signal.elements[0].confidence, 0.0f);
}

// A shape with no message mapping yet (e.g. U_TURN, see the TODO in updateTrafficSignals) hits
// the outer default: UNKNOWN with the confidence zeroed, even though the detection carried a
// positive one.
TEST(CnnLampRecognizerCoreUpdateSignalsTest, UnsupportedShapeZeroesConfidence)
{
  // Arrange
  TrafficLight signal;
  const std::vector<tl::LampElement> lamps{
    make_lamp(tl::Color::GREEN, tl::Shape::U_TURN, tl::ArrowDirection::UNKNOWN, 0.9f)};

  // Act
  tl::CnnLampRecognizerCore::updateTrafficSignals(lamps, signal);

  // Assert
  ASSERT_EQ(signal.elements.size(), 1u);
  EXPECT_EQ(signal.elements[0].shape, TrafficLightElement::UNKNOWN);
  EXPECT_FLOAT_EQ(signal.elements[0].confidence, 0.0f);
}

// Fill color is immaterial -- the debug tests assert geometry only.
const cv::Scalar roi_fill_color{0, 255, 0};

// outputDebugImage rescales the ROI to a fixed width and stacks a fixed-height label strip below
// it (kDebugImageWidth / kDebugTextHeight in the production code).
constexpr int debug_image_width = 200;
constexpr int debug_text_height = 50;

// A non-square ROI makes an accidental rows/cols swap in the rescale detectable.
TEST(CnnLampRecognizerCoreDebugImageTest, MosaicGeometry)
{
  // Arrange -- distinct width and height; one detection to draw and one signal element to
  // drive the label strip.
  const int roi_width = 100;
  const int roi_height = 50;
  cv::Mat debug_image(roi_height, roi_width, CV_8UC3, roi_fill_color);

  tl::LampElement lamp =
    make_lamp(tl::Color::GREEN, tl::Shape::CIRCLE, tl::ArrowDirection::UNKNOWN);
  lamp.box = tl::BBox{0.2f, 0.2f, 0.8f, 0.8f};
  const std::vector<tl::LampElement> lamps{lamp};

  TrafficLight signal;
  TrafficLightElement element;
  element.color = TrafficLightElement::GREEN;
  element.shape = TrafficLightElement::CIRCLE;
  element.confidence = 0.9f;
  signal.elements.push_back(element);

  // Act
  tl::CnnLampRecognizerCore::outputDebugImage(debug_image, signal, &lamps);

  // Assert -- fixed width; height = aspect-preserving scaled ROI + the label strip.
  const int scaled_roi_height = debug_image_width * roi_height / roi_width;  // 200*50/100 = 100
  EXPECT_EQ(debug_image.cols, debug_image_width);
  EXPECT_EQ(debug_image.rows, scaled_roi_height + debug_text_height);
  EXPECT_EQ(debug_image.type(), CV_8UC3);
}

// outputDebugImage still produces the mosaic when no per-lamp detections are supplied
// (elements == nullptr) -- the box-drawing branch is guarded and must not dereference it.
TEST(CnnLampRecognizerCoreDebugImageTest, HandlesNullElements)
{
  // Arrange
  const int roi_width = 100;
  const int roi_height = 50;
  cv::Mat debug_image(roi_height, roi_width, CV_8UC3, roi_fill_color);
  TrafficLight signal;  // no elements -> empty label

  // Act
  tl::CnnLampRecognizerCore::outputDebugImage(debug_image, signal, nullptr);

  // Assert
  const int scaled_roi_height = debug_image_width * roi_height / roi_width;
  EXPECT_EQ(debug_image.cols, debug_image_width);
  EXPECT_EQ(debug_image.rows, scaled_roi_height + debug_text_height);
  EXPECT_EQ(debug_image.type(), CV_8UC3);
}

// ================== GPU-gated tests of the TensorRT classify() path ==================

// Lamp recognizer model, downloaded under autoware_data by the ansible artifacts role:
// https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v4/traffic_light_lamp_recognizer_comlops.onnx
constexpr char model_filename[] = "traffic_light_lamp_recognizer_comlops.onnx";

// Resolve a file under autoware_data, trying the canonical ml_models/ layout and the legacy
// flat one; $TLC_TEST_DATA_DIR overrides both. Returns "" when not found.
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

// Load the normal-traffic-light test crop as RGB (imread yields BGR; the classifier is fed RGB).
// It is the only crop these tests load. Throws on a missing file so a setup error fails loudly
// instead of classifying an empty Mat.
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

// True when a normalized box coordinate lies in the unit interval [0, 1] -- the range decode
// clamps every coordinate to.
bool in_unit_range(float value)
{
  return 0.0f <= value && value <= 1.0f;
}

// Builds the real core once per suite (the TensorRT engine build is minutes-long), straight
// from a CnnLampRecognizerConfig -- no node. When the model or a usable GPU is missing, core_
// stays null with a skip_reason_ and each test GTEST_SKIPs.
class CnnLampRecognizerCoreClassifyTest : public ::testing::Test
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
    // is_cuda_runtime_available() only counts devices; probe context creation too, since a broken
    // GPU (OOM/driver) makes TensorRT abort the whole process during the engine build with an
    // error that cannot be caught.
    if (const cudaError_t cuda_status = cudaFree(0); cuda_status != cudaSuccess) {
      skip_reason_ = std::string("CUDA device unusable: ") + cudaGetErrorString(cuda_status);
      return;
    }

    tl::CnnLampRecognizerConfig config;
    config.model_path = model_path;
    config.precision = "fp16";
    config.score_threshold = 0.2f;
    config.nms_threshold = 0.2f;
    config.max_batch_size = 64;
    // The YOLO-head layout matches the LampRegressionArchitecture defaults
    // (lamp_recognizer_ml.param.yaml); only the anchors are model-specific with no default.
    config.model_params.scale_x_y = 2.0f;
    config.model_params.anchors = {7.f, 7.f, 14.f, 14.f, 42.f, 42.f};

    // Device is usable (checked above), so a ctor throw here is a config/engine mismatch (e.g.
    // output channels vs model_params); treat as unavailable -> skip, with a reason per test.
    try {
      core_ = std::make_unique<tl::CnnLampRecognizerCore>(config);
    } catch (const std::exception & e) {
      skip_reason_ = std::string("CnnLampRecognizerCore environment unavailable: ") + e.what();
      core_.reset();
    }
  }

  static void TearDownTestSuite() { core_.reset(); }

  static inline std::string skip_reason_;
  static inline std::unique_ptr<tl::CnnLampRecognizerCore> core_;
};

// Count/color/shape are NOT pinned (brittle across model updates; the mapping is covered
// model-free by the updateTrafficSignals tests). Require >=1 detection so the per-lamp checks
// can't vacuously pass on an empty result.
TEST_F(CnnLampRecognizerCoreClassifyTest, RealCropYieldsWellFormedDetections)
{
  if (!core_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange
  const cv::Mat image{load_rgb_crop()};

  // Act
  const auto result = core_->classify({image});

  // Assert
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.lamps_per_image.size(), 1u);
  ASSERT_FALSE(result.lamps_per_image[0].empty());
  for (const auto & lamp : result.lamps_per_image[0]) {
    EXPECT_TRUE(in_unit_range(lamp.box.x1));
    EXPECT_TRUE(in_unit_range(lamp.box.y1));
    EXPECT_TRUE(in_unit_range(lamp.box.x2));
    EXPECT_TRUE(in_unit_range(lamp.box.y2));
    EXPECT_GT(lamp.confidence, 0.0f);
    EXPECT_LE(lamp.confidence, 1.0f);
  }
}

// classify() returns one result vector per input image. A detected crop and a black image
// (which yields nothing) make the slots asymmetric, so a scatter swap would empty slot 0 and
// fill slot 1 -- identical crops could not catch that. Black keeps the empty slot model-stable:
// no model detects a lamp in pure black. Contents are left to the updateTrafficSignals tests.
TEST_F(CnnLampRecognizerCoreClassifyTest, BatchClassificationScattersPerImageResults)
{
  if (!core_) {
    GTEST_SKIP() << skip_reason_;
  }

  // Arrange -- detected crop in slot 0, a black (featureless) image in slot 1.
  const cv::Mat detected{load_rgb_crop()};
  const cv::Mat black{cv::Mat::zeros(detected.size(), CV_8UC3)};

  // Act
  const auto result = core_->classify({detected, black});

  // Assert -- one result vector per input, scattered to the correct slot.
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.lamps_per_image.size(), 2u);
  EXPECT_FALSE(result.lamps_per_image[0].empty());
  EXPECT_TRUE(result.lamps_per_image[1].empty());
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
