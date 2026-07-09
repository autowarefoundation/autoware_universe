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
// ROS-free unit tests for CNNClassifierCore's pure, GPU-free helpers.
//
// CNNClassifierCore's constructor builds a TensorRT engine (needs a GPU + model) and
// classify() runs inference, so neither can run in a hardware-free test -- that path is
// covered by the GPU-gated characterization test test_cnn_classifier.cpp. The two pure
// helpers, decode_label() and make_debug_image(), are static and depend only on the
// perception message types + OpenCV, so they are exercised here without a node, a GPU,
// or a model.
//
// Tests follow Arrange-Act-Assert.
//

#include "../src/classifier/cnn_classifier.hpp"

#include <opencv2/core.hpp>

#include <tier4_perception_msgs/msg/traffic_light.hpp>
#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

#include <gtest/gtest.h>

namespace
{
namespace tl = autoware::traffic_light;

using tier4_perception_msgs::msg::TrafficLight;
using tier4_perception_msgs::msg::TrafficLightElement;

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

// make_debug_image returns a fresh Mat and does not mutate the input ROI (the old
// in-place outputDebugImage resized its argument).
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

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
