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
// Unit tests for the ROS-free ColorClassifierCore.
//
// The core is constructed directly from an HSVConfig and its classify() /
// make_debug_image() are exercised over in-memory cv::Mat inputs -- no rclcpp,
// no node, no parameters. (Earlier revisions drove this through the ROS
// ColorClassifier adapter and a hosted node; the Act now talks to the core
// directly, but the classification assertions are unchanged.)
//
// The adapter-only concerns (the images/signals size guard, logging, debug-image
// publishing) live at other layers -- see test_color_classifier_adapter and
// test_traffic_light_classifier_integration.
//
// Tests follow Arrange-Act-Assert.
//

#include "../src/classifier/color_classifier.hpp"

#include <opencv2/imgproc.hpp>

#include <tier4_perception_msgs/msg/traffic_light_array.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace
{
namespace tl = autoware::traffic_light;

using tier4_perception_msgs::msg::TrafficLightArray;
using tier4_perception_msgs::msg::TrafficLightElement;

// CHARACTERIZATION (remove on promotion to a contract test): these sizes pin the
// CURRENT confidence formula -- pixel_num / (confidence_saturation_side ^ 2) -- and
// the morphology behavior, not a stable contract. A solid in-band image of side N
// yields N*N matching pixels (erode/dilate preserve a solid region), so side <
// confidence_saturation_side keeps confidence in (0, 1) and side >= it clamps to
// exactly 1.0. They must be kept in sync with the 20*20 divisor in
// ColorClassifierCore::classify_element.
// TODO(follow-up): once production exposes the saturation threshold as a named
// constant, derive the sizes from it and drop this manual sync.
constexpr int confidence_saturation_side = 20;
constexpr int unsaturated_side = 16;  // 16*16=256 < 400 -> confidence in (0, 1)
constexpr int saturated_side = 24;    // 24*24=576 >= 400 -> confidence == 1.0
static_assert(
  unsaturated_side < confidence_saturation_side && confidence_saturation_side <= saturated_side,
  "sizes must straddle the confidence saturation threshold");

// Solid image whose every pixel maps to the given OpenCV HSV triple, built in
// HSV then converted to BGR (ColorClassifierCore::filter_hsv does BGR2HSV).
cv::Mat make_solid_hsv_image(int h, int s, int v, int size = unsaturated_side)
{
  cv::Mat hsv(size, size, CV_8UC3, cv::Scalar(h, s, v));
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  return bgr;
}

// In-band solids for each color (green H[50,120] / yellow H[0,50] / red
// H[160,180], all with S,V comfortably inside their bands).
const cv::Mat green_image = make_solid_hsv_image(85, 150, 200);
const cv::Mat amber_image = make_solid_hsv_image(25, 150, 200);
const cv::Mat red_image = make_solid_hsv_image(170, 200, 200);
// CHARACTERIZATION (remove on promotion): in-band green sized off the current
// 20*20 divisor purely to saturate the confidence clamp at 1.0.
const cv::Mat green_image_saturated = make_solid_hsv_image(85, 150, 200, saturated_side);
// Black: V=0 falls below every band's min value -> matches no color.
const cv::Mat black_image = make_solid_hsv_image(0, 0, 0);

// First element of the signal at the given slot, bounds-checked: .at() throws on
// an out-of-range slot or an empty element list, which gtest reports as a test
// failure. A pure accessor (no gtest macros), so failures point at the test body.
const TrafficLightElement & element_at(const TrafficLightArray & signals, std::size_t slot)
{
  return signals.signals.at(slot).elements.at(0);
}

// A default-configured core. The constructor builds the HSV bands from the
// default HSVConfig, so no priming is needed.
class ColorClassifierCoreTest : public ::testing::Test
{
protected:
  tl::ColorClassifierCore core_;
};

// One batched call classifies each in-band solid into its HSV band.
TEST_F(ColorClassifierCoreTest, ClassifiesByHsvBand)
{
  // Arrange
  const std::vector<cv::Mat> images{green_image, amber_image, red_image};

  // Act
  const auto result = core_.classify(images);

  // Assert
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.signals.signals.size(), 3u);
  // Each classified image yields exactly one element; element_at() reads .at(0),
  // so pin the count here rather than discover a regression via a thrown .at().
  for (const auto & signal : result.signals.signals) {
    ASSERT_EQ(signal.elements.size(), 1u);
  }
  EXPECT_EQ(element_at(result.signals, 0).color, TrafficLightElement::GREEN);
  EXPECT_EQ(element_at(result.signals, 1).color, TrafficLightElement::AMBER);
  EXPECT_EQ(element_at(result.signals, 2).color, TrafficLightElement::RED);
}

// An image outside every color band yields UNKNOWN with zero confidence.
TEST_F(ColorClassifierCoreTest, OutOfBandImageIsUnknown)
{
  // Arrange
  const std::vector<cv::Mat> images{black_image};

  // Act
  const auto result = core_.classify(images);

  // Assert
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.signals.signals.size(), 1u);
  EXPECT_EQ(element_at(result.signals, 0).color, TrafficLightElement::UNKNOWN);
  EXPECT_EQ(element_at(result.signals, 0).confidence, 0.0f);
}

// A matched color below the saturation pixel count reports confidence strictly
// inside (0, 1) -- enough to confirm a match scores positive without yet hitting
// the clamp (the clamp itself is covered by SaturatedConfidenceClampsToOne).
TEST_F(ColorClassifierCoreTest, MatchedConfidenceIsPositiveAndBounded)
{
  // Arrange
  const std::vector<cv::Mat> images{green_image};  // unsaturated_side -> below clamp

  // Act
  const auto result = core_.classify(images);

  // Assert
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.signals.signals.size(), 1u);
  EXPECT_GT(element_at(result.signals, 0).confidence, 0.0f);
  EXPECT_LT(element_at(result.signals, 0).confidence, 1.0f);
}

// CHARACTERIZATION (remove on promotion to a contract test): a match with enough
// pixels saturates the confidence clamp at exactly 1.0, exercising the
// std::min(1.0, ...) upper bound. This pins the current pixel-count formula; a
// contract test would instead assert "a strong-enough match reaches 1.0" without
// depending on the 20*20 threshold.
TEST_F(ColorClassifierCoreTest, SaturatedConfidenceClampsToOne)
{
  // Arrange
  const std::vector<cv::Mat> images{green_image_saturated};  // saturated_side -> at/over clamp

  // Act
  const auto result = core_.classify(images);

  // Assert
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.signals.signals.size(), 1u);
  EXPECT_EQ(element_at(result.signals, 0).color, TrafficLightElement::GREEN);
  EXPECT_EQ(element_at(result.signals, 0).confidence, 1.0f);
}

// The thresholds actually drive classification: narrowing the green band to
// exclude the green image's hue turns its result from GREEN into UNKNOWN. Also
// exercises set_config() rebuilding the bands.
TEST_F(ColorClassifierCoreTest, ConfigDrivesClassification)
{
  // Arrange
  tl::HSVConfig config;     // defaults
  config.green_max_h = 60;  // green hue 85 now out of band
  core_.set_config(config);
  const std::vector<cv::Mat> images{green_image};

  // Act
  const auto result = core_.classify(images);

  // Assert
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.signals.signals.size(), 1u);
  EXPECT_EQ(element_at(result.signals, 0).color, TrafficLightElement::UNKNOWN);
}

// An empty batch is a successful no-op.
TEST_F(ColorClassifierCoreTest, EmptyBatchSucceeds)
{
  // Arrange
  const std::vector<cv::Mat> images;

  // Act
  const auto result = core_.classify(images);

  // Assert
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.signals.signals.empty());
}

// CHARACTERIZATION (pins the current debug-mosaic layout, a visualization format
// rather than a stable contract): make_debug_image lays out a 3-column x 4-row
// mosaic of the ROI -- a raw row on top, then the green / yellow / red rows, each
// showing filtered | binarized | denoised -- rendered in color. A non-square ROI
// makes an accidental rows/cols swap detectable.
TEST_F(ColorClassifierCoreTest, DebugImageMosaicGeometry)
{
  // Arrange -- distinct width and height so the 3x vs 4x factors can't be confused.
  const int roi_width = 12;
  const int roi_height = 8;
  cv::Mat roi;
  cv::cvtColor(
    cv::Mat(roi_height, roi_width, CV_8UC3, cv::Scalar(85, 150, 200)), roi, cv::COLOR_HSV2BGR);

  // Act
  const cv::Mat debug_image = core_.make_debug_image(roi);

  // Assert
  EXPECT_EQ(debug_image.cols, roi_width * 3);
  EXPECT_EQ(debug_image.rows, roi_height * 4);
  EXPECT_EQ(debug_image.type(), CV_8UC3);
}

// The mosaic is input-dependent: two same-sized ROIs of different colors produce
// different mosaics (their raw strips and lit color panels both differ). Robust
// against the exact layout -- it only pins that make_debug_image renders the
// input rather than a constant/blank image.
TEST_F(ColorClassifierCoreTest, DebugImageReflectsInput)
{
  // Act
  const cv::Mat green_debug = core_.make_debug_image(green_image);
  const cv::Mat red_debug = core_.make_debug_image(red_image);

  // Assert
  ASSERT_EQ(green_debug.size(), red_debug.size());
  ASSERT_EQ(green_debug.type(), red_debug.type());
  EXPECT_GT(cv::norm(green_debug, red_debug, cv::NORM_INF), 0.0);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
