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
// Unit tests for ColorClassifier.
//
// ColorClassifier currently takes an rclcpp::Node * and reads its HSV thresholds
// from parameters, so these tests host a node and prime the thresholds via
// set_parameter() (which synchronously fires the on-set-parameter callback that
// populates the thresholds). A later refactor removes that ROS coupling; only
// the Arrange (construction/priming) changes then, the assertions stay the same.
//
// Tests follow Arrange-Act-Assert.
//

#include "../src/classifier/color_classifier.hpp"

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_perception_msgs/msg/traffic_light_array.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
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
// ColorClassifier::getTrafficSignals.
// TODO(follow-up): once production exposes the saturation threshold as a named
// constant, derive the sizes from it and drop this manual sync.
constexpr int confidence_saturation_side = 20;
constexpr int unsaturated_side = 16;  // 16*16=256 < 400 -> confidence in (0, 1)
constexpr int saturated_side = 24;    // 24*24=576 >= 400 -> confidence == 1.0
static_assert(
  unsaturated_side < confidence_saturation_side && confidence_saturation_side <= saturated_side,
  "sizes must straddle the confidence saturation threshold");

// Solid image whose every pixel maps to the given OpenCV HSV triple, built in
// HSV then converted to BGR (ColorClassifier::filterHSV does BGR2HSV).
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

// Hosts a node and a ColorClassifier with its HSV thresholds primed to the
// declared defaults. Setting any HSV parameter fires the on-set-parameter
// callback that populates all thresholds.
class ColorClassifierTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("color_classifier_test");
    classifier_ = std::make_unique<tl::ColorClassifier>(node_.get());
    // Prime all HSV thresholds: the ctor leaves min_/max_hsv_* unset, and only
    // the on-set-parameter callback builds them (rebuilding ALL bands from
    // hsv_config_ on any change), so setting one param to its default suffices.
    // TODO(follow-up): drop this once the ctor initializes the thresholds.
    node_->set_parameter(rclcpp::Parameter("green_min_h", 50));
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<tl::ColorClassifier> classifier_;
};

// One batched call classifies each in-band solid into its HSV band.
TEST_F(ColorClassifierTest, ClassifiesByHsvBand)
{
  // Arrange
  const std::vector<cv::Mat> images{green_image, amber_image, red_image};
  TrafficLightArray signals;
  signals.signals.resize(images.size());

  // Act
  const bool ok = classifier_->getTrafficSignals(images, signals);

  // Assert
  ASSERT_TRUE(ok);
  ASSERT_EQ(signals.signals.size(), 3u);
  // Each classified image yields exactly one element; element_at() reads .at(0),
  // so pin the count here rather than discover a regression via a thrown .at().
  for (const auto & signal : signals.signals) {
    ASSERT_EQ(signal.elements.size(), 1u);
  }
  EXPECT_EQ(element_at(signals, 0).color, TrafficLightElement::GREEN);
  EXPECT_EQ(element_at(signals, 1).color, TrafficLightElement::AMBER);
  EXPECT_EQ(element_at(signals, 2).color, TrafficLightElement::RED);
}

// An image outside every color band yields UNKNOWN with zero confidence.
TEST_F(ColorClassifierTest, OutOfBandImageIsUnknown)
{
  // Arrange
  const std::vector<cv::Mat> images{black_image};
  TrafficLightArray signals;
  signals.signals.resize(1);

  // Act
  const bool ok = classifier_->getTrafficSignals(images, signals);

  // Assert
  ASSERT_TRUE(ok);
  ASSERT_EQ(signals.signals.size(), 1u);
  EXPECT_EQ(element_at(signals, 0).color, TrafficLightElement::UNKNOWN);
  EXPECT_EQ(element_at(signals, 0).confidence, 0.0f);
}

// A matched color below the saturation pixel count reports confidence strictly
// inside (0, 1) -- enough to confirm a match scores positive without yet hitting
// the clamp (the clamp itself is covered by SaturatedConfidenceClampsToOne).
TEST_F(ColorClassifierTest, MatchedConfidenceIsPositiveAndBounded)
{
  // Arrange
  const std::vector<cv::Mat> images{green_image};  // unsaturated_side -> below clamp
  TrafficLightArray signals;
  signals.signals.resize(1);

  // Act
  const bool ok = classifier_->getTrafficSignals(images, signals);

  // Assert
  ASSERT_TRUE(ok);
  ASSERT_EQ(signals.signals.size(), 1u);
  EXPECT_GT(element_at(signals, 0).confidence, 0.0f);
  EXPECT_LT(element_at(signals, 0).confidence, 1.0f);
}

// CHARACTERIZATION (remove on promotion to a contract test): a match with enough
// pixels saturates the confidence clamp at exactly 1.0, exercising the
// std::min(1.0, ...) upper bound. This pins the current pixel-count formula; a
// contract test would instead assert "a strong-enough match reaches 1.0" without
// depending on the 20*20 threshold.
TEST_F(ColorClassifierTest, SaturatedConfidenceClampsToOne)
{
  // Arrange
  const std::vector<cv::Mat> images{green_image_saturated};  // saturated_side -> at/over clamp
  TrafficLightArray signals;
  signals.signals.resize(1);

  // Act
  const bool ok = classifier_->getTrafficSignals(images, signals);

  // Assert
  ASSERT_TRUE(ok);
  ASSERT_EQ(signals.signals.size(), 1u);
  EXPECT_EQ(element_at(signals, 0).color, TrafficLightElement::GREEN);
  EXPECT_EQ(element_at(signals, 0).confidence, 1.0f);
}

// The thresholds actually drive classification: narrowing the green band to
// exclude the green image's hue turns its result from GREEN into UNKNOWN.
TEST_F(ColorClassifierTest, ConfigDrivesClassification)
{
  // Arrange
  node_->set_parameter(rclcpp::Parameter("green_max_h", 60));  // green hue 85 now out of band
  const std::vector<cv::Mat> images{green_image};
  TrafficLightArray signals;
  signals.signals.resize(1);

  // Act
  const bool ok = classifier_->getTrafficSignals(images, signals);

  // Assert
  ASSERT_TRUE(ok);
  ASSERT_EQ(signals.signals.size(), 1u);
  EXPECT_EQ(element_at(signals, 0).color, TrafficLightElement::UNKNOWN);
}

// A mismatched images/signals count is rejected.
TEST_F(ColorClassifierTest, MismatchedSizesReturnFalse)
{
  // Arrange
  const std::vector<cv::Mat> images{green_image};
  TrafficLightArray signals;
  signals.signals.resize(2);

  // Act
  const bool ok = classifier_->getTrafficSignals(images, signals);

  // Assert
  EXPECT_FALSE(ok);
}

// An empty batch is a successful no-op.
TEST_F(ColorClassifierTest, EmptyBatchReturnsTrue)
{
  // Arrange
  const std::vector<cv::Mat> images;
  TrafficLightArray signals;

  // Act
  const bool ok = classifier_->getTrafficSignals(images, signals);

  // Assert
  EXPECT_TRUE(ok);
  EXPECT_TRUE(signals.signals.empty());
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
