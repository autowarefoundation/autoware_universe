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

#include "autoware_crosswalk_traffic_light_estimator/flashing_detection.hpp"

#include <gtest/gtest.h>

#include <utility>

using autoware::crosswalk_traffic_light_estimator::FlashingDetectionConfig;
using autoware::crosswalk_traffic_light_estimator::FlashingDetector;
using autoware::crosswalk_traffic_light_estimator::TrafficSignal;
using autoware::crosswalk_traffic_light_estimator::TrafficSignalElement;

namespace
{

const int DEFAULT_SIGNAL_ID = 100;
const rclcpp::Time DEFAULT_TIME(1000, 0, RCL_ROS_TIME);

TrafficSignal make_signal(uint8_t color, float confidence = 1.0)
{
  TrafficSignal signal;
  signal.traffic_light_group_id = DEFAULT_SIGNAL_ID;  // Default ID for testing
  TrafficSignalElement element;
  element.color = color;
  element.shape = TrafficSignalElement::CIRCLE;
  element.status = TrafficSignalElement::SOLID_ON;
  element.confidence = confidence;
  signal.elements.push_back(element);
  return signal;
}

}  // namespace

// --- estimate_stable_color tests ---

TEST(FlashingDetectorTest, FirstCall_ReturnsDetectedColor)
{
  // Arrange
  auto detector = FlashingDetector(FlashingDetectionConfig{1.0});
  auto signal = make_signal(TrafficSignalElement::GREEN);

  // Act
  const uint8_t color = detector.estimate_stable_color(signal, DEFAULT_TIME);

  // Assert
  EXPECT_EQ(color, TrafficSignalElement::GREEN);
}

TEST(FlashingDetectorTest, FlashingDetected_UnknownAfterGreen)
{
  // Arrange
  auto detector = FlashingDetector(FlashingDetectionConfig{1.0});

  // Act
  detector.estimate_stable_color(make_signal(TrafficSignalElement::GREEN), DEFAULT_TIME);
  const uint8_t color = detector.estimate_stable_color(
    make_signal(TrafficSignalElement::UNKNOWN, 0.5),
    DEFAULT_TIME + rclcpp::Duration::from_seconds(0.1));

  // Assert: maintains GREEN during flashing (UNKNOWN input doesn't change state to UNKNOWN)
  EXPECT_EQ(color, TrafficSignalElement::GREEN);
}

TEST(FlashingDetectorTest, FlashingTransition_GreenToRed)
{
  // Arrange
  auto detector = FlashingDetector(FlashingDetectionConfig{1.0});

  // Act: during flashing, RED input transitions from GREEN to RED
  detector.estimate_stable_color(make_signal(TrafficSignalElement::GREEN), DEFAULT_TIME);
  detector.estimate_stable_color(
    make_signal(TrafficSignalElement::UNKNOWN, 0.5),
    DEFAULT_TIME + rclcpp::Duration::from_seconds(0.1));
  const uint8_t color = detector.estimate_stable_color(
    make_signal(TrafficSignalElement::RED), DEFAULT_TIME + rclcpp::Duration::from_seconds(0.2));

  // Assert
  EXPECT_EQ(color, TrafficSignalElement::RED);
}

TEST(FlashingDetectorTest, FlashingTransition_RedToGreen)
{
  // Arrange
  auto detector = FlashingDetector(FlashingDetectionConfig{1.0});

  // Act: during flashing, GREEN input transitions from RED to GREEN
  detector.estimate_stable_color(make_signal(TrafficSignalElement::GREEN), DEFAULT_TIME);
  detector.estimate_stable_color(
    make_signal(TrafficSignalElement::UNKNOWN, 0.5),
    DEFAULT_TIME + rclcpp::Duration::from_seconds(0.1));
  detector.estimate_stable_color(
    make_signal(TrafficSignalElement::RED), DEFAULT_TIME + rclcpp::Duration::from_seconds(0.2));
  const uint8_t color = detector.estimate_stable_color(
    make_signal(TrafficSignalElement::GREEN), DEFAULT_TIME + rclcpp::Duration::from_seconds(0.3));

  // Assert
  EXPECT_EQ(color, TrafficSignalElement::GREEN);
}

TEST(FlashingDetectorTest, EstimateStableColor_PrunesOldEntries)
{
  // Arrange
  auto detector = FlashingDetector(FlashingDetectionConfig{1.0});
  const rclcpp::Time base_time(1000, 0, RCL_ROS_TIME);

  // Build state and trigger flashing
  detector.estimate_stable_color(make_signal(TrafficSignalElement::GREEN), base_time);
  detector.estimate_stable_color(
    make_signal(TrafficSignalElement::UNKNOWN, 0.5),
    base_time + rclcpp::Duration::from_seconds(0.1));

  // Act: feed RED at time > hold_time, old GREEN/UNKNOWN entries are pruned
  const uint8_t color = detector.estimate_stable_color(
    make_signal(TrafficSignalElement::RED), base_time + rclcpp::Duration::from_seconds(2.0));

  // Assert: flashing was reset because old entries were pruned, only RED remains
  EXPECT_EQ(color, TrafficSignalElement::RED);
}

// --- clear_state tests ---

TEST(FlashingDetectorTest, ClearState_RemovesTracking)
{
  // Arrange
  auto detector = FlashingDetector(FlashingDetectionConfig{1.0});

  // Act
  // Trigger flashing
  detector.estimate_stable_color(make_signal(TrafficSignalElement::GREEN), DEFAULT_TIME);
  detector.estimate_stable_color(
    make_signal(TrafficSignalElement::UNKNOWN, 0.5),
    DEFAULT_TIME + rclcpp::Duration::from_seconds(0.1));
  // After clearing, estimate_stable_color should behave as if id was never seen
  detector.clear_state(DEFAULT_SIGNAL_ID);
  const uint8_t color = detector.estimate_stable_color(
    make_signal(TrafficSignalElement::RED), DEFAULT_TIME + rclcpp::Duration::from_seconds(0.2));

  // Assert: returns RED as a first call (no flashing state)
  EXPECT_EQ(color, TrafficSignalElement::RED);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
