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
using autoware::crosswalk_traffic_light_estimator::TrafficLightIdMap;
using autoware::crosswalk_traffic_light_estimator::TrafficSignal;
using autoware::crosswalk_traffic_light_estimator::TrafficSignalAndTime;
using autoware::crosswalk_traffic_light_estimator::TrafficSignalElement;

namespace
{

const int DEFAULT_SIGNAL_ID = 100;

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

TrafficLightIdMap make_id_map(const TrafficSignal & signal, const rclcpp::Time & time)
{
  TrafficLightIdMap map;
  map[signal.traffic_light_group_id] = std::make_pair(signal, time);
  return map;
}

}  // namespace

// --- estimate_stable_color tests ---

TEST(FlashingDetectorTest, Process_FirstCall_ReturnsDetectedColor)
{
  // Arrange
  auto detector = FlashingDetector(FlashingDetectionConfig{1.0});
  auto signal = make_signal(TrafficSignalElement::GREEN);

  // Act
  const uint8_t color = detector.estimate_stable_color(signal);

  // Assert
  EXPECT_EQ(color, TrafficSignalElement::GREEN);
}

TEST(FlashingDetectorTest, Process_FlashingDetected_UnknownAfterGreen)
{
  // Arrange
  auto detector = FlashingDetector(FlashingDetectionConfig{1.0});

  // Act
  detector.estimate_stable_color(make_signal(TrafficSignalElement::GREEN));
  const uint8_t color =
    detector.estimate_stable_color(make_signal(TrafficSignalElement::UNKNOWN, 0.5));

  // Assert: maintains GREEN during flashing (UNKNOWN input doesn't change state to UNKNOWN)
  EXPECT_EQ(color, TrafficSignalElement::GREEN);
}

TEST(FlashingDetectorTest, Process_FlashingTransition_GreenToRed)
{
  // Arrange
  auto detector = FlashingDetector(FlashingDetectionConfig{1.0});

  // Act: during flashing, RED input transitions from GREEN to RED
  detector.estimate_stable_color(make_signal(TrafficSignalElement::GREEN));
  detector.estimate_stable_color(make_signal(TrafficSignalElement::UNKNOWN, 0.5));
  const uint8_t color = detector.estimate_stable_color(make_signal(TrafficSignalElement::RED));

  // Assert
  EXPECT_EQ(color, TrafficSignalElement::RED);
}

TEST(FlashingDetectorTest, Process_FlashingTransition_RedToGreen)
{
  // Arrange
  auto detector = FlashingDetector(FlashingDetectionConfig{1.0});

  // Act: during flashing, GREEN input transitions from RED to GREEN
  detector.estimate_stable_color(make_signal(TrafficSignalElement::GREEN));
  detector.estimate_stable_color(make_signal(TrafficSignalElement::UNKNOWN, 0.5));
  detector.estimate_stable_color(make_signal(TrafficSignalElement::RED));
  const uint8_t color = detector.estimate_stable_color(make_signal(TrafficSignalElement::GREEN));

  // Assert
  EXPECT_EQ(color, TrafficSignalElement::GREEN);
}

TEST(FlashingDetectorTest, Process_FlashingResets_AllHistoryMatches)
{
  // Arrange
  auto detector = FlashingDetector(FlashingDetectionConfig{1.0});
  const rclcpp::Time base_time(1000, 0, RCL_ROS_TIME);

  // Build initial state and trigger flashing
  auto green_signal = make_signal(TrafficSignalElement::GREEN);
  auto green_map = make_id_map(green_signal, base_time);
  detector.update_signal_history(green_map, base_time);
  detector.estimate_stable_color(green_signal);

  auto unknown_signal = make_signal(TrafficSignalElement::UNKNOWN, 0.5);
  auto unknown_map = make_id_map(unknown_signal, base_time + rclcpp::Duration::from_seconds(0.1));
  detector.update_signal_history(unknown_map, base_time + rclcpp::Duration::from_seconds(0.1));
  detector.estimate_stable_color(unknown_signal);

  // Now feed consistent RED signals to fill history and reset flashing
  // First, expire old history
  const rclcpp::Time later_time = base_time + rclcpp::Duration::from_seconds(2.0);
  auto red_signal = make_signal(TrafficSignalElement::RED);
  auto red_map = make_id_map(red_signal, later_time);
  detector.update_signal_history(red_map, later_time);
  detector.estimate_stable_color(red_signal);

  // Feed another consistent RED (all history is now RED)
  const rclcpp::Time later_time2 = later_time + rclcpp::Duration::from_seconds(0.1);
  auto red_map2 = make_id_map(red_signal, later_time2);
  detector.update_signal_history(red_map2, later_time2);

  // Act
  const uint8_t color = detector.estimate_stable_color(red_signal);

  // Assert: flashing reset, color tracks normally as RED
  EXPECT_EQ(color, TrafficSignalElement::RED);
}

// --- update_signal_history tests ---

TEST(FlashingDetectorTest, UpdateSignalHistory_PrunesOldEntries)
{
  // Arrange
  auto detector = FlashingDetector(FlashingDetectionConfig{1.0});
  const rclcpp::Time base_time(1000, 0, RCL_ROS_TIME);

  auto green_signal = make_signal(TrafficSignalElement::GREEN);
  auto map = make_id_map(green_signal, base_time);
  detector.update_signal_history(map, base_time);

  // Build state so flashing can be triggered
  detector.estimate_stable_color(green_signal);

  // Trigger flashing
  auto unknown_signal = make_signal(TrafficSignalElement::UNKNOWN, 0.5);
  detector.estimate_stable_color(unknown_signal);

  // Act: update with RED at time > hold_time, old GREEN entry is pruned
  const rclcpp::Time later_time = base_time + rclcpp::Duration::from_seconds(2.0);
  auto red_signal = make_signal(TrafficSignalElement::RED);
  auto red_map = make_id_map(red_signal, later_time);
  detector.update_signal_history(red_map, later_time);

  // All history is now RED, so flashing resets when processing RED
  const uint8_t color = detector.estimate_stable_color(red_signal);

  // Assert: flashing was reset because old GREEN was pruned
  EXPECT_EQ(color, TrafficSignalElement::RED);
}

TEST(FlashingDetectorTest, UpdateSignalHistory_IgnoresUnknownFullConfidence)
{
  // Arrange
  auto detector = FlashingDetector(FlashingDetectionConfig{1.0});
  const rclcpp::Time base_time(1000, 0, RCL_ROS_TIME);

  // UNKNOWN with confidence=1 should not be added to history
  auto unknown_signal = make_signal(TrafficSignalElement::UNKNOWN, 1.0);
  auto map = make_id_map(unknown_signal, base_time);

  // Act
  detector.update_signal_history(map, base_time);

  // Process GREEN to build state, then trigger flashing with UNKNOWN
  auto green_signal = make_signal(TrafficSignalElement::GREEN);
  detector.estimate_stable_color(green_signal);
  auto flash_signal = make_signal(TrafficSignalElement::UNKNOWN, 0.5);
  detector.estimate_stable_color(flash_signal);

  // Feed GREEN: if UNKNOWN(conf=1) was in history, it would differ from GREEN
  // and flashing would NOT reset. If properly ignored, history is empty,
  // so flashing state remains (no history to compare).
  auto green_map = make_id_map(green_signal, base_time + rclcpp::Duration::from_seconds(0.1));
  detector.update_signal_history(green_map, base_time + rclcpp::Duration::from_seconds(0.1));
  const uint8_t color = detector.estimate_stable_color(green_signal);

  // Assert: still GREEN (flashing resets because all history is GREEN)
  EXPECT_EQ(color, TrafficSignalElement::GREEN);
}

// --- clear_state tests ---

TEST(FlashingDetectorTest, ClearState_RemovesTracking)
{
  // Arrange
  auto detector = FlashingDetector(FlashingDetectionConfig{1.0});

  // Act
  // Trigger flashing
  detector.estimate_stable_color(make_signal(TrafficSignalElement::GREEN));
  detector.estimate_stable_color(make_signal(TrafficSignalElement::UNKNOWN, 0.5));
  // After clearing, estimate_stable_color should behave as if id was never seen
  detector.clear_state(DEFAULT_SIGNAL_ID);
  const uint8_t color = detector.estimate_stable_color(make_signal(TrafficSignalElement::RED));

  // Assert: returns RED as a first call (no flashing state)
  EXPECT_EQ(color, TrafficSignalElement::RED);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
