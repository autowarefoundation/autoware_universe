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

#include "../src/multi_camera_fusion.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_element.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <string>
#include <vector>

namespace
{
using autoware::traffic_light::ConflictType;
using autoware::traffic_light::MultiCameraFusion;
using autoware::traffic_light::MultiCameraFusionConfig;
using autoware::traffic_light::MultiCameraFusionResult;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using sensor_msgs::msg::CameraInfo;
using tier4_perception_msgs::msg::TrafficLight;
using tier4_perception_msgs::msg::TrafficLightArray;
using tier4_perception_msgs::msg::TrafficLightRoi;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

using T4Element = tier4_perception_msgs::msg::TrafficLightElement;

// IDs used by the lanelet map fixture. LEFT and RIGHT traffic-light line strings are bound to a
// single regulatory element so that group fusion aggregates evidence from both into one group.
constexpr lanelet::Id LEFT_TRAFFIC_LIGHT_ID = 11;
constexpr lanelet::Id RIGHT_TRAFFIC_LIGHT_ID = 12;
constexpr lanelet::Id REGULATORY_ELEMENT_ID = 100;
constexpr lanelet::Id UNMAPPED_TRAFFIC_LIGHT_ID = 999;

constexpr uint32_t CAMERA_IMAGE_WIDTH = 1440;
constexpr uint32_t CAMERA_IMAGE_HEIGHT = 1080;

// ROI placed safely inside the image so the visibility score equals 1.
constexpr uint32_t ROI_X_OFFSET = 100;
constexpr uint32_t ROI_Y_OFFSET = 100;
constexpr uint32_t ROI_WIDTH = 100;
constexpr uint32_t ROI_HEIGHT = 100;

lanelet::LaneletMapPtr make_lanelet_map_with_two_traffic_lights()
{
  lanelet::Point3d road_left_start(1, 0.0, 2.0, 0.0);
  lanelet::Point3d road_left_end(2, 30.0, 2.0, 0.0);
  lanelet::Point3d road_right_start(3, 0.0, -2.0, 0.0);
  lanelet::Point3d road_right_end(4, 30.0, -2.0, 0.0);
  lanelet::LineString3d road_left(5, {road_left_start, road_left_end});
  lanelet::LineString3d road_right(6, {road_right_start, road_right_end});

  auto road_lanelet = lanelet::Lanelet(1000, road_left, road_right);
  road_lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;

  lanelet::Point3d left_traffic_light_left_end(7, 19.5, 0.5, 3.5);
  lanelet::Point3d left_traffic_light_right_end(8, 19.5, -0.5, 3.5);
  lanelet::LineString3d left_traffic_light(
    LEFT_TRAFFIC_LIGHT_ID, {left_traffic_light_left_end, left_traffic_light_right_end});
  left_traffic_light.attributes()["subtype"] = "red_yellow_green";
  left_traffic_light.attributes()["height"] = "1.0";

  lanelet::Point3d right_traffic_light_left_end(9, 20.5, 0.5, 3.5);
  lanelet::Point3d right_traffic_light_right_end(10, 20.5, -0.5, 3.5);
  lanelet::LineString3d right_traffic_light(
    RIGHT_TRAFFIC_LIGHT_ID, {right_traffic_light_left_end, right_traffic_light_right_end});
  right_traffic_light.attributes()["subtype"] = "red_yellow_green";
  right_traffic_light.attributes()["height"] = "1.0";

  auto traffic_light_regulatory_element = lanelet::autoware::AutowareTrafficLight::make(
    REGULATORY_ELEMENT_ID, lanelet::AttributeMap(), {left_traffic_light, right_traffic_light});
  road_lanelet.addRegulatoryElement(traffic_light_regulatory_element);

  auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet_map->add(road_lanelet);
  return lanelet_map;
}

MultiCameraFusionConfig make_default_config()
{
  MultiCameraFusionConfig config;
  config.message_lifespan = 1.0;
  config.prior_log_odds = 0.0;
  config.use_signal_consistency_check = false;
  config.publish_partial_matched_signal = false;
  config.lanelet_map_ptr = make_lanelet_map_with_two_traffic_lights();
  return config;
}

CameraInfo make_camera_info(const rclcpp::Time & stamp, const std::string & frame_id)
{
  CameraInfo camera_info;
  camera_info.header.stamp = stamp;
  camera_info.header.frame_id = frame_id;
  camera_info.width = CAMERA_IMAGE_WIDTH;
  camera_info.height = CAMERA_IMAGE_HEIGHT;
  return camera_info;
}

TrafficLightRoi make_roi(lanelet::Id traffic_light_id)
{
  TrafficLightRoi roi;
  roi.traffic_light_id = traffic_light_id;
  roi.roi.x_offset = ROI_X_OFFSET;
  roi.roi.y_offset = ROI_Y_OFFSET;
  roi.roi.width = ROI_WIDTH;
  roi.roi.height = ROI_HEIGHT;
  return roi;
}

TrafficLightRoiArray make_roi_array(
  const rclcpp::Time & stamp, const std::string & frame_id, lanelet::Id traffic_light_id)
{
  TrafficLightRoiArray roi_array;
  roi_array.header.stamp = stamp;
  roi_array.header.frame_id = frame_id;
  roi_array.rois.push_back(make_roi(traffic_light_id));
  return roi_array;
}

// ROI placed against the top-left image boundary so is_fully_visible returns false.
TrafficLightRoiArray make_truncated_roi_array(
  const rclcpp::Time & stamp, const std::string & frame_id, lanelet::Id traffic_light_id)
{
  TrafficLightRoi roi;
  roi.traffic_light_id = traffic_light_id;
  roi.roi.x_offset = 0;
  roi.roi.y_offset = 0;
  roi.roi.width = ROI_WIDTH;
  roi.roi.height = ROI_HEIGHT;

  TrafficLightRoiArray roi_array;
  roi_array.header.stamp = stamp;
  roi_array.header.frame_id = frame_id;
  roi_array.rois.push_back(roi);
  return roi_array;
}

TrafficLight make_signal(lanelet::Id traffic_light_id, uint8_t color, float confidence)
{
  T4Element element;
  element.color = color;
  element.shape = T4Element::CIRCLE;
  element.status = T4Element::SOLID_ON;
  element.confidence = confidence;

  TrafficLight signal;
  signal.traffic_light_id = traffic_light_id;
  signal.elements.push_back(element);
  return signal;
}

// Matches utils::is_signal_unknown: a single element whose color and shape are both UNKNOWN.
TrafficLight make_unknown_signal(lanelet::Id traffic_light_id)
{
  T4Element element;
  element.color = T4Element::UNKNOWN;
  element.shape = T4Element::UNKNOWN;
  element.status = T4Element::SOLID_ON;
  element.confidence = 0.0f;

  TrafficLight signal;
  signal.traffic_light_id = traffic_light_id;
  signal.elements.push_back(element);
  return signal;
}

TrafficLight make_signal_with_left_arrow(
  lanelet::Id traffic_light_id, uint8_t color, float circle_confidence, float arrow_confidence)
{
  TrafficLight signal = make_signal(traffic_light_id, color, circle_confidence);
  T4Element arrow_element;
  arrow_element.color = color;
  arrow_element.shape = T4Element::LEFT_ARROW;
  arrow_element.status = T4Element::SOLID_ON;
  arrow_element.confidence = arrow_confidence;
  signal.elements.push_back(arrow_element);
  return signal;
}

TrafficLightArray make_signal_array(
  const rclcpp::Time & stamp, const std::string & frame_id, const TrafficLight & signal)
{
  TrafficLightArray signal_array;
  signal_array.header.stamp = stamp;
  signal_array.header.frame_id = frame_id;
  signal_array.signals.push_back(signal);
  return signal_array;
}

struct FusionInput
{
  CameraInfo camera_info;
  TrafficLightRoiArray roi_array;
  TrafficLightArray signal_array;
};

FusionInput make_fusion_input(
  const std::string & frame_id, const TrafficLight & signal,
  const rclcpp::Time & stamp = rclcpp::Time(0, 0))
{
  return FusionInput{
    make_camera_info(stamp, frame_id), make_roi_array(stamp, frame_id, signal.traffic_light_id),
    make_signal_array(stamp, frame_id, signal)};
}

void expect_single_fused_color(const TrafficLightGroupArray & groups, uint8_t expected_color)
{
  ASSERT_EQ(groups.traffic_light_groups.size(), 1u);
  const auto & group = groups.traffic_light_groups.front();
  ASSERT_EQ(group.elements.size(), 1u);
  EXPECT_EQ(group.elements.front().color, expected_color);
}

void expect_single_fused_color_and_shape(
  const TrafficLightGroupArray & groups, uint8_t expected_color, uint8_t expected_shape)
{
  ASSERT_EQ(groups.traffic_light_groups.size(), 1u);
  const auto & group = groups.traffic_light_groups.front();
  ASSERT_EQ(group.elements.size(), 1u);
  EXPECT_EQ(group.elements.front().color, expected_color);
  EXPECT_EQ(group.elements.front().shape, expected_shape);
}

void expect_single_conflict_status(
  const MultiCameraFusionResult & result, ConflictType expected_conflict_type)
{
  ASSERT_EQ(result.conflicted_regulatory_element_status.size(), 1u);
  EXPECT_EQ(
    result.conflicted_regulatory_element_status.front().conflict_type, expected_conflict_type);
}

void expect_element_confidence(
  const TrafficLightGroupArray & groups, size_t element_index, float expected_confidence)
{
  ASSERT_EQ(groups.traffic_light_groups.size(), 1u);
  const auto & group = groups.traffic_light_groups.front();
  ASSERT_LT(element_index, group.elements.size());
  EXPECT_FLOAT_EQ(group.elements[element_index].confidence, expected_confidence);
}

}  // namespace

TEST(MultiCameraFusionFuse, SingleCameraSingleLightOutputsGroupWithMappedRegulatoryId)
{
  // Arrange
  MultiCameraFusion fusion(make_default_config());
  const auto input =
    make_fusion_input("camera0", make_signal(LEFT_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.9f));

  // Act
  TrafficLightGroupArray groups;
  const auto result = fusion.fuse(input.camera_info, input.roi_array, input.signal_array, groups);

  // Assert
  ASSERT_EQ(groups.traffic_light_groups.size(), 1u);
  const auto & group = groups.traffic_light_groups.front();
  EXPECT_EQ(group.traffic_light_group_id, REGULATORY_ELEMENT_ID);
  ASSERT_EQ(group.elements.size(), 1u);
  EXPECT_EQ(group.elements.front().color, TrafficLightElement::GREEN);
  EXPECT_EQ(group.elements.front().shape, TrafficLightElement::CIRCLE);
  EXPECT_TRUE(result.unmapped_traffic_light_ids.empty());
}

TEST(MultiCameraFusionFuse, EmptyRoiArrayProducesEmptyTrafficLightGroups)
{
  // Arrange
  MultiCameraFusion fusion(make_default_config());
  const rclcpp::Time stamp(100, 0);
  const std::string frame_id = "camera0";
  TrafficLightRoiArray empty_rois;
  empty_rois.header.stamp = stamp;
  empty_rois.header.frame_id = frame_id;
  TrafficLightArray empty_signals;
  empty_signals.header.stamp = stamp;
  empty_signals.header.frame_id = frame_id;

  // Act
  TrafficLightGroupArray groups;
  const auto result =
    fusion.fuse(make_camera_info(stamp, frame_id), empty_rois, empty_signals, groups);

  // Assert
  EXPECT_TRUE(groups.traffic_light_groups.empty());
  EXPECT_TRUE(result.unmapped_traffic_light_ids.empty());
}

TEST(MultiCameraFusionFuse, UnknownTrafficLightIdIsRecordedAsUnmapped)
{
  // Arrange
  MultiCameraFusion fusion(make_default_config());
  const auto input =
    make_fusion_input("camera0", make_signal(UNMAPPED_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.9f));

  // Act
  TrafficLightGroupArray groups;
  const auto result = fusion.fuse(input.camera_info, input.roi_array, input.signal_array, groups);

  // Assert
  EXPECT_TRUE(groups.traffic_light_groups.empty());
  ASSERT_EQ(result.unmapped_traffic_light_ids.size(), 1u);
  EXPECT_EQ(result.unmapped_traffic_light_ids.front(), UNMAPPED_TRAFFIC_LIGHT_ID);
}

TEST(MultiCameraFusionFuse, RoiWithoutMatchingSignalIsIgnored)
{
  // Arrange
  MultiCameraFusion fusion(make_default_config());
  const rclcpp::Time stamp(100, 0);
  const std::string frame_id = "camera0";

  // signal id does not match the roi id -> the roi cannot be paired with a signal
  TrafficLightArray mismatched_signals =
    make_signal_array(stamp, frame_id, make_signal(RIGHT_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.9f));

  // Act
  TrafficLightGroupArray groups;
  fusion.fuse(
    make_camera_info(stamp, frame_id), make_roi_array(stamp, frame_id, LEFT_TRAFFIC_LIGHT_ID),
    mismatched_signals, groups);

  // Assert
  EXPECT_TRUE(groups.traffic_light_groups.empty());
}

TEST(MultiCameraFusionFuse, HigherConfidenceColorIsSelectedAcrossTwoLights)
{
  // Arrange
  // Two traffic lights belong to the same regulatory element. When their colors disagree,
  // the color reported with the higher confidence (GREEN at 0.9 vs RED at 0.6) is selected.
  MultiCameraFusion fusion(make_default_config());
  const auto input0 =
    make_fusion_input("camera0", make_signal(LEFT_TRAFFIC_LIGHT_ID, T4Element::RED, 0.6f));
  const auto input1 =
    make_fusion_input("camera1", make_signal(RIGHT_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.9f));

  // Act
  TrafficLightGroupArray groups;
  fusion.fuse(input0.camera_info, input0.roi_array, input0.signal_array, groups);
  fusion.fuse(input1.camera_info, input1.roi_array, input1.signal_array, groups);

  // Assert
  expect_single_fused_color(groups, TrafficLightElement::GREEN);
}

TEST(MultiCameraFusionFuse, RecordOlderThanMessageLifespanIsDiscarded)
{
  // Arrange
  // message_lifespan=1.0s. First record at t=100s, second at t=102s.
  // Difference (2s) exceeds the lifespan, so the first record is purged before group fusion.
  auto config = make_default_config();
  config.message_lifespan = 1.0;
  MultiCameraFusion fusion(config);
  const auto input0 = make_fusion_input(
    "camera0", make_signal(LEFT_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.9f), rclcpp::Time(100, 0));
  const auto input1 = make_fusion_input(
    "camera1", make_signal(RIGHT_TRAFFIC_LIGHT_ID, T4Element::RED, 0.6f), rclcpp::Time(102, 0));

  // Act
  TrafficLightGroupArray groups;
  fusion.fuse(input0.camera_info, input0.roi_array, input0.signal_array, groups);
  fusion.fuse(input1.camera_info, input1.roi_array, input1.signal_array, groups);

  // Assert
  // Only the second record contributes -> single light, RED color.
  expect_single_fused_color(groups, TrafficLightElement::RED);
}

TEST(MultiCameraFusionFuse, RecordWithinMessageLifespanIsKeptAndAccumulated)
{
  // Arrange
  // message_lifespan=2.0s, records at t=100s and t=101s. Both records remain.
  auto config = make_default_config();
  config.message_lifespan = 2.0;
  MultiCameraFusion fusion(config);
  const auto input0 = make_fusion_input(
    "camera0", make_signal(LEFT_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.9f), rclcpp::Time(100, 0));
  const auto input1 = make_fusion_input(
    "camera1", make_signal(RIGHT_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.7f), rclcpp::Time(101, 0));

  // Act
  TrafficLightGroupArray groups;
  fusion.fuse(input0.camera_info, input0.roi_array, input0.signal_array, groups);
  fusion.fuse(input1.camera_info, input1.roi_array, input1.signal_array, groups);

  // Assert
  // Both records aggregate into a single group (same regulatory element) with color GREEN.
  expect_single_fused_color(groups, TrafficLightElement::GREEN);
}

TEST(MultiCameraFusionFuse, ConsistencyCheckWithSameColorOutputsNoConflict)
{
  // Arrange
  auto config = make_default_config();
  config.use_signal_consistency_check = true;
  MultiCameraFusion fusion(config);
  const auto input0 =
    make_fusion_input("camera0", make_signal(LEFT_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.9f));
  const auto input1 =
    make_fusion_input("camera1", make_signal(RIGHT_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.9f));

  // Act
  TrafficLightGroupArray groups;
  fusion.fuse(input0.camera_info, input0.roi_array, input0.signal_array, groups);
  const auto result =
    fusion.fuse(input1.camera_info, input1.roi_array, input1.signal_array, groups);

  // Assert
  expect_single_fused_color(groups, TrafficLightElement::GREEN);
  EXPECT_TRUE(result.conflicted_regulatory_element_status.empty());
}

TEST(MultiCameraFusionFuse, ConsistencyCheckWithConflictingColorsOutputsUnknownFailsafe)
{
  // Arrange
  // Both cameras see the same regulatory element with different colors (no shared state).
  // With consistency_check enabled and partial_match publishing disabled, the result is a
  // fail-safe UNKNOWN group and the conflict is recorded as CONFLICT.
  auto config = make_default_config();
  config.use_signal_consistency_check = true;
  config.publish_partial_matched_signal = false;
  MultiCameraFusion fusion(config);
  const auto input0 =
    make_fusion_input("camera0", make_signal(LEFT_TRAFFIC_LIGHT_ID, T4Element::RED, 0.9f));
  const auto input1 =
    make_fusion_input("camera1", make_signal(RIGHT_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.9f));

  // Act
  TrafficLightGroupArray groups;
  fusion.fuse(input0.camera_info, input0.roi_array, input0.signal_array, groups);
  const auto result =
    fusion.fuse(input1.camera_info, input1.roi_array, input1.signal_array, groups);

  // Assert
  expect_single_fused_color_and_shape(
    groups, TrafficLightElement::UNKNOWN, TrafficLightElement::UNKNOWN);
  expect_single_conflict_status(result, ConflictType::CONFLICT);
}

TEST(MultiCameraFusionFuse, TruncatedRoiHasLowerPriorityThanCenteredRoi)
{
  // Arrange
  // Two cameras observe the same traffic_light_id. camera0 reports a truncated ROI
  // (truncated) with the higher confidence; camera1 reports a centered ROI
  // (fully visible) with the lower confidence. has_higher_or_equal_priority's visibility check
  // ranks above the confidence check, so camera1's record is selected and the fused output is RED.
  MultiCameraFusion fusion(make_default_config());
  const rclcpp::Time stamp(100, 0);
  const auto truncated_camera_info = make_camera_info(stamp, "camera0");
  const auto truncated_rois = make_truncated_roi_array(stamp, "camera0", LEFT_TRAFFIC_LIGHT_ID);
  const auto truncated_signals =
    make_signal_array(stamp, "camera0", make_signal(LEFT_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.9f));
  const auto centered_input =
    make_fusion_input("camera1", make_signal(LEFT_TRAFFIC_LIGHT_ID, T4Element::RED, 0.6f), stamp);

  // Act
  TrafficLightGroupArray groups;
  fusion.fuse(truncated_camera_info, truncated_rois, truncated_signals, groups);
  fusion.fuse(
    centered_input.camera_info, centered_input.roi_array, centered_input.signal_array, groups);

  // Assert
  expect_single_fused_color(groups, TrafficLightElement::RED);
}

TEST(MultiCameraFusionFuse, UnknownSignalLosesToValidSignalForSameTrafficLightId)
{
  // Arrange
  // Two cameras observe the same traffic_light_id. camera0 reports an UNKNOWN signal;
  // camera1 reports GREEN. has_higher_or_equal_priority's unknown check drops the unknown record,
  // so the fused output reflects camera1.
  MultiCameraFusion fusion(make_default_config());
  const auto unknown_input =
    make_fusion_input("camera0", make_unknown_signal(LEFT_TRAFFIC_LIGHT_ID));
  const auto valid_input =
    make_fusion_input("camera1", make_signal(LEFT_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.8f));

  // Act
  TrafficLightGroupArray groups;
  fusion.fuse(
    unknown_input.camera_info, unknown_input.roi_array, unknown_input.signal_array, groups);
  fusion.fuse(valid_input.camera_info, valid_input.roi_array, valid_input.signal_array, groups);

  // Assert
  expect_single_fused_color(groups, TrafficLightElement::GREEN);
}

TEST(MultiCameraFusionFuse, NewerTimestampWinsForSameFrameIdAndTrafficLightId)
{
  // Arrange
  // The same camera publishes two observations of the same traffic_light_id within the lifespan.
  // has_higher_or_equal_priority's first priority (same frame_id with timestamps differing by >= 1
  // ms) prefers the newer record regardless of confidence, so the later RED supersedes the earlier
  // GREEN.
  MultiCameraFusion fusion(make_default_config());
  const std::string frame_id = "camera0";
  const auto earlier_input = make_fusion_input(
    frame_id, make_signal(LEFT_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.9f), rclcpp::Time(100, 0));
  const auto later_input = make_fusion_input(
    frame_id, make_signal(LEFT_TRAFFIC_LIGHT_ID, T4Element::RED, 0.4f),
    rclcpp::Time(100, 500000000));

  // Act
  TrafficLightGroupArray groups;
  fusion.fuse(
    earlier_input.camera_info, earlier_input.roi_array, earlier_input.signal_array, groups);
  fusion.fuse(later_input.camera_info, later_input.roi_array, later_input.signal_array, groups);

  // Assert
  expect_single_fused_color(groups, TrafficLightElement::RED);
}

TEST(MultiCameraFusionFuse, HigherConfidenceWinsWhenBothFullyVisibleForSameTrafficLightId)
{
  // Arrange
  // Two cameras observe the same traffic_light_id with centered ROIs (fully visible each).
  // has_higher_or_equal_priority falls through to its last priority — the confidence
  // comparison — and selects the higher-confidence RED over the lower-confidence GREEN.
  MultiCameraFusion fusion(make_default_config());
  const auto low_confidence_input =
    make_fusion_input("camera0", make_signal(LEFT_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.5f));
  const auto high_confidence_input =
    make_fusion_input("camera1", make_signal(LEFT_TRAFFIC_LIGHT_ID, T4Element::RED, 0.9f));

  // Act
  TrafficLightGroupArray groups;
  fusion.fuse(
    low_confidence_input.camera_info, low_confidence_input.roi_array,
    low_confidence_input.signal_array, groups);
  fusion.fuse(
    high_confidence_input.camera_info, high_confidence_input.roi_array,
    high_confidence_input.signal_array, groups);

  // Assert
  expect_single_fused_color(groups, TrafficLightElement::RED);
}

TEST(MultiCameraFusionFuse, PartialConflictWithPartialMatchEnabledPublishesCommonState)
{
  // Arrange
  // Two traffic lights in the same regulatory element report partially-overlapping states:
  //   LEFT  -> [(RED, CIRCLE)]
  //   RIGHT -> [(RED, CIRCLE), (RED, LEFT_ARROW)]
  // With consistency_check enabled and partial-match publishing enabled, the merged output keeps
  // only the common (RED, CIRCLE) element and the conflict is reported as PARTIAL_CONFLICT.
  auto config = make_default_config();
  config.use_signal_consistency_check = true;
  config.publish_partial_matched_signal = true;
  MultiCameraFusion fusion(config);
  const auto input0 =
    make_fusion_input("camera0", make_signal(LEFT_TRAFFIC_LIGHT_ID, T4Element::RED, 0.9f));
  const auto input1 = make_fusion_input(
    "camera1", make_signal_with_left_arrow(RIGHT_TRAFFIC_LIGHT_ID, T4Element::RED, 0.9f, 0.9f));

  // Act
  TrafficLightGroupArray groups;
  fusion.fuse(input0.camera_info, input0.roi_array, input0.signal_array, groups);
  const auto result =
    fusion.fuse(input1.camera_info, input1.roi_array, input1.signal_array, groups);

  // Assert
  expect_single_fused_color_and_shape(
    groups, TrafficLightElement::RED, TrafficLightElement::CIRCLE);
  expect_single_conflict_status(result, ConflictType::PARTIAL_CONFLICT);
}

TEST(MultiCameraFusionFuse, MinElementConfidenceDeterminesWinnerForMultiElementSignals)
{
  // Arrange
  // Two cameras observe the same traffic_light_id with centered ROIs (both fully visible).
  // Each signal has CIRCLE + LEFT_ARROW with differing per-element confidences:
  //   camera0: {CIRCLE 0.8, LEFT_ARROW 0.8}  -> min = 0.8
  //   camera1: {CIRCLE 0.9, LEFT_ARROW 0.3}  -> min = 0.3
  // has_higher_or_equal_priority falls through to the confidence check, which uses
  // utils::get_min_confidence. With min-aggregation, camera0 wins (0.8 > 0.3); with
  // max-aggregation, camera1 would win (0.9 > 0.8). The output's preserved per-element confidences
  // distinguish the two cases.
  MultiCameraFusion fusion(make_default_config());
  const auto input0 = make_fusion_input(
    "camera0", make_signal_with_left_arrow(LEFT_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.8f, 0.8f));
  const auto input1 = make_fusion_input(
    "camera1", make_signal_with_left_arrow(LEFT_TRAFFIC_LIGHT_ID, T4Element::GREEN, 0.9f, 0.3f));

  // Act
  TrafficLightGroupArray groups;
  fusion.fuse(input0.camera_info, input0.roi_array, input0.signal_array, groups);
  fusion.fuse(input1.camera_info, input1.roi_array, input1.signal_array, groups);

  // Assert
  // camera0 wins -> output preserves camera0's per-element confidences (0.8 each).
  expect_element_confidence(groups, 0, 0.8f);
  expect_element_confidence(groups, 1, 0.8f);
}

TEST(MultiCameraFusionFuse, PartialConflictWithPartialMatchDisabledPublishesFailsafe)
{
  // Arrange
  // Same partial-overlap setup as the previous test, but with partial-match publishing disabled.
  // The fused output is replaced with a fail-safe UNKNOWN element, and the conflict is still
  // reported as PARTIAL_CONFLICT (not CONFLICT).
  auto config = make_default_config();
  config.use_signal_consistency_check = true;
  config.publish_partial_matched_signal = false;
  MultiCameraFusion fusion(config);
  const auto input0 =
    make_fusion_input("camera0", make_signal(LEFT_TRAFFIC_LIGHT_ID, T4Element::RED, 0.9f));
  const auto input1 = make_fusion_input(
    "camera1", make_signal_with_left_arrow(RIGHT_TRAFFIC_LIGHT_ID, T4Element::RED, 0.9f, 0.9f));

  // Act
  TrafficLightGroupArray groups;
  fusion.fuse(input0.camera_info, input0.roi_array, input0.signal_array, groups);
  const auto result =
    fusion.fuse(input1.camera_info, input1.roi_array, input1.signal_array, groups);

  // Assert
  expect_single_fused_color_and_shape(
    groups, TrafficLightElement::UNKNOWN, TrafficLightElement::UNKNOWN);
  expect_single_conflict_status(result, ConflictType::PARTIAL_CONFLICT);
}
