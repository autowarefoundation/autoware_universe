// Copyright 2025 TIER IV, Inc.
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

#include "../src/traffic_light_multi_camera_fusion_process.hpp"

#include <rclcpp/time.hpp>

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>

namespace
{
namespace utils = autoware::traffic_light::utils;
using TrafficLightElement = tier4_perception_msgs::msg::TrafficLightElement;

// colors / shapes used by the tests
constexpr uint8_t red = TrafficLightElement::RED;
constexpr uint8_t green = TrafficLightElement::GREEN;
constexpr uint8_t circle = TrafficLightElement::CIRCLE;
// unknown is shared between the color and shape fields
constexpr uint8_t unknown = TrafficLightElement::UNKNOWN;

// image / roi geometry
constexpr uint32_t image_width = 1440;
constexpr uint32_t image_height = 1080;
constexpr uint32_t roi_size = 100;
constexpr uint32_t boundary_threshold = 5;
// roi offset that keeps the traffic light fully inside the image (fully visible)
constexpr uint32_t visible_offset = 100;
// roi offsets that push the traffic light against each image boundary (truncated)
constexpr uint32_t left_top_boundary_offset = boundary_threshold;
constexpr uint32_t right_boundary_offset = image_width - (roi_size + boundary_threshold);
constexpr uint32_t bottom_boundary_offset = image_height - (roi_size + boundary_threshold);

constexpr int32_t default_stamp_sec = 100;

tier4_perception_msgs::msg::TrafficLight make_signal(
  uint8_t color, uint8_t shape, double confidence)
{
  TrafficLightElement element;
  element.color = color;
  element.shape = shape;
  element.confidence = confidence;

  tier4_perception_msgs::msg::TrafficLight signal;
  signal.elements.push_back(element);
  return signal;
}

utils::FusionRecord make_record(
  uint8_t color, uint8_t shape, double confidence, const std::string & frame_id = "camera0",
  int32_t stamp_sec = default_stamp_sec, uint32_t roi_x_offset = visible_offset,
  uint32_t roi_y_offset = visible_offset)
{
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(stamp_sec, 10);
  header.frame_id = frame_id;

  utils::FusionRecord record;
  record.header = header;
  record.cam_info.header = header;
  record.cam_info.width = image_width;
  record.cam_info.height = image_height;
  record.roi.roi.x_offset = roi_x_offset;
  record.roi.roi.y_offset = roi_y_offset;
  record.roi.roi.width = roi_size;
  record.roi.roi.height = roi_size;
  record.signal = make_signal(color, shape, confidence);
  return record;
}

// only the roi position affects the visible score, so the signal content is fixed here
utils::FusionRecord make_record_with_roi(
  uint32_t roi_x_offset, uint32_t roi_y_offset, const std::string & frame_id = "camera0")
{
  return make_record(red, circle, 0.9, frame_id, default_stamp_sec, roi_x_offset, roi_y_offset);
}
}  // namespace

TEST(IsSignalUnknown, ReturnsTrueForUnknownSignal)
{
  const auto signal = make_signal(unknown, unknown, 0.0);
  EXPECT_TRUE(utils::is_signal_unknown(signal));
}

TEST(IsSignalUnknown, ReturnsFalseForColoredSignal)
{
  const auto signal = make_signal(red, circle, 0.9);
  EXPECT_FALSE(utils::is_signal_unknown(signal));
}

TEST(AtOr, ReturnsMappedValueWhenKeyExists)
{
  const std::unordered_map<int, int> map = {{1, 2}, {3, 4}};
  EXPECT_EQ(utils::at_or(map, 1, -1), 2);
}

TEST(AtOr, ReturnsDefaultValueWhenKeyMissing)
{
  const std::unordered_map<int, int> map = {{1, 2}, {3, 4}};
  EXPECT_EQ(utils::at_or(map, 2, -1), -1);
}

// first condition: records from the same camera are ranked by timestamp
TEST(HasHigherOrEqualPrioritySameCamera, NewerRecordWinsOverOlderRecord)
{
  const auto newer_record = make_record(red, circle, 0.9, "camera0", 200);
  const auto older_record = make_record(green, circle, 0.8, "camera0", 100);
  EXPECT_TRUE(utils::has_higher_or_equal_priority(newer_record, older_record));
}

TEST(HasHigherOrEqualPrioritySameCamera, OlderRecordLosesToNewerRecord)
{
  const auto older_record = make_record(red, circle, 0.9, "camera0", 100);
  const auto newer_record = make_record(green, circle, 0.8, "camera0", 200);
  EXPECT_FALSE(utils::has_higher_or_equal_priority(older_record, newer_record));
}

// second condition: an unknown signal loses to a recognized one
TEST(HasHigherOrEqualPrioritySameCamera, UnknownRecordLosesToKnownRecord)
{
  const auto unknown_record = make_record(unknown, unknown, 0.0);
  const auto known_record = make_record(green, circle, 0.8);
  EXPECT_FALSE(utils::has_higher_or_equal_priority(unknown_record, known_record));
}

TEST(HasHigherOrEqualPrioritySameCamera, KnownRecordWinsOverUnknownRecord)
{
  const auto known_record = make_record(red, circle, 0.9);
  const auto unknown_record = make_record(unknown, unknown, 0.0);
  EXPECT_TRUE(utils::has_higher_or_equal_priority(known_record, unknown_record));
}

TEST(HasHigherOrEqualPrioritySameCamera, BothUnknownRecordsHaveEqualPriority)
{
  const auto unknown_record_1 = make_record(unknown, unknown, 0.0);
  const auto unknown_record_2 = make_record(unknown, unknown, 0.0);
  // equal priority: each record has a higher-or-equal priority than the other
  EXPECT_TRUE(utils::has_higher_or_equal_priority(unknown_record_1, unknown_record_2));
  EXPECT_TRUE(utils::has_higher_or_equal_priority(unknown_record_2, unknown_record_1));
}

// third condition: a fully visible signal wins over a truncated one
TEST(HasHigherOrEqualPrioritySameCamera, VisibleRecordWinsOverTruncatedRecord)
{
  const auto visible_record = make_record_with_roi(visible_offset, visible_offset);
  const auto truncated_record =
    make_record_with_roi(left_top_boundary_offset, left_top_boundary_offset);
  EXPECT_TRUE(utils::has_higher_or_equal_priority(visible_record, truncated_record));
}

TEST(HasHigherOrEqualPrioritySameCamera, TruncatedRecordLosesToVisibleRecord)
{
  const auto truncated_record =
    make_record_with_roi(left_top_boundary_offset, left_top_boundary_offset);
  const auto visible_record = make_record_with_roi(visible_offset, visible_offset);
  EXPECT_FALSE(utils::has_higher_or_equal_priority(truncated_record, visible_record));
}

// fourth condition: a higher confidence signal wins
TEST(HasHigherOrEqualPrioritySameCamera, HigherConfidenceRecordWins)
{
  const auto higher_confidence_record = make_record(red, circle, 0.9);
  const auto lower_confidence_record = make_record(green, circle, 0.8);
  EXPECT_TRUE(
    utils::has_higher_or_equal_priority(higher_confidence_record, lower_confidence_record));
}

TEST(HasHigherOrEqualPrioritySameCamera, LowerConfidenceRecordLoses)
{
  const auto lower_confidence_record = make_record(red, circle, 0.9);
  const auto higher_confidence_record = make_record(green, circle, 0.95);
  EXPECT_FALSE(
    utils::has_higher_or_equal_priority(lower_confidence_record, higher_confidence_record));
}

// the timestamp condition does not apply across different cameras, so the
// remaining conditions are exercised with different frame_ids below

// second condition
TEST(HasHigherOrEqualPriorityDifferentCamera, UnknownRecordLosesToKnownRecord)
{
  const auto unknown_record = make_record(unknown, unknown, 0.0, "camera0");
  const auto known_record = make_record(green, circle, 0.8, "camera1");
  EXPECT_FALSE(utils::has_higher_or_equal_priority(unknown_record, known_record));
}

TEST(HasHigherOrEqualPriorityDifferentCamera, KnownRecordWinsOverUnknownRecord)
{
  const auto known_record = make_record(red, circle, 0.9, "camera0");
  const auto unknown_record = make_record(unknown, unknown, 0.0, "camera1");
  EXPECT_TRUE(utils::has_higher_or_equal_priority(known_record, unknown_record));
}

TEST(HasHigherOrEqualPriorityDifferentCamera, BothUnknownRecordsHaveEqualPriority)
{
  const auto unknown_record_1 = make_record(unknown, unknown, 0.0, "camera0");
  const auto unknown_record_2 = make_record(unknown, unknown, 0.0, "camera1");
  // equal priority: each record has a higher-or-equal priority than the other
  EXPECT_TRUE(utils::has_higher_or_equal_priority(unknown_record_1, unknown_record_2));
  EXPECT_TRUE(utils::has_higher_or_equal_priority(unknown_record_2, unknown_record_1));
}

// third condition
TEST(HasHigherOrEqualPriorityDifferentCamera, VisibleRecordWinsOverTruncatedRecord)
{
  const auto visible_record = make_record_with_roi(visible_offset, visible_offset, "camera0");
  const auto truncated_record =
    make_record_with_roi(left_top_boundary_offset, left_top_boundary_offset, "camera1");
  EXPECT_TRUE(utils::has_higher_or_equal_priority(visible_record, truncated_record));
}

TEST(HasHigherOrEqualPriorityDifferentCamera, TruncatedRecordLosesToVisibleRecord)
{
  const auto truncated_record =
    make_record_with_roi(left_top_boundary_offset, left_top_boundary_offset, "camera0");
  const auto visible_record = make_record_with_roi(visible_offset, visible_offset, "camera1");
  EXPECT_FALSE(utils::has_higher_or_equal_priority(truncated_record, visible_record));
}

// fourth condition
TEST(HasHigherOrEqualPriorityDifferentCamera, HigherConfidenceRecordWins)
{
  const auto higher_confidence_record = make_record(red, circle, 0.9, "camera0");
  const auto lower_confidence_record = make_record(green, circle, 0.8, "camera1");
  EXPECT_TRUE(
    utils::has_higher_or_equal_priority(higher_confidence_record, lower_confidence_record));
}

TEST(HasHigherOrEqualPriorityDifferentCamera, LowerConfidenceRecordLoses)
{
  const auto lower_confidence_record = make_record(red, circle, 0.9, "camera0");
  const auto higher_confidence_record = make_record(green, circle, 0.95, "camera1");
  EXPECT_FALSE(
    utils::has_higher_or_equal_priority(lower_confidence_record, higher_confidence_record));
}

TEST(IsFullyVisible, ReturnsTrueWhenRoiIsCentered)
{
  const auto record = make_record_with_roi(visible_offset, visible_offset);
  EXPECT_TRUE(utils::is_fully_visible(record));
}

TEST(IsFullyVisible, ReturnsFalseWhenRoiNearLeftBoundary)
{
  const auto record = make_record_with_roi(left_top_boundary_offset, visible_offset);
  EXPECT_FALSE(utils::is_fully_visible(record));
}

TEST(IsFullyVisible, ReturnsFalseWhenRoiNearRightBoundary)
{
  const auto record = make_record_with_roi(right_boundary_offset, visible_offset);
  EXPECT_FALSE(utils::is_fully_visible(record));
}

TEST(IsFullyVisible, ReturnsFalseWhenRoiNearTopBoundary)
{
  const auto record = make_record_with_roi(visible_offset, left_top_boundary_offset);
  EXPECT_FALSE(utils::is_fully_visible(record));
}

TEST(IsFullyVisible, ReturnsFalseWhenRoiNearBottomBoundary)
{
  const auto record = make_record_with_roi(visible_offset, bottom_boundary_offset);
  EXPECT_FALSE(utils::is_fully_visible(record));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
