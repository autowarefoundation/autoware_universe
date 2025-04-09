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

#include "../src/traffic_light_selector_utils.hpp"

#include <gtest/gtest.h>

TEST(isInsideRoughRoi, inside)
{
  sensor_msgs::msg::RegionOfInterest detected_roi;
  detected_roi.x_offset = 10;
  detected_roi.y_offset = 10;
  detected_roi.width = 15;
  detected_roi.height = 15;

  sensor_msgs::msg::RegionOfInterest rough_roi;
  rough_roi.x_offset = 5;
  rough_roi.y_offset = 5;
  rough_roi.width = 20;
  rough_roi.height = 20;

  const bool is_inside = autoware::traffic_light::utils::isInsideRoughRoi(detected_roi, rough_roi);
  EXPECT_TRUE(is_inside);
}

TEST(isInsideRoughRoi, inside_boundary)
{
  sensor_msgs::msg::RegionOfInterest detected_roi;
  detected_roi.x_offset = 0;
  detected_roi.y_offset = 0;
  detected_roi.width = 10;
  detected_roi.height = 10;

  sensor_msgs::msg::RegionOfInterest rough_roi;
  rough_roi.x_offset = 0;
  rough_roi.y_offset = 0;
  rough_roi.width = 10;
  rough_roi.height = 10;

  const bool is_inside = autoware::traffic_light::utils::isInsideRoughRoi(detected_roi, rough_roi);
  EXPECT_TRUE(is_inside);
}

TEST(isInsideRoughRoi, outside)
{
  sensor_msgs::msg::RegionOfInterest detected_roi;
  detected_roi.x_offset = 0;
  detected_roi.y_offset = 0;
  detected_roi.width = 10;
  detected_roi.height = 10;

  sensor_msgs::msg::RegionOfInterest rough_roi;
  rough_roi.x_offset = 5;
  rough_roi.y_offset = 5;
  rough_roi.width = 10;
  rough_roi.height = 10;

  const bool is_inside = autoware::traffic_light::utils::isInsideRoughRoi(detected_roi, rough_roi);
  EXPECT_FALSE(is_inside);
}

TEST(computeCenterOffset, normal)
{
  sensor_msgs::msg::RegionOfInterest source;
  source.x_offset = 40;
  source.y_offset = 35;
  source.width = 10;
  source.height = 10;

  sensor_msgs::msg::RegionOfInterest target;
  target.x_offset = 20;
  target.y_offset = 5;
  target.width = 30;
  target.height = 20;

  int32_t shift_x, shift_y;
  autoware::traffic_light::utils::computeCenterOffset(source, target, shift_x, shift_y);
  EXPECT_EQ(shift_x, 10);
  EXPECT_EQ(shift_y, 25);
}

TEST(computeCenterOffset, same)
{
  sensor_msgs::msg::RegionOfInterest source;
  source.x_offset = 0;
  source.y_offset = 0;
  source.width = 10;
  source.height = 10;

  sensor_msgs::msg::RegionOfInterest target;
  target.x_offset = 0;
  target.y_offset = 0;
  target.width = 10;
  target.height = 10;

  int32_t shift_x, shift_y;
  autoware::traffic_light::utils::computeCenterOffset(source, target, shift_x, shift_y);
  EXPECT_EQ(shift_x, 0);
  EXPECT_EQ(shift_y, 0);
}

TEST(computeCenterOffset, out_of_natural_range)
{
  sensor_msgs::msg::RegionOfInterest source;
  source.x_offset = 5;
  source.y_offset = 10;
  source.width = 10;
  source.height = 10;

  sensor_msgs::msg::RegionOfInterest target;
  target.x_offset = 40;
  target.y_offset = 55;
  target.width = 10;
  target.height = 10;

  int32_t shift_x, shift_y;
  autoware::traffic_light::utils::computeCenterOffset(source, target, shift_x, shift_y);
  EXPECT_EQ(shift_x, -35);
  EXPECT_EQ(shift_y, -45);
}

TEST(getShiftedRoi, normal)
{
  const uint32_t image_width = 100;
  const uint32_t image_height = 100;

  sensor_msgs::msg::RegionOfInterest source;
  source.x_offset = 0;
  source.y_offset = 0;
  source.width = 10;
  source.height = 10;

  const int32_t shift_x = 5;
  const int32_t shift_y = 5;
  const sensor_msgs::msg::RegionOfInterest source_shifted =
    autoware::traffic_light::utils::getShiftedRoi(
      source, image_width, image_height, shift_x, shift_y);
  EXPECT_EQ(source_shifted.x_offset, 5);
  EXPECT_EQ(source_shifted.y_offset, 5);
  EXPECT_EQ(source_shifted.width, 10);
  EXPECT_EQ(source_shifted.height, 10);
}

TEST(getShiftedRoi, out_of_range_in_top)
{
  const uint32_t image_width = 100;
  const uint32_t image_height = 100;

  sensor_msgs::msg::RegionOfInterest source;
  source.x_offset = 70;
  source.y_offset = 15;
  source.width = 20;
  source.height = 40;

  const int32_t shift_x = 5;
  const int32_t shift_y = -20;
  const sensor_msgs::msg::RegionOfInterest source_shifted =
    autoware::traffic_light::utils::getShiftedRoi(
      source, image_width, image_height, shift_x, shift_y);
  EXPECT_EQ(source_shifted.x_offset, 0);
  EXPECT_EQ(source_shifted.y_offset, 0);
  EXPECT_EQ(source_shifted.width, 0);
  EXPECT_EQ(source_shifted.height, 0);
}

TEST(getShiftedRoi, out_of_range_in_right)
{
  const uint32_t image_width = 100;
  const uint32_t image_height = 100;

  sensor_msgs::msg::RegionOfInterest source;
  source.x_offset = 30;
  source.y_offset = 5;
  source.width = 60;
  source.height = 30;

  const int32_t shift_x = 20;
  const int32_t shift_y = 5;
  const sensor_msgs::msg::RegionOfInterest source_shifted =
    autoware::traffic_light::utils::getShiftedRoi(
      source, image_width, image_height, shift_x, shift_y);
  EXPECT_EQ(source_shifted.x_offset, 0);
  EXPECT_EQ(source_shifted.y_offset, 0);
  EXPECT_EQ(source_shifted.width, 0);
  EXPECT_EQ(source_shifted.height, 0);
}

TEST(getShiftedRoi, out_of_range_in_left)
{
  const uint32_t image_width = 100;
  const uint32_t image_height = 100;

  sensor_msgs::msg::RegionOfInterest source;
  source.x_offset = 5;
  source.y_offset = 55;
  source.width = 40;
  source.height = 10;

  const int32_t shift_x = -30;
  const int32_t shift_y = 5;
  const sensor_msgs::msg::RegionOfInterest source_shifted =
    autoware::traffic_light::utils::getShiftedRoi(
      source, image_width, image_height, shift_x, shift_y);
  EXPECT_EQ(source_shifted.x_offset, 0);
  EXPECT_EQ(source_shifted.y_offset, 0);
  EXPECT_EQ(source_shifted.width, 0);
  EXPECT_EQ(source_shifted.height, 0);
}

TEST(getShiftedRoi, out_of_range_in_bottom)
{
  const uint32_t image_width = 100;
  const uint32_t image_height = 100;

  sensor_msgs::msg::RegionOfInterest source;
  source.x_offset = 5;
  source.y_offset = 60;
  source.width = 30;
  source.height = 30;

  const int32_t shift_x = 10;
  const int32_t shift_y = 50;
  const sensor_msgs::msg::RegionOfInterest source_shifted =
    autoware::traffic_light::utils::getShiftedRoi(
      source, image_width, image_height, shift_x, shift_y);
  EXPECT_EQ(source_shifted.x_offset, 0);
  EXPECT_EQ(source_shifted.y_offset, 0);
  EXPECT_EQ(source_shifted.width, 0);
  EXPECT_EQ(source_shifted.height, 0);
}

TEST(getIoUgetGenIoU, a_part_of_overlap)
{
  sensor_msgs::msg::RegionOfInterest bbox1;
  bbox1.x_offset = 0;
  bbox1.y_offset = 0;
  bbox1.width = 10;
  bbox1.height = 10;

  sensor_msgs::msg::RegionOfInterest bbox2;
  bbox2.x_offset = 5;
  bbox2.y_offset = 5;
  bbox2.width = 10;
  bbox2.height = 10;

  const double iou = autoware::traffic_light::utils::getIoU(bbox1, bbox2);
  EXPECT_FLOAT_EQ(iou, 0.14285714285);

  const double gen_iou = autoware::traffic_light::utils::getGenIoU(bbox1, bbox2);
  EXPECT_FLOAT_EQ(gen_iou, -0.07936507937);
}

TEST(getIoUgetGenIoU, overlap)
{
  sensor_msgs::msg::RegionOfInterest bbox1;
  bbox1.x_offset = 0;
  bbox1.y_offset = 0;
  bbox1.width = 10;
  bbox1.height = 10;

  sensor_msgs::msg::RegionOfInterest bbox2;
  bbox2.x_offset = 5;
  bbox2.y_offset = 5;
  bbox2.width = 5;
  bbox2.height = 5;

  const double iou = autoware::traffic_light::utils::getIoU(bbox1, bbox2);
  EXPECT_FLOAT_EQ(iou, 0.25);

  const double gen_iou = autoware::traffic_light::utils::getGenIoU(bbox1, bbox2);
  EXPECT_FLOAT_EQ(gen_iou, 0.25);
}

TEST(getIoUgetGenIoU, no_overlap)
{
  sensor_msgs::msg::RegionOfInterest bbox1;
  bbox1.x_offset = 0;
  bbox1.y_offset = 0;
  bbox1.width = 10;
  bbox1.height = 10;

  sensor_msgs::msg::RegionOfInterest bbox2;
  bbox2.x_offset = 15;
  bbox2.y_offset = 15;
  bbox2.width = 10;
  bbox2.height = 10;

  const double iou = autoware::traffic_light::utils::getIoU(bbox1, bbox2);
  EXPECT_FLOAT_EQ(iou, 0.0);

  const double gen_iou = autoware::traffic_light::utils::getGenIoU(bbox1, bbox2);
  EXPECT_FLOAT_EQ(gen_iou, -0.68);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
