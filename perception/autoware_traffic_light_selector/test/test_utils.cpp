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

#include "../src/utils.hpp"

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

TEST(getShiftedRoi, normal)
{
  sensor_msgs::msg::RegionOfInterest source;
  source.x_offset = 0;
  source.y_offset = 0;
  source.width = 10;
  source.height = 10;

  sensor_msgs::msg::RegionOfInterest target;
  target.x_offset = 5;
  target.y_offset = 5;
  target.width = 10;
  target.height = 10;

  int32_t shift_x, shift_y;
  const sensor_msgs::msg::RegionOfInterest source_shifted =
    autoware::traffic_light::utils::getShiftedRoi(source, target, shift_x, shift_y);
  EXPECT_EQ(shift_x, -5);
  EXPECT_EQ(shift_y, -5);
  EXPECT_EQ(source_shifted.x_offset, 5);
  EXPECT_EQ(source_shifted.y_offset, 5);
  EXPECT_EQ(source_shifted.width, 10);
  EXPECT_EQ(source_shifted.height, 10);
}

TEST(getShiftedRoi, out_of_range)
{
  sensor_msgs::msg::RegionOfInterest source;
  source.x_offset = 5;
  source.y_offset = 5;
  source.width = 20;
  source.height = 20;

  sensor_msgs::msg::RegionOfInterest target;
  target.x_offset = 0;
  target.y_offset = 0;
  target.width = 10;
  target.height = 10;

  int32_t shift_x, shift_y;
  const sensor_msgs::msg::RegionOfInterest source_shifted =
    autoware::traffic_light::utils::getShiftedRoi(source, target, shift_x, shift_y);
  EXPECT_EQ(shift_x, 10);
  EXPECT_EQ(shift_y, 10);
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
  EXPECT_NEAR(iou, 0.14285714285, 1e-6);

  const double gen_iou = autoware::traffic_light::utils::getGenIoU(bbox1, bbox2);
  EXPECT_NEAR(gen_iou, -0.07936507937, 1e-6);
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
  EXPECT_NEAR(iou, 0.25, 1e-6);

  const double gen_iou = autoware::traffic_light::utils::getGenIoU(bbox1, bbox2);
  EXPECT_NEAR(gen_iou, 0.25, 1e-6);
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
  EXPECT_NEAR(iou, 0.0, 1e-6);

  const double gen_iou = autoware::traffic_light::utils::getGenIoU(bbox1, bbox2);
  EXPECT_NEAR(gen_iou, -0.68, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}