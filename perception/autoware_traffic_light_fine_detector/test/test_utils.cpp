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

#include "../src/traffic_light_fine_detector_process.hpp"

#include <gtest/gtest.h>

TEST(calWeightedIou, a_part_of_overlap)
{
  sensor_msgs::msg::RegionOfInterest roi;
  roi.x_offset = 0;
  roi.y_offset = 0;
  roi.width = 10;
  roi.height = 10;

  autoware::tensorrt_yolox::Object obj;
  obj.x_offset = 5;
  obj.y_offset = 5;
  obj.width = 10;
  obj.height = 10;
  obj.score = 0.5;

  float iou = autoware::traffic_light::utils::calWeightedIou(roi, obj);
  EXPECT_FLOAT_EQ(iou, 0.07142857143);
}

TEST(calWeightedIou, overlap)
{
  sensor_msgs::msg::RegionOfInterest roi;
  roi.x_offset = 0;
  roi.y_offset = 0;
  roi.width = 10;
  roi.height = 10;

  autoware::tensorrt_yolox::Object obj;
  obj.x_offset = 5;
  obj.y_offset = 5;
  obj.width = 5;
  obj.height = 5;
  obj.score = 0.5;

  float iou = autoware::traffic_light::utils::calWeightedIou(roi, obj);
  EXPECT_FLOAT_EQ(iou, 0.125);
}

TEST(calWeightedIou, no_overlap)
{
  sensor_msgs::msg::RegionOfInterest roi;
  roi.x_offset = 0;
  roi.y_offset = 0;
  roi.width = 10;
  roi.height = 10;

  autoware::tensorrt_yolox::Object obj;
  obj.x_offset = 15;
  obj.y_offset = 15;
  obj.width = 10;
  obj.height = 10;
  obj.score = 0.5;

  float iou = autoware::traffic_light::utils::calWeightedIou(roi, obj);
  EXPECT_FLOAT_EQ(iou, 0.0);
}

TEST(fitInFrame, in_range)
{
  cv::Point lt(1, 2);
  cv::Point rb(11, 22);
  cv::Size size(20, 30);

  bool result = autoware::traffic_light::utils::fitInFrame(lt, rb, size);
  EXPECT_TRUE(result);
  EXPECT_EQ(lt.x, 1);
  EXPECT_EQ(lt.y, 2);
  EXPECT_EQ(rb.x, 11);
  EXPECT_EQ(rb.y, 22);
}

TEST(fitInFrame, out_of_range_left_top)
{
  cv::Point lt(-1, -2);
  cv::Point rb(11, 22);
  cv::Size size(20, 30);

  bool result = autoware::traffic_light::utils::fitInFrame(lt, rb, size);
  EXPECT_TRUE(result);
  EXPECT_EQ(lt.x, 0);
  EXPECT_EQ(lt.y, 0);
  EXPECT_EQ(rb.x, 11);
  EXPECT_EQ(rb.y, 22);
}

TEST(fitInFrame, out_of_range_right_bottom)
{
  cv::Point lt(1, 2);
  cv::Point rb(21, 32);
  cv::Size size(20, 30);

  bool result = autoware::traffic_light::utils::fitInFrame(lt, rb, size);
  EXPECT_TRUE(result);
  EXPECT_EQ(lt.x, 1);
  EXPECT_EQ(lt.y, 2);
  EXPECT_EQ(rb.x, 19);
  EXPECT_EQ(rb.y, 29);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
