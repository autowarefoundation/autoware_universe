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

#include "../src/traffic_light_fine_detector_utils.hpp"

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
  obj.score = 1.0;

  float iou = autoware::traffic_light::utils::calWeightedIou(roi, obj);
  EXPECT_FLOAT_EQ(iou, 0.14285714285);
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
  obj.score = 1.0;

  float iou = autoware::traffic_light::utils::calWeightedIou(roi, obj);
  EXPECT_FLOAT_EQ(iou, 0.25);
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
  obj.score = 1.0;

  float iou = autoware::traffic_light::utils::calWeightedIou(roi, obj);
  EXPECT_FLOAT_EQ(iou, 0.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
