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

#include "../src/traffic_light_map_based_detector_process.hpp"

#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <gtest/gtest.h>
#include <math.h>

#include <vector>

TEST(roundInImageFrame, no_round)
{
  const uint32_t width = 1440;
  const uint32_t height = 1080;
  cv::Point2d point2d(1.0, 2.0);
  autoware::traffic_light::utils::roundInImageFrame(width, height, point2d);
  EXPECT_EQ(point2d.x, 1.0);
  EXPECT_EQ(point2d.y, 2.0);
}

TEST(roundInImageFrame, out_of_range_bottom_right)
{
  const uint32_t width = 1440;
  const uint32_t height = 1080;
  cv::Point2d point2d(1500.0, 1100.0);
  autoware::traffic_light::utils::roundInImageFrame(width, height, point2d);
  EXPECT_EQ(point2d.x, 1439.0);
  EXPECT_EQ(point2d.y, 1079.0);
}

TEST(roundInImageFrame, out_of_range_top_left)
{
  const uint32_t width = 1440;
  const uint32_t height = 1080;
  cv::Point2d point2d(-1.5, -2.5);
  autoware::traffic_light::utils::roundInImageFrame(width, height, point2d);
  EXPECT_EQ(point2d.x, 0.0);
  EXPECT_EQ(point2d.y, 0.0);
}

TEST(isInDistanceRange, in_range)
{
  const tf2::Vector3 v1(1.0, 1.0, 3.0);
  const tf2::Vector3 v2(4.0, 5.0, 6.0);
  const double max_distance_range = 6.0;
  const bool result = autoware::traffic_light::utils::isInDistanceRange(v1, v2, max_distance_range);
  EXPECT_TRUE(result);
}

TEST(isInDistanceRange, out_of_range)
{
  const tf2::Vector3 v1(1.0, 1.0, 3.0);
  const tf2::Vector3 v2(4.0, 5.0, 6.0);
  const double max_distance_range = 5.0;
  const bool result = autoware::traffic_light::utils::isInDistanceRange(v1, v2, max_distance_range);
  EXPECT_FALSE(result);
}

TEST(isInAngleRange, in_range)
{
  const double tl_yaw = M_PI / 2;
  const double camera_yaw = M_PI;
  const double max_angle_range = M_PI;
  const bool result =
    autoware::traffic_light::utils::isInAngleRange(tl_yaw, camera_yaw, max_angle_range);
  EXPECT_TRUE(result);
}

TEST(isInAngleRange, out_of_range)
{
  const double tl_yaw = M_PI / 2;
  const double camera_yaw = M_PI;
  const double max_angle_range = M_PI / 4;
  bool result = autoware::traffic_light::utils::isInAngleRange(tl_yaw, camera_yaw, max_angle_range);
  EXPECT_FALSE(result);
}

TEST(isInAngleRange, in_range_boundary)
{
  const double tl_yaw = M_PI - M_PI / 16;
  const double camera_yaw = -M_PI + M_PI / 16;
  const double max_angle_range = M_PI / 4;
  bool result = autoware::traffic_light::utils::isInAngleRange(tl_yaw, camera_yaw, max_angle_range);
  EXPECT_TRUE(result);
}

TEST(getVibrationMargin, calculate)
{
  const tf2::Vector3 position(10.0, 20.0, 100.0);
  const double margin_pitch = 0.01745329251;  // 1 degree in radians
  const double margin_yaw = 0.03490658503;    // 2 degree in radians
  const double margin_height = 0.3;
  const double margin_width = 0.4;
  const double margin_depth = 0.5;

  const tf2::Vector3 result = autoware::traffic_light::utils::getVibrationMargin(
    position.z(), margin_pitch, margin_yaw, margin_height, margin_width, margin_depth);

  EXPECT_FLOAT_EQ(result.x(), 1.945240643);
  EXPECT_FLOAT_EQ(result.y(), 1.022653549);
  EXPECT_FLOAT_EQ(result.z(), 0.25);
}

TEST(computeBoundingRoi, select)
{
  const uint32_t width = 1440;
  const uint32_t height = 1080;
  std::vector<tier4_perception_msgs::msg::TrafficLightRoi> rois;
  tier4_perception_msgs::msg::TrafficLightRoi max_roi;
  {
    tier4_perception_msgs::msg::TrafficLightRoi roi;
    roi.roi.x_offset = 0;
    roi.roi.y_offset = 0;
    roi.roi.width = 10;
    roi.roi.height = 20;
    rois.push_back(roi);
  }
  {
    tier4_perception_msgs::msg::TrafficLightRoi roi;
    roi.roi.x_offset = 10;
    roi.roi.y_offset = 20;
    roi.roi.width = 160;
    roi.roi.height = 200;
    rois.push_back(roi);
  }
  {
    tier4_perception_msgs::msg::TrafficLightRoi roi;
    roi.roi.x_offset = 40;
    roi.roi.y_offset = 80;
    roi.roi.width = 30;
    roi.roi.height = 40;
    rois.push_back(roi);
  }

  autoware::traffic_light::utils::computeBoundingRoi(width, height, rois, max_roi);
  EXPECT_EQ(max_roi.roi.x_offset, 0);
  EXPECT_EQ(max_roi.roi.y_offset, 0);
  const uint32_t expected_width = 10 + 160 - 0;
  const uint32_t expected_height = 20 + 200 - 0;
  EXPECT_EQ(max_roi.roi.width, expected_width);
  EXPECT_EQ(max_roi.roi.height, expected_height);
}

TEST(getCameraYaw, calculate)
{
  tf2::Quaternion q;
  q.setRPY(M_PI / 2.0, 0.0, 0.0);
  const tf2::Transform tf_map2camera(q, tf2::Vector3(1.0, 2.0, 3.0));
  const double result = autoware::traffic_light::utils::getCameraYaw(tf_map2camera);
  const double expected_yaw = -M_PI / 2.0;
  EXPECT_FLOAT_EQ(result, expected_yaw);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
