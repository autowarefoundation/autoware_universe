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

#include <sensor_msgs/msg/camera_info.hpp>

#include <gtest/gtest.h>
#include <math.h>

image_geometry::PinholeCameraModel createPinholeCameraModel()
{
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.width = 1440;
  camera_info.height = 1080;
  image_geometry::PinholeCameraModel pinhole_camera_model;
  pinhole_camera_model.fromCameraInfo(camera_info);
  return pinhole_camera_model;
}

TEST(roundInImageFrame, no_round)
{
  const auto pinhole_camera_model = createPinholeCameraModel();
  cv::Point2d point2d(1.0, 2.0);
  autoware::traffic_light::utils::roundInImageFrame(pinhole_camera_model, point2d);
  EXPECT_EQ(point2d.x, 1.0);
  EXPECT_EQ(point2d.y, 2.0);
}

TEST(roundInImageFrame, out_of_range_upper)
{
  const auto pinhole_camera_model = createPinholeCameraModel();
  cv::Point2d point2d(1500.0, 1100.0);
  autoware::traffic_light::utils::roundInImageFrame(pinhole_camera_model, point2d);
  EXPECT_EQ(point2d.x, 1439.0);
  EXPECT_EQ(point2d.y, 1079.0);
}

TEST(roundInImageFrame, out_of_range_lower)
{
  const auto pinhole_camera_model = createPinholeCameraModel();
  cv::Point2d point2d(-1.5, -2.5);
  autoware::traffic_light::utils::roundInImageFrame(pinhole_camera_model, point2d);
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
