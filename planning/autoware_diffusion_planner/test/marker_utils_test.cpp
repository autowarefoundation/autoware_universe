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

#include "autoware/diffusion_planner/utils/marker_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::test
{

using autoware::diffusion_planner::utils::create_lane_marker;
using autoware::diffusion_planner::utils::get_traffic_light_color;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::MarkerArray;

TEST(MarkerUtilsTest, GetTrafficLightColorGreen)
{
  ColorRGBA orig;
  orig.r = 0.5f;
  orig.g = 0.5f;
  orig.b = 0.5f;
  orig.a = 1.0f;
  ColorRGBA c = get_traffic_light_color(1.0f, 0.0f, 0.0f, orig);
  EXPECT_FLOAT_EQ(c.g, 1.0f);
  EXPECT_FLOAT_EQ(c.r, 0.0f);
  EXPECT_FLOAT_EQ(c.b, 0.0f);
  EXPECT_FLOAT_EQ(c.a, 0.8f);
}

TEST(MarkerUtilsTest, GetTrafficLightColorYellow)
{
  ColorRGBA orig;
  orig.r = 0.1f;
  orig.g = 0.2f;
  orig.b = 0.3f;
  orig.a = 0.4f;
  ColorRGBA c = get_traffic_light_color(0.0f, 1.0f, 0.0f, orig);
  EXPECT_FLOAT_EQ(c.g, 1.0f);
  EXPECT_FLOAT_EQ(c.r, 1.0f);
  EXPECT_FLOAT_EQ(c.b, 0.0f);
  EXPECT_FLOAT_EQ(c.a, 0.8f);
}

TEST(MarkerUtilsTest, GetTrafficLightColorRed)
{
  ColorRGBA orig;
  orig.r = 0.1f;
  orig.g = 0.2f;
  orig.b = 0.3f;
  orig.a = 0.4f;
  ColorRGBA c = get_traffic_light_color(0.0f, 0.0f, 1.0f, orig);
  EXPECT_FLOAT_EQ(c.g, 0.0f);
  EXPECT_FLOAT_EQ(c.r, 1.0f);
  EXPECT_FLOAT_EQ(c.b, 0.0f);
  EXPECT_FLOAT_EQ(c.a, 0.8f);
}

TEST(MarkerUtilsTest, GetTrafficLightColorFallback)
{
  ColorRGBA orig;
  orig.r = 0.1f;
  orig.g = 0.2f;
  orig.b = 0.3f;
  orig.a = 0.4f;
  ColorRGBA c = get_traffic_light_color(0.0f, 0.0f, 0.0f, orig);
  EXPECT_FLOAT_EQ(c.g, orig.g);
  EXPECT_FLOAT_EQ(c.r, orig.r);
  EXPECT_FLOAT_EQ(c.b, orig.b);
  EXPECT_FLOAT_EQ(c.a, orig.a);
}

TEST(MarkerUtilsTest, CreateLaneMarkerBasic)
{
  // Lane vector: 1 segment, 2 points, 8 dims (X, Y, LB_X, LB_Y, RB_X, RB_Y, ...), minimal
  std::vector<float> lane_vector = {1.0f, 2.0f, 0.5f, 0.5f, -0.5f, -0.5f, 0.0f, 0.0f,
                                    2.0f, 3.0f, 0.5f, 0.5f, -0.5f, -0.5f, 0.0f, 0.0f};
  std::vector<int64_t> shape = {1, 1, 2, 8};  // batch, ?, points, dims
  rclcpp::Time stamp(123456, 789, RCL_ROS_TIME);
  rclcpp::Duration lifetime(1, 0);
  Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
  auto marker_array = create_lane_marker(identity, lane_vector, shape, stamp, lifetime);

  // Should create at least 1 marker for the centerline, and possibly for bounds/spheres
  EXPECT_GE(marker_array.markers.size(), 1u);

  for (const auto & marker : marker_array.markers) {
    EXPECT_EQ(marker.header.stamp, stamp);
    EXPECT_EQ(marker.header.frame_id, "base_link");
    EXPECT_GT(marker.points.size(), 0u);
    EXPECT_GT(marker.color.a, 0.0f);
  }
}

TEST(MarkerUtilsTest, CreateLaneMarkerTrafficLightColor)
{
  // Lane vector with green light for first point
  std::vector<float> lane_vector = {1.0f, 2.0f, 0.5f, 0.5f, -0.5f, -0.5f, 1.0f, 0.0f,  // green
                                    2.0f, 3.0f, 0.5f, 0.5f, -0.5f, -0.5f, 0.0f, 0.0f};
  std::vector<int64_t> shape = {1, 1, 2, 8};
  rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
  rclcpp::Duration lifetime(1, 0);
  Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
  auto marker_array = create_lane_marker(
    identity, lane_vector, shape, stamp, lifetime, {0.0f, 1.0f, 0.0f, 0.8f}, "base_link", true);

  // The first marker should have green color if set_traffic_light_color is true
  bool found_green = false;
  for (const auto & marker : marker_array.markers) {
    if (marker.color.g == 1.0f && marker.color.r == 0.0f && marker.color.b == 0.0f) {
      found_green = true;
      break;
    }
  }
  EXPECT_TRUE(found_green);
}

}  // namespace autoware::diffusion_planner::test
