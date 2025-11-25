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

}  // namespace autoware::diffusion_planner::test
