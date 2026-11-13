// Copyright 2022 TIER IV, Inc.
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

#include "../src/forward_projection.hpp"
#include "../src/types.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <boost/geometry/io/wkt/write.hpp>

#include <geometry_msgs/msg/point.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <iostream>

constexpr auto EPS = 1e-15;
constexpr auto EPS_APPROX = 1e-3;

TEST(TestForwardProjection, forwardSimulatedSegment)
{
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::forwardSimulatedSegment;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::ProjectionParameters;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::segment_t;

  geometry_msgs::msg::Point point;
  point.x = 0.0;
  point.y = 0.0;
  ProjectionParameters params;
  params.model = ProjectionParameters::PARTICLE;

  const auto check_vector = [&](const auto expected_vector_length) {
    params.heading = 0.0;
    auto vector = forwardSimulatedSegment(point, params);
    EXPECT_DOUBLE_EQ(vector.first.x(), point.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), point.y);
    EXPECT_DOUBLE_EQ(vector.second.x(), expected_vector_length);
    EXPECT_DOUBLE_EQ(vector.second.y(), 0.0);
    params.heading = M_PI_2;
    vector = forwardSimulatedSegment(point, params);
    EXPECT_DOUBLE_EQ(vector.first.x(), point.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), point.y);
    EXPECT_NEAR(vector.second.x(), 0.0, 1e-9);
    EXPECT_DOUBLE_EQ(vector.second.y(), expected_vector_length);
    params.heading = M_PI_4;
    vector = forwardSimulatedSegment(point, params);
    EXPECT_DOUBLE_EQ(vector.first.x(), point.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), point.y);
    EXPECT_DOUBLE_EQ(vector.second.x(), std::sqrt(0.5) * expected_vector_length);
    EXPECT_DOUBLE_EQ(vector.second.y(), std::sqrt(0.5) * expected_vector_length);
    params.heading = -M_PI_2;
    vector = forwardSimulatedSegment(point, params);
    EXPECT_DOUBLE_EQ(vector.first.x(), point.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), point.y);
    EXPECT_NEAR(vector.second.x(), 0.0, 1e-9);
    EXPECT_DOUBLE_EQ(vector.second.y(), -expected_vector_length);
  };

  // 0 velocity: whatever the duration the vector length is always = to extra_dist
  params.velocity = 0.0;

  params.duration = 0.0;
  params.extra_length = 0.0;
  check_vector(params.extra_length);

  params.duration = 5.0;
  params.extra_length = 2.0;
  check_vector(params.extra_length);

  params.duration = -5.0;
  params.extra_length = 3.5;
  check_vector(params.extra_length);

  // set non-zero velocities
  params.velocity = 1.0;

  params.duration = 1.0;
  params.extra_length = 0.0;
  check_vector(1.0 + params.extra_length);

  params.duration = 5.0;
  params.extra_length = 2.0;
  check_vector(5.0 + params.extra_length);

  params.duration = -5.0;
  params.extra_length = 3.5;
  check_vector(-5.0 + params.extra_length);
}

TEST(TestForwardProjection, bicycleProjectionLineStraight)
{
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::bicycleProjectionLine;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::ProjectionParameters;

  geometry_msgs::msg::Point origin;
  origin.x = 1.0;
  origin.y = 2.0;
  ProjectionParameters params;
  params.model = ProjectionParameters::BICYCLE;
  params.wheel_base = 2.79;
  params.velocity = 5.0;
  params.duration = 2.0;
  params.extra_length = 4.0;
  params.heading = M_PI_2;
  params.points_per_projection = 5;

  const auto line = bicycleProjectionLine(origin, params, 0.0);
  ASSERT_EQ(line.size(), 5ul);
  EXPECT_DOUBLE_EQ(line[0].x(), origin.x);
  EXPECT_DOUBLE_EQ(line[0].y(), origin.y);
  for (size_t i = 1; i < line.size(); ++i) {
    const auto t = static_cast<double>(i) * params.duration / 4.0;
    EXPECT_NEAR(line[i].x(), origin.x, EPS_APPROX) << "index: " << i;
    EXPECT_NEAR(line[i].y(), origin.y + params.velocity * t + params.extra_length, EPS_APPROX)
      << "index: " << i;
  }
}

// The projected points must follow the arc of the turning circle of the rear axle center,
// shifted by the extra length along the heading reached at that point.
TEST(TestForwardProjection, bicycleProjectionLineArc)
{
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::bicycleProjectionLine;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::ProjectionParameters;

  geometry_msgs::msg::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  ProjectionParameters params;
  params.model = ProjectionParameters::BICYCLE;
  params.wheel_base = 2.79;
  params.velocity = 5.0;
  params.duration = 3.0;
  params.extra_length = 4.0;
  params.heading = 0.0;
  params.points_per_projection = 7;

  for (const auto steering_angle : {0.2, -0.2}) {
    const auto line = bicycleProjectionLine(origin, params, steering_angle);
    ASSERT_EQ(line.size(), 7ul);
    // turning circle centered at (0, radius), radius is negative when turning right
    const auto radius = params.wheel_base / std::tan(steering_angle);
    for (size_t i = 1; i < line.size(); ++i) {
      const auto t = static_cast<double>(i) * params.duration / 6.0;
      const auto heading = params.velocity * t / radius;
      const auto expected_x = radius * std::sin(heading) + params.extra_length * std::cos(heading);
      const auto expected_y =
        radius * (1.0 - std::cos(heading)) + params.extra_length * std::sin(heading);
      EXPECT_NEAR(line[i].x(), expected_x, EPS_APPROX)
        << "steering: " << steering_angle << ", index: " << i;
      EXPECT_NEAR(line[i].y(), expected_y, EPS_APPROX)
        << "steering: " << steering_angle << ", index: " << i;
    }
  }
}

const auto point_in_polygon = [](const auto x, const auto y, const auto & polygon) {
  return std::find_if(polygon.outer().begin(), polygon.outer().end(), [=](const auto & pt) {
           return pt.x() == x && pt.y() == y;
         }) != polygon.outer().end();
};

TEST(TestForwardProjection, generateFootprint)
{
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::generateFootprint;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::linestring_t;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::segment_t;

  auto footprint = generateFootprint(linestring_t{{0.0, 0.0}, {1.0, 0.0}}, 1.0);
  EXPECT_TRUE(point_in_polygon(0.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(0.0, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, -1.0, footprint));
  footprint = generateFootprint(segment_t{{0.0, 0.0}, {1.0, 0.0}}, 1.0);
  EXPECT_TRUE(point_in_polygon(0.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(0.0, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, -1.0, footprint));

  footprint = generateFootprint(linestring_t{{0.0, 0.0}, {0.0, -1.0}}, 0.5);
  EXPECT_TRUE(point_in_polygon(0.5, 0.0, footprint));
  EXPECT_TRUE(point_in_polygon(0.5, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(-0.5, 0.0, footprint));
  EXPECT_TRUE(point_in_polygon(-0.5, -1.0, footprint));
  footprint = generateFootprint(segment_t{{0.0, 0.0}, {0.0, -1.0}}, 0.5);
  EXPECT_TRUE(point_in_polygon(0.5, 0.0, footprint));
  EXPECT_TRUE(point_in_polygon(0.5, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(-0.5, 0.0, footprint));
  EXPECT_TRUE(point_in_polygon(-0.5, -1.0, footprint));

  footprint = generateFootprint(linestring_t{{-2.5, 5.0}, {2.5, 0.0}}, std::sqrt(2));
  EXPECT_TRUE(point_in_polygon(3.5, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.5, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(-3.5, 4.0, footprint));
  EXPECT_TRUE(point_in_polygon(-1.5, 6.0, footprint));
  footprint = generateFootprint(segment_t{{-2.5, 5.0}, {2.5, 0.0}}, std::sqrt(2));
  EXPECT_TRUE(point_in_polygon(3.5, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.5, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(-3.5, 4.0, footprint));
  EXPECT_TRUE(point_in_polygon(-1.5, 6.0, footprint));
}

TEST(TestForwardProjection, generateFootprintMultiLinestrings)
{
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::generateFootprint;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::linestring_t;
  using autoware::motion_velocity_planner::obstacle_velocity_limiter::multi_linestring_t;

  auto footprint = generateFootprint(
    multi_linestring_t{
      linestring_t{{0.0, 0.0}, {0.0, 1.0}}, linestring_t{{0.0, 0.0}, {0.8, 0.8}},
      linestring_t{{0.0, 0.0}, {1.0, 0.0}}},
    0.5);
  std::cout << boost::geometry::wkt(footprint) << std::endl;
  /*
  EXPECT_TRUE(point_in_polygon(-0.5, 0.0, footprint));
  EXPECT_TRUE(point_in_polygon(-0.5, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(0.5, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, -1.0, footprint));
  */
}
