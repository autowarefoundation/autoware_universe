// Copyright 2026 TIER IV, Inc. All rights reserved.
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

#include "../src/collision.hpp"
#include "../src/parameters.hpp"
#include "../src/types.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
namespace
{
Parameters make_params()
{
  Parameters params;
  params.collision_time_margin = 1.0;
  params.collision_same_direction_angle_threshold = 0.785;      // [rad] 45 degrees
  params.collision_opposite_direction_angle_threshold = 0.785;  // [rad] 45 degrees
  params.ignore_collision_conditions.if_ego_arrives_first.enable = false;
  params.ignore_collision_conditions.if_ego_arrives_first.margin.ego_enter_times = {0.0, 10.0};
  params.ignore_collision_conditions.if_ego_arrives_first.margin.time_margins = {0.0, 0.0};
  params.ignore_collision_conditions.if_ego_arrives_first.max_overlap_duration = 0.0;
  params.ignore_collision_conditions.if_ego_arrives_first_and_cannot_stop.enable = false;
  params.ignore_collision_conditions.if_ego_arrives_first_and_cannot_stop
    .calculated_stop_time_limit = 0.0;
  return params;
}

/// @brief intersection of an object moving in the opposite direction of ego
FootprintIntersection make_intersection(
  const double ego_time, const double object_time, const double arc_length,
  const IntersectionPosition position = front_left)
{
  FootprintIntersection fi;
  fi.ego_time = ego_time;
  fi.object_time = object_time;
  fi.arc_length = arc_length;
  fi.position = position;
  fi.yaw_diff = M_PI;  // ego and the object move in opposite directions
  fi.ego_vel = 5.0;
  fi.vel_diff = 10.0;
  return fi;
}
}  // namespace

TEST(TestCollisions, degenerate_overlap_gives_finite_collision_time)
{
  const auto params = make_params();
  const auto intersection = make_intersection(11.1, 12.0, 33.0);
  const TimeOverlapInterval ego(11.1, 11.1, intersection, intersection);
  const TimeOverlapInterval object(12.0, 12.0, intersection, intersection);
  Collision c(ego, object);

  calculate_overlapping_collision(c, ego, object, params);

  EXPECT_EQ(c.type, collision);
  EXPECT_TRUE(std::isfinite(c.ego_collision_time)) << "collision time = " << c.ego_collision_time;
  EXPECT_DOUBLE_EQ(c.ego_collision_time, 11.1);
}

TEST(TestCollisions, zero_overlap_length_gives_finite_collision_time)
{
  const auto params = make_params();
  const TimeOverlapInterval ego(
    11.0, 12.0, make_intersection(11.0, 10.0, 30.0), make_intersection(12.0, 12.0, 30.0));
  const TimeOverlapInterval object(
    10.0, 12.0, make_intersection(11.0, 10.0, 30.0), make_intersection(12.0, 12.0, 30.0));
  Collision c(ego, object);

  calculate_overlapping_collision(c, ego, object, params);

  EXPECT_EQ(c.type, collision);
  EXPECT_TRUE(std::isfinite(c.ego_collision_time)) << "collision time = " << c.ego_collision_time;
  EXPECT_DOUBLE_EQ(c.ego_collision_time, 11.0);
}

TEST(TestCollisions, zero_ego_duration_gives_finite_collision_time)
{
  const auto params = make_params();
  const TimeOverlapInterval ego(
    11.0, 11.0, make_intersection(11.0, 10.0, 30.0), make_intersection(11.0, 12.0, 40.0));
  const TimeOverlapInterval object(
    10.0, 12.0, make_intersection(11.0, 10.0, 30.0), make_intersection(11.0, 12.0, 40.0));
  Collision c(ego, object);

  calculate_overlapping_collision(c, ego, object, params);

  EXPECT_EQ(c.type, collision);
  EXPECT_TRUE(std::isfinite(c.ego_collision_time)) << "collision time = " << c.ego_collision_time;
  EXPECT_DOUBLE_EQ(c.ego_collision_time, 11.0);
}

TEST(TestCollisions, zero_object_duration_gives_finite_collision_time)
{
  const auto params = make_params();
  const TimeOverlapInterval ego(
    11.0, 12.0, make_intersection(11.0, 10.0, 30.0), make_intersection(12.0, 10.0, 40.0));
  const TimeOverlapInterval object(
    10.0, 10.0, make_intersection(11.0, 10.0, 30.0), make_intersection(12.0, 10.0, 40.0));
  Collision c(ego, object);

  calculate_overlapping_collision(c, ego, object, params);

  EXPECT_EQ(c.type, collision);
  EXPECT_TRUE(std::isfinite(c.ego_collision_time)) << "collision time = " << c.ego_collision_time;
  EXPECT_DOUBLE_EQ(c.ego_collision_time, 11.0);
}

TEST(TestCollisions, regular_opposite_direction_overlap_is_unchanged)
{
  const auto params = make_params();
  // ego covers the 10m overlap in 1s, the object covers it in 2s
  const TimeOverlapInterval ego(
    11.0, 12.0, make_intersection(11.0, 10.0, 30.0), make_intersection(12.0, 12.0, 40.0));
  const TimeOverlapInterval object(
    10.0, 12.0, make_intersection(11.0, 10.0, 30.0), make_intersection(12.0, 12.0, 40.0));
  Collision c(ego, object);

  calculate_overlapping_collision(c, ego, object, params);

  EXPECT_EQ(c.type, collision);
  // lon_buffer = min(10, 4) = 4, ego_vel = 10, obj_vel = 5 -> 11.0 + (10 - 4) / 15 = 11.4
  EXPECT_DOUBLE_EQ(c.ego_collision_time, 11.4);
}

TEST(TestCollisions, unclosed_trailing_overlap_gives_finite_collision_time)
{
  const auto params = make_params();
  std::vector<FootprintIntersection> intersections;
  // the object enters and exits the rear part of the footprint, closing a first overlap
  intersections.push_back(make_intersection(9.0, 0.5, 20.0, rear_left));
  intersections.push_back(make_intersection(10.0, 5.0, 25.0, rear_right));
  // this one opens a new overlap and the intersection list ends
  intersections.push_back(make_intersection(11.1, 12.0, 33.0, front_left));

  const auto intervals = calculate_overlap_intervals(intersections);

  ASSERT_EQ(intervals.size(), 2UL);
  const auto & degenerate = intervals[1];
  EXPECT_DOUBLE_EQ(degenerate.ego.from, degenerate.ego.to);

  const auto c = calculate_collision(degenerate.ego, degenerate.object, params);

  EXPECT_EQ(c.type, collision);
  EXPECT_TRUE(std::isfinite(c.ego_collision_time)) << "collision time = " << c.ego_collision_time;
  EXPECT_DOUBLE_EQ(c.ego_collision_time, 11.1);
}
}  // namespace autoware::motion_velocity_planner::run_out
