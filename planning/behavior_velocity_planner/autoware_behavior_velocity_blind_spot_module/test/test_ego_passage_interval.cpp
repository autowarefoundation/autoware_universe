// Copyright 2026 The Autoware Contributors
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

#include "autoware/behavior_velocity_blind_spot_module/time_to_collision.hpp"
#include "experimental/time_to_collision.hpp"

#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/LineString.h>

#include <cmath>
#include <optional>
#include <utility>
#include <vector>

namespace
{

using autoware::experimental::lanelet2_utils::TurnDirection;

/*
 * A left turn and its mirror image, a right turn, at a junction whose entry line is x = 0. Ego
 * drives along +x on y = 0, turns with radius 10 m and leaves along x = 10. Both virtual lines lie
 * on the turning side of ego, as the module generates them:
 *
 * - virtual_ego_straight_path_after_turning: ego's turning-side edge carried straight on (|y| = 1)
 * - virtual_blind_lane_boundary_after_turning: the outermost blind side lane's turning-side bound
 *   carried straight on (|y| = 6.1, the geometry of the right turn reported in #13509)
 *
 * Ego crosses the first line, then the second one, whichever way it turns, so a right turn must
 * yield exactly the interval of the mirrored left turn.
 */
constexpr double turn_radius = 10.0;
constexpr double ego_straight_offset = 1.0;
constexpr double blind_boundary_offset = 6.1;
constexpr double velocity = 5.0;
constexpr double ds = 0.1;

geometry_msgs::msg::Pose make_pose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.z = std::sin(yaw / 2.0);
  pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

// ego poses (base_link) and the time to reach them at constant velocity
std::vector<std::pair<geometry_msgs::msg::Pose, double>> ego_poses(const TurnDirection direction)
{
  const double sign = (direction == TurnDirection::Left) ? 1.0 : -1.0;
  std::vector<std::pair<geometry_msgs::msg::Pose, double>> poses;
  double s = 0.0;
  for (double x = -15.0; x < 0.0; x += ds, s += ds) {
    poses.emplace_back(make_pose(x, 0.0, 0.0), s / velocity);
  }
  const double arc_step = ds / turn_radius;
  for (double theta = 0.0; theta < M_PI / 2.0; theta += arc_step, s += ds) {
    poses.emplace_back(
      make_pose(
        turn_radius * std::sin(theta), sign * turn_radius * (1.0 - std::cos(theta)), sign * theta),
      s / velocity);
  }
  for (double d = 0.0; d < 20.0; d += ds, s += ds) {
    poses.emplace_back(
      make_pose(turn_radius, sign * (turn_radius + d), sign * M_PI / 2.0), s / velocity);
  }
  return poses;
}

autoware_utils_geometry::LinearRing2d ego_footprint()
{
  // base_link at the rear axle: 3.8 m to the front, 1.0 m to the rear, 1.8 m wide
  autoware_utils_geometry::LinearRing2d footprint;
  footprint.emplace_back(3.8, 0.9);
  footprint.emplace_back(3.8, -0.9);
  footprint.emplace_back(-1.0, -0.9);
  footprint.emplace_back(-1.0, 0.9);
  footprint.emplace_back(3.8, 0.9);
  return footprint;
}

lanelet::ConstLineString3d straight_line_after_entry(
  const TurnDirection direction, const double offset)
{
  const double sign = (direction == TurnDirection::Left) ? 1.0 : -1.0;
  return lanelet::LineString3d{
    lanelet::InvalId,
    {lanelet::Point3d{lanelet::InvalId, 0.0, sign * offset, 0.0},
     lanelet::Point3d{lanelet::InvalId, 30.0, sign * offset, 0.0}}};
}

std::vector<autoware::behavior_velocity_planner::FuturePosition> ego_future_profile(
  const TurnDirection direction)
{
  std::vector<autoware::behavior_velocity_planner::FuturePosition> profile;
  for (const auto & [pose, time] : ego_poses(direction)) {
    autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
    point.point.pose = pose;
    profile.push_back({point, time});
  }
  return profile;
}

std::vector<autoware::behavior_velocity_planner::experimental::FuturePose>
experimental_ego_future_profile(const TurnDirection direction)
{
  std::vector<autoware::behavior_velocity_planner::experimental::FuturePose> profile;
  for (const auto & [pose, time] : ego_poses(direction)) {
    profile.push_back({pose, time});
  }
  return profile;
}

std::optional<std::pair<double, double>> ego_passage_interval(const TurnDirection direction)
{
  return autoware::behavior_velocity_planner::compute_ego_passage_time_interval(
    ego_future_profile(direction), ego_footprint(),
    straight_line_after_entry(direction, ego_straight_offset),
    straight_line_after_entry(direction, blind_boundary_offset));
}

std::optional<std::pair<double, double>> experimental_ego_passage_interval(
  const TurnDirection direction)
{
  const auto interval =
    autoware::behavior_velocity_planner::experimental::compute_ego_passage_time_interval(
      experimental_ego_future_profile(direction), ego_footprint(),
      straight_line_after_entry(direction, ego_straight_offset),
      straight_line_after_entry(direction, blind_boundary_offset));
  if (!interval) {
    return std::nullopt;
  }
  return std::make_pair(interval->start, interval->end);
}

void expect_mirrored_turns_agree(
  const std::optional<std::pair<double, double>> & left,
  const std::optional<std::pair<double, double>> & right)
{
  ASSERT_TRUE(left.has_value());
  ASSERT_TRUE(right.has_value());
  const auto [left_entry, left_exit] = left.value();
  const auto [right_entry, right_exit] = right.value();
  EXPECT_LT(left_entry, left_exit);
  EXPECT_LT(right_entry, right_exit);
  EXPECT_NEAR(right_entry, left_entry, 1e-6);
  EXPECT_NEAR(right_exit, left_exit, 1e-6);
}

}  // namespace

TEST(EgoPassageTimeInterval, rightTurnMirrorsLeftTurn)
{
  expect_mirrored_turns_agree(
    ego_passage_interval(TurnDirection::Left), ego_passage_interval(TurnDirection::Right));
}

TEST(EgoPassageTimeInterval, rightTurnMirrorsLeftTurnExperimental)
{
  expect_mirrored_turns_agree(
    experimental_ego_passage_interval(TurnDirection::Left),
    experimental_ego_passage_interval(TurnDirection::Right));
}
