// Copyright 2025 Tier IV, Inc.
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

#ifndef EXPERIMENTAL__TIME_TO_COLLISION_HPP_
#define EXPERIMENTAL__TIME_TO_COLLISION_HPP_

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware/trajectory/path_point_with_lane_id.hpp>

#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using Trajectory = autoware::experimental::trajectory::Trajectory<PathPointWithLaneId>;

struct FuturePose
{
  const geometry_msgs::msg::Pose pose;
  const double duration;
};

struct TimeInterval
{
  double start;
  double end;
};

/**
 * @brief calculate ego vehicle's future position & duration from current position
 */
std::vector<FuturePose> calculate_future_profile(
  const Trajectory & path, const double minimum_default_velocity, const double time_to_restart,
  const PlannerData & planner_data, const lanelet::Id lane_id);

/**
 * @brief compute the time interval for ego to pass from `entry_line` to `exit_line`
 */
std::optional<TimeInterval> compute_passage_time_interval(
  const std::vector<FuturePose> & future_profile,
  const autoware_utils_geometry::LinearRing2d & footprint,
  const lanelet::ConstLineString3d & entry_line, const lanelet::ConstLineString3d & exit_line);

/**
 * @brief compute the time interval in which ego passes the conflict area, entering it at
 * `virtual_ego_straight_path_after_turning` and leaving it at
 * `virtual_blind_lane_boundary_after_turning`
 * @note both lines are generated on the turning side of the ego path (the turning-side edge of ego
 * carried straight on, and the turning-side bound of the outermost blind side lane carried
 * straight on), so ego crosses `virtual_ego_straight_path_after_turning` first for both left and
 * right turns
 */
std::optional<TimeInterval> compute_ego_passage_time_interval(
  const std::vector<FuturePose> & ego_future_profile,
  const autoware_utils_geometry::LinearRing2d & footprint,
  const lanelet::ConstLineString3d & virtual_ego_straight_path_after_turning,
  const lanelet::ConstLineString3d & virtual_blind_lane_boundary_after_turning);

/**
 * @brief compute the time interval for `object` to pass from `line1` (or `entry_line` instead) to
 * `line2` along every predicted path considering footprint
 */
std::vector<std::pair<TimeInterval, autoware_perception_msgs::msg::PredictedPath>>
compute_passage_time_intervals(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const lanelet::ConstLineString3d & line1, const lanelet::ConstLineString3d & entry_line,
  const lanelet::ConstLineString3d & line2);

/**
 * @brief return the most critical time for collision if collision is detected
 */
std::optional<double> get_unsafe_time_if_critical(
  const TimeInterval & ego_passage_interval, const TimeInterval & object_passage_interval,
  const double ttc_start_margin, const double ttc_end_margin);

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__TIME_TO_COLLISION_HPP_
