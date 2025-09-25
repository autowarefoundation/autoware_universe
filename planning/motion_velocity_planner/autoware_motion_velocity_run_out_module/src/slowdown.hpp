// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef SLOWDOWN_HPP_
#define SLOWDOWN_HPP_

#include "parameters.hpp"
#include "types.hpp"

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
/// @brief calculate the interpolated point along the trajectory at the given time from start
/// @param trajectory ego trajectory starting from the ego pose (UB if empty)
/// @param time [s] requested time
/// @return trajectory point corresponding to the given time
geometry_msgs::msg::Point interpolated_point_at_time(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory, const double time);

/// @brief get the most recent collision in the history
std::optional<geometry_msgs::msg::Point> get_most_recent_slowdown_point(
  const DecisionHistory & history);

/// @brief calculate the stop for the given decision history
/// @param [inout] history decision history where the current decision may be updated with the
/// calculated stop point
/// @param [in] trajectory ego trajectory starting from the current ego pose
/// @param [inout] unfeasible_stop_deceleration maximum deceleration found that breaks the limit
/// @param [in] params module parameters
/// @return stop point
std::optional<geometry_msgs::msg::Point> calculate_stop_position(
  DecisionHistory & history,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  std::optional<double> & unfeasible_stop_deceleration, const Parameters & params);

/// @brief calculate the slowdown for the given decision history
/// @param [inout] history decision history where the current decision may be updated with the
/// calculated slowdown interval
/// @param [in] trajectory ego trajectory starting from the current ego pose
/// @param [in] current_velocity [m/s] current ego velocity
/// @param [in] params module parameters
/// @return slowdown interval (from point, to point, velocity)
std::optional<SlowdownInterval> calculate_slowdown_interval(
  DecisionHistory & history,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const double current_velocity, const Parameters & params);

/// @brief calculate slowdowns for the given decisions
/// @param [inout] decision_tracker decision history of all objects
/// @param [in] trajectory ego trajectory starting from current pose
/// @param [in] planner_data planner data with deceleration limits
/// @param [in] current_velocity [m/s] current ego velocity
/// @param [inout] unfeasible_stop_deceleration maximum deceleration found that breaks the limit
/// @param [in] params module parameters
/// @return result with the calculated stop point and slowdown intervals and their safety factors
RunOutResult calculate_slowdowns(
  ObjectDecisionsTracker & decision_tracker,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const double current_velocity, std::optional<double> & unfeasible_stop_deceleration,
  const Parameters & params);
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // SLOWDOWN_HPP_
