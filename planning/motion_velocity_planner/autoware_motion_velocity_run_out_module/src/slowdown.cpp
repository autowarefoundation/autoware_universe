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

#include "slowdown.hpp"

#include "parameters.hpp"
#include "types.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <algorithm>
#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
geometry_msgs::msg::Point interpolated_point_at_time(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory, const double time)
{
  const auto & prev_it = std::find_if(
    trajectory.begin(), trajectory.end(),
    [&](const autoware_planning_msgs::msg::TrajectoryPoint & t) {
      return rclcpp::Duration(t.time_from_start).seconds() >= time;
    });
  const auto prev_time = rclcpp::Duration(prev_it->time_from_start).seconds();
  if (prev_time == time) {
    return prev_it->pose.position;
  }
  const auto next_time = rclcpp::Duration(std::next(prev_it)->time_from_start).seconds();
  if (next_time == prev_time) {
    return std::next(prev_it)->pose.position;
  }
  const auto t_delta = next_time - prev_time;
  const auto t_diff = time - prev_time;
  const auto ratio = t_diff / t_delta;
  return universe_utils::calcInterpolatedPoint(
    prev_it->pose.position, std::next(prev_it)->pose.position, ratio);
}

std::optional<geometry_msgs::msg::Point> get_most_recent_slowdown_point(
  const DecisionHistory & history)
{
  for (auto it = history.decisions.rbegin(); it != history.decisions.rend(); ++it) {
    if (it->stop_point) {
      return it->stop_point;
    }
    if (it->slowdown_interval) {
      return it->slowdown_interval->to;
    }
  }
  return std::nullopt;
}

std::optional<geometry_msgs::msg::Point> calculate_stop_position(
  DecisionHistory & history,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  double current_velocity, std::optional<double> & unfeasible_stop_deceleration,
  const Parameters & params)
{
  const auto & update_unfeasible_stop = [&](const auto stop_distance) {
    const auto stop_decel = (current_velocity * current_velocity) / (2 * stop_distance);
    if (
      stop_decel > params.stop_deceleration_limit &&
      stop_decel > unfeasible_stop_deceleration.value_or(0.0)) {
      unfeasible_stop_deceleration = stop_decel;
    }
  };
  const auto max_time = rclcpp::Duration(trajectory.back().time_from_start).seconds();
  auto & current_decision = history.decisions.back();
  if (current_decision.type == stop) {
    if (!current_decision.collision || current_decision.collision->type != collision) {
      const auto most_recent_slowdown_point = get_most_recent_slowdown_point(history);
      if (!most_recent_slowdown_point) {
        return std::nullopt;
      }
      const auto stop_point_length =
        motion_utils::calcSignedArcLength(trajectory, 0, *most_recent_slowdown_point);
      current_decision.stop_point =
        motion_utils::calcInterpolatedPose(trajectory, stop_point_length).position;
      update_unfeasible_stop(stop_point_length);
      return current_decision.stop_point;
    }
    const auto t_coll = current_decision.collision->ego_collision_time;
    if (t_coll > max_time) {
      return std::nullopt;
    }
    const auto t_stop = std::max(0.0, t_coll);
    const auto base_link_point = interpolated_point_at_time(trajectory, t_stop);
    const auto stop_point_length =
      motion_utils::calcSignedArcLength(trajectory, 0, base_link_point) -
      params.stop_distance_buffer;
    current_decision.stop_point =
      motion_utils::calcInterpolatedPose(trajectory, stop_point_length).position;
    update_unfeasible_stop(stop_point_length);
  }
  return current_decision.stop_point;
}

std::optional<SlowdownInterval> calculate_slowdown_interval(
  DecisionHistory & history,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const double current_velocity, const Parameters & params)
{
  auto & current_decision = history.decisions.back();
  if (current_decision.type != slowdown) {
    return std::nullopt;
  }
  const auto t_collision = current_decision.collision->ego_collision_time;
  const auto p_collision = [&]() {
    if (current_decision.collision && current_decision.collision->type == collision) {
      return std::make_optional(interpolated_point_at_time(trajectory, t_collision));
    }
    return get_most_recent_slowdown_point(history);
  }();
  if (!p_collision) {
    return std::nullopt;
  }
  const auto min_slow_arc_length = current_velocity * 0.1;
  auto from_arc_length = std::max(
    min_slow_arc_length, motion_utils::calcSignedArcLength(trajectory, 0, *p_collision) -
                           params.slowdown_distance_buffer);
  const auto p_slowdown = motion_utils::calcInterpolatedPose(trajectory, from_arc_length).position;
  // safe velocity that guarantees we can smoothly stop before the collision
  const auto safe_velocity =
    std::sqrt(2.0 * params.stop_deceleration_limit * params.slowdown_distance_buffer);
  // velocity limit we can reach by applying minimum deceleration until the slowdown point
  const auto smooth_velocity = std::sqrt(
    current_velocity * current_velocity -
    2.0 * params.slowdown_deceleration_limit * from_arc_length);
  const SlowdownInterval interval{
    p_slowdown, *p_collision, std::max({0.0, safe_velocity, smooth_velocity})};
  current_decision.slowdown_interval = interval;
  return interval;
}

VelocityPlanningResult calculate_slowdowns(
  ObjectDecisionsTracker & decision_tracker,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const double current_velocity, std::optional<double> & unfeasible_stop_deceleration,
  const Parameters & params)
{
  VelocityPlanningResult result;
  for (auto & [object, history] : decision_tracker.history_per_object) {
    const auto stop_position = calculate_stop_position(
      history, trajectory, current_velocity, unfeasible_stop_deceleration, params);
    if (stop_position) {
      result.stop_points.push_back(*stop_position);
    }
    const auto slowdown_interval =
      calculate_slowdown_interval(history, trajectory, current_velocity, params);
    if (slowdown_interval) {
      result.slowdown_intervals.push_back(*slowdown_interval);
    }
  }
  return result;
}
}  // namespace autoware::motion_velocity_planner::run_out
