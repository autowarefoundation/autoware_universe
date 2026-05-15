// Copyright 2026 TIER IV, Inc.
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

// NOTE: This file implements an ad-hoc pseudo emergency stop fallback used for evaluation only.
// It is intended to be removed once the proper MRM is implemented.

#include "autoware/trajectory_validator/pseudo_emergency_stop_handler.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace autoware::trajectory_validator
{

PseudoEmergencyStopHandler::PseudoEmergencyStopHandler(rclcpp::Node & node)
: logger_(node.get_logger()),
  clock_(node.get_clock()),
  planning_factor_interface_(
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "pseudo_emergency_stop"))
{
}

void PseudoEmergencyStopHandler::handle(
  const CandidateTrajectories & input_trajectories, CandidateTrajectories & filtered_trajectories,
  const std::vector<EvaluationTable> & evaluation_tables, const FilterContext & context,
  const validator::Params & params)
{
  const bool pseudo_emergency_stop_triggered =
    is_pseudo_emergency_stop_triggered(evaluation_tables);
  update_pseudo_emergency_stop_state(
    pseudo_emergency_stop_triggered, context.odometry->twist.twist.linear.x, params);
  // Refresh the cached fallback shape from the latest clean MinimumRuleBasedPlanner trajectory.
  cache_fallback_trajectory(input_trajectories, evaluation_tables);
  if (pseudo_emergency_stop_active_) {
    apply_pseudo_emergency_stop_fallback(filtered_trajectories, context, params);
  }
  planning_factor_interface_->publish();
}

bool PseudoEmergencyStopHandler::is_pseudo_emergency_stop_triggered(
  const std::vector<EvaluationTable> & evaluation_tables) const
{
  return std::any_of(
    evaluation_tables.cbegin(), evaluation_tables.cend(),
    [&](const EvaluationTable & table) { return !table.all_feasible(); });
}

void PseudoEmergencyStopHandler::update_pseudo_emergency_stop_state(
  const bool triggered, const double ego_velocity_mps, const validator::Params & params)
{
  if (triggered) {
    pseudo_emergency_stop_active_ = true;
    return;
  }
  if (params.pseudo_emergency_stop.recovery_policy == "immediate") {
    // Release the emergency stop immediately when the trigger condition disappears.
    pseudo_emergency_stop_active_ = false;
    return;
  }
  if (
    pseudo_emergency_stop_active_ &&
    params.pseudo_emergency_stop.recovery_policy == "until_stopped" &&
    std::abs(ego_velocity_mps) < params.pseudo_emergency_stop.recovery_velocity_threshold_mps) {
    // Ego has come to a stop, release the latched emergency-stop state.
    pseudo_emergency_stop_active_ = false;
  }
}

void PseudoEmergencyStopHandler::cache_fallback_trajectory(
  const CandidateTrajectories & input_trajectories,
  const std::vector<EvaluationTable> & evaluation_tables)
{
  constexpr const char * fallback_generator_name = "MinimumRuleBasedPlanner";

  // Find the generator_id whose name matches the fallback generator name.
  const auto generator_info_it = std::find_if(
    input_trajectories.generator_info.cbegin(), input_trajectories.generator_info.cend(),
    [&](const auto & info) { return info.generator_name.data == fallback_generator_name; });
  if (generator_info_it == input_trajectories.generator_info.cend()) {
    return;
  }
  const auto & fallback_generator_id = generator_info_it->generator_id;
  const auto fallback_it = std::find_if(
    input_trajectories.candidate_trajectories.cbegin(),
    input_trajectories.candidate_trajectories.cend(),
    [&](const autoware_internal_planning_msgs::msg::CandidateTrajectory & trajectory) {
      return trajectory.generator_id.uuid == fallback_generator_id.uuid;
    });
  if (fallback_it == input_trajectories.candidate_trajectories.cend()) {
    return;
  }

  // Only cache when no trigger filter has flagged this fallback trajectory as infeasible.
  const auto generator_id_str = autoware_utils_uuid::to_hex_string(fallback_it->generator_id);
  const auto table_it = std::find_if(
    evaluation_tables.cbegin(), evaluation_tables.cend(),
    [&](const EvaluationTable & table) { return table.generator_id == generator_id_str; });
  if (table_it != evaluation_tables.cend() && !table_it->all_feasible()) {
    return;
  }

  cached_fallback_trajectory_ = *fallback_it;
}

void PseudoEmergencyStopHandler::apply_pseudo_emergency_stop_fallback(
  CandidateTrajectories & filtered_trajectories, const FilterContext & context,
  const validator::Params & params)
{
  if (!cached_fallback_trajectory_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "Emergency stop active but no cached fallback trajectory is available yet.");
    return;
  }

  auto stopping_trajectory = *cached_fallback_trajectory_;
  auto & points = stopping_trajectory.points;
  if (points.size() < 2) {
    return;
  }

  const double min_decel = params.pseudo_emergency_stop.deceleration_mps2;
  const double min_jerk = params.pseudo_emergency_stop.jerk_mps3;

  // Drop points behind ego so that the resulting trajectory starts at the nearest point to the
  // current ego position.
  const size_t ego_nearest_idx =
    autoware::motion_utils::findNearestIndex(points, context.odometry->pose.pose.position);
  points.erase(points.begin(), points.begin() + ego_nearest_idx);
  if (points.size() < 2) {
    return;
  }

  // Overwrite the velocity/acceleration profile in place with a jerk/decel-limited stopping
  // profile starting from the current ego state.
  double v = std::max(0.0, context.odometry->twist.twist.linear.x);
  double a = std::min(0.0, context.acceleration->accel.accel.linear.x);
  points.front().longitudinal_velocity_mps = v;
  points.front().acceleration_mps2 = a;
  for (size_t i = 1; i < points.size(); ++i) {
    const double ds = autoware_utils_geometry::calc_distance2d(points[i - 1].pose, points[i].pose);
    if (v <= 1e-3) {
      v = 0.0;
      a = 0.0;
    } else {
      const double dt = ds / v;
      a = std::max(min_decel, a + min_jerk * dt);
      v = std::max(0.0, v + a * dt);
    }
    points[i].longitudinal_velocity_mps = v;
    points[i].acceleration_mps2 = a;
  }

  autoware::motion_utils::calculate_time_from_start(
    stopping_trajectory.points, context.odometry->pose.pose.position);
  filtered_trajectories.candidate_trajectories.clear();
  filtered_trajectories.candidate_trajectories.push_back(stopping_trajectory);

  const auto stop_distance = autoware::motion_utils::calculate_stop_distance(
    context.odometry->twist.twist.linear.x, context.acceleration->accel.accel.linear.x, min_decel,
    min_jerk);
  const auto stop_pose_opt = stop_distance ? autoware::motion_utils::calcLongitudinalOffsetPose(
                                               stopping_trajectory.points, 0, *stop_distance)
                                           : std::nullopt;
  const auto & stop_pose = stop_pose_opt ? *stop_pose_opt : stopping_trajectory.points.back().pose;
  planning_factor_interface_->add(
    stopping_trajectory.points, context.odometry->pose.pose, stop_pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/,
    0.0 /*velocity*/, 0.0 /*shift_length*/);

  RCLCPP_WARN_THROTTLE(
    logger_, *clock_, 1000,
    "Emergency-stop trigger filter fired; falling back to cached MinimumRuleBasedPlanner "
    "trajectory with emergency stop.");
}

}  // namespace autoware::trajectory_validator
