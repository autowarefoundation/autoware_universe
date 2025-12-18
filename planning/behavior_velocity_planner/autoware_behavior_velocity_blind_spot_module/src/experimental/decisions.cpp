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

#include "scene.hpp"

#include <autoware/trajectory/utils/find_nearest.hpp>

#include <algorithm>

namespace autoware::behavior_velocity_planner::experimental
{

/*
 * for default
 */
template <typename T>
void BlindSpotModule::setRTCStatusByDecision(
  const T &, [[maybe_unused]] const Trajectory & path,
  [[maybe_unused]] const PlannerData & planner_data)
{
  static_assert("Unsupported type passed to setRTCStatus");
  return;
}

template <typename T>
void BlindSpotModule::reactRTCApprovalByDecision(
  [[maybe_unused]] const T & decision, [[maybe_unused]] Trajectory & path,
  [[maybe_unused]] const PlannerData & planner_data)
{
  static_assert("Unsupported type passed to reactRTCApprovalByDecision");
}

/*
 * for InternalError
 */
template <>
void BlindSpotModule::setRTCStatusByDecision(
  [[maybe_unused]] const InternalError & decision, [[maybe_unused]] const Trajectory & path,
  [[maybe_unused]] const PlannerData & planner_data)
{
  return;
}

template <>
void BlindSpotModule::reactRTCApprovalByDecision(
  [[maybe_unused]] const InternalError & decision, [[maybe_unused]] Trajectory & path,
  [[maybe_unused]] const PlannerData & planner_data)
{
  return;
}

/*
 * For OverPassJudge
 */
template <>
void BlindSpotModule::setRTCStatusByDecision(
  [[maybe_unused]] const OverPassJudge & decision, [[maybe_unused]] const Trajectory & path,
  [[maybe_unused]] const PlannerData & planner_data)
{
  return;
}

template <>
void BlindSpotModule::reactRTCApprovalByDecision(
  [[maybe_unused]] const OverPassJudge & decision, [[maybe_unused]] Trajectory & path,
  [[maybe_unused]] const PlannerData & planner_data)
{
  return;
}

/*
 * for Unsafe
 */
template <>
void BlindSpotModule::setRTCStatusByDecision(
  const Unsafe & decision, const Trajectory & path, const PlannerData & planner_data)
{
  setSafe(false);
  setDistance(
    decision.stop_s - autoware::experimental::trajectory::find_nearest_index(
                        path, planner_data.current_odometry->pose.position));
}

template <>
void BlindSpotModule::reactRTCApprovalByDecision(
  const Unsafe & decision, Trajectory & path, const PlannerData & planner_data)
{
  if (isActivated()) {
    return;
  }

  path.longitudinal_velocity_mps()
    .range(decision.stop_s, path.length())
    .set([](const double & current_vel) {
      constexpr auto stop_vel = 0.0;
      return std::min(current_vel, stop_vel);
    });

  debug_data_.virtual_wall_pose =
    path.compute(decision.stop_s + planner_data.vehicle_info_.max_longitudinal_offset_m).point.pose;

  planning_factor_interface_->add(
    path.restore(), planner_data.current_odometry->pose, path.compute(decision.stop_s).point.pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "blind_spot(module is judging as UNSAFE)");
}

/*
 * for Safe
 */
template <>
void BlindSpotModule::setRTCStatusByDecision(
  const Safe & decision, const Trajectory & path, const PlannerData & planner_data)
{
  setSafe(true);
  setDistance(
    decision.stop_s - autoware::experimental::trajectory::find_nearest_index(
                        path, planner_data.current_odometry->pose.position));
}

template <>
void BlindSpotModule::reactRTCApprovalByDecision(
  const Safe & decision, Trajectory & path, const PlannerData & planner_data)
{
  if (isActivated()) {
    return;
  }

  path.longitudinal_velocity_mps()
    .range(decision.stop_s, path.length())
    .set([](const double & current_vel) {
      constexpr auto stop_vel = 0.0;
      return std::min(current_vel, stop_vel);
    });

  debug_data_.virtual_wall_pose =
    path.compute(decision.stop_s + planner_data.vehicle_info_.max_longitudinal_offset_m).point.pose;

  planning_factor_interface_->add(
    path.restore(), planner_data.current_odometry->pose, path.compute(decision.stop_s).point.pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "blind_spot(module is judging as SAFE and RTC is not approved)");
}

}  // namespace autoware::behavior_velocity_planner::experimental
