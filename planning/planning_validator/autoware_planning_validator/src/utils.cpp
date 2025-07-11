// Copyright 2022 Tier IV, Inc.
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

#include "autoware/planning_validator/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_validator
{
using autoware_utils::calc_distance2d;

// Do not interpolate.
Trajectory resampleTrajectory(const Trajectory & trajectory, const double min_interval)
{
  Trajectory resampled;
  resampled.header = trajectory.header;

  if (trajectory.points.empty()) {
    return resampled;
  }

  resampled.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto prev = resampled.points.back();
    const auto curr = trajectory.points.at(i);
    if (calc_distance2d(prev, curr) > min_interval) {
      resampled.points.push_back(curr);
    }
  }
  return resampled;
}

double calculateStoppingDistance(
  const double current_vel, const double current_accel, const double decel, const double jerk_limit)
{
  // calculate time to ramp acceleration from current accel to decel
  const auto t1 = std::max((current_accel - decel) / jerk_limit, 0.0);
  // calculate velocity and distance after t1
  const auto v1 = current_vel + current_accel * t1 - 0.5 * jerk_limit * t1 * t1;
  const auto d1 =
    (current_vel * t1) + (0.5 * current_accel * t1 * t1) - (jerk_limit * t1 * t1 * t1 / 6.0);
  // calculate distance to stop from v1
  const auto d2 = std::abs((v1 * v1) / (2 * decel));
  return d1 + d2;
}

Trajectory getStopTrajectory(
  const Trajectory & trajectory, const int nearest_traj_idx, const double current_vel,
  const double current_accel, const double decel, const double jerk_limit)
{
  const auto stopping_distance =
    calculateStoppingDistance(current_vel, current_accel, decel, jerk_limit);

  Trajectory soft_stop_traj = trajectory;
  soft_stop_traj.header = trajectory.header;
  double accumulated_distance = 0.0;
  static constexpr double zero_velocity_th = 0.5;
  for (size_t i = nearest_traj_idx + 1; i < trajectory.points.size(); ++i) {
    accumulated_distance += calc_distance2d(trajectory.points.at(i - 1), trajectory.points.at(i));
    if (
      accumulated_distance >= stopping_distance ||
      soft_stop_traj.points.at(i - 1).longitudinal_velocity_mps < zero_velocity_th) {
      soft_stop_traj.points.at(i).longitudinal_velocity_mps = 0.0;
      continue;
    }
    const float interpolated_velocity =
      current_vel * (stopping_distance - accumulated_distance) / stopping_distance;
    soft_stop_traj.points.at(i).longitudinal_velocity_mps =
      std::min(interpolated_velocity, soft_stop_traj.points.at(i).longitudinal_velocity_mps);
  }
  soft_stop_traj.points.back().longitudinal_velocity_mps = 0.0;
  return soft_stop_traj;
}

void shiftPose(geometry_msgs::msg::Pose & pose, double longitudinal)
{
  const auto yaw = tf2::getYaw(pose.orientation);
  pose.position.x += std::cos(yaw) * longitudinal;
  pose.position.y += std::sin(yaw) * longitudinal;
}

}  // namespace autoware::planning_validator
