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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_extender_utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::trajectory_optimizer::plugin::trajectory_extender_utils
{

void add_ego_state_to_trajectory(
  TrajectoryPoints & traj_points, const Odometry & current_odometry,
  const double nearest_dist_threshold_m, const double nearest_yaw_threshold_rad,
  const double backward_trajectory_extension_m)
{
  TrajectoryPoint ego_state;
  ego_state.pose = current_odometry.pose.pose;
  ego_state.longitudinal_velocity_mps = static_cast<float>(current_odometry.twist.twist.linear.x);

  if (traj_points.empty()) {
    traj_points.push_back(ego_state);
    return;
  }
  const auto & last_point = traj_points.back();
  const auto ego_yaw_rad = tf2::getYaw(ego_state.pose.orientation);
  const auto last_point_yaw_rad = tf2::getYaw(last_point.pose.orientation);

  const auto yaw_diff =
    std::abs(autoware_utils_math::normalize_radian(ego_yaw_rad - last_point_yaw_rad));
  const auto distance = autoware_utils_geometry::calc_distance2d(last_point, ego_state);
  constexpr double epsilon{1e-2};
  constexpr double epsilon_rad{1e-2};
  const bool is_change_small = distance < epsilon && yaw_diff < epsilon_rad;
  if (is_change_small) {
    return;
  }

  const bool is_change_large =
    distance > nearest_dist_threshold_m || yaw_diff > nearest_yaw_threshold_rad;
  if (is_change_large) {
    traj_points = {ego_state};
    return;
  }

  traj_points.push_back(ego_state);

  size_t clip_idx = 0;
  double accumulated_length = 0.0;
  for (size_t i = traj_points.size() - 1; i > 0; i--) {
    accumulated_length +=
      autoware_utils_geometry::calc_distance2d(traj_points.at(i - 1), traj_points.at(i));
    if (accumulated_length > backward_trajectory_extension_m) {
      clip_idx = i;
      break;
    }
  }
  traj_points.erase(traj_points.begin(), traj_points.begin() + static_cast<int>(clip_idx));
}

void expand_trajectory_with_ego_history(
  TrajectoryPoints & traj_points, const TrajectoryPoints & ego_history_points,
  const Odometry & current_odometry)
{
  if (ego_history_points.empty()) {
    return;
  }

  if (traj_points.empty()) {
    traj_points.insert(traj_points.begin(), ego_history_points.begin(), ego_history_points.end());
    return;
  }

  const auto first_ego_history_point = ego_history_points.front();
  const auto first_ego_trajectory_point = traj_points.front();
  const auto first_ego_trajectory_point_arc_length = autoware::motion_utils::calcSignedArcLength(
    ego_history_points, first_ego_history_point.pose.position,
    first_ego_trajectory_point.pose.position);

  const auto ego_position = current_odometry.pose.pose.position;
  const auto distance_ego_to_first_trajectory_point =
    autoware_utils_geometry::calc_distance2d(first_ego_trajectory_point, ego_position);

  std::for_each(ego_history_points.rbegin(), ego_history_points.rend(), [&](const auto & point) {
    const auto point_arc_length = autoware::motion_utils::calcSignedArcLength(
      ego_history_points, first_ego_history_point.pose.position, point.pose.position);

    const bool is_ahead_of_first_point = point_arc_length > first_ego_trajectory_point_arc_length;
    const bool is_closer_than_first_trajectory_point =
      autoware_utils_geometry::calc_distance2d(point, ego_position) <
      distance_ego_to_first_trajectory_point;
    const bool is_point_already_in_trajectory =
      std::any_of(traj_points.begin(), traj_points.end(), [&](const TrajectoryPoint & traj_point) {
        return autoware_utils_geometry::calc_distance2d(traj_point, point) < 1e-1;
      });
    if (
      is_ahead_of_first_point || is_point_already_in_trajectory ||
      is_closer_than_first_trajectory_point) {
      return;
    }

    traj_points.insert(traj_points.begin(), point);
    traj_points.front().longitudinal_velocity_mps =
      first_ego_trajectory_point.longitudinal_velocity_mps;
    traj_points.front().acceleration_mps2 = first_ego_trajectory_point.acceleration_mps2;
  });
}

}  // namespace autoware::trajectory_optimizer::plugin::trajectory_extender_utils
