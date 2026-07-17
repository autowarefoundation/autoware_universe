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

#include "autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <cmath>

namespace autoware::trajectory_modifier::utils
{

bool validate_trajectory(const TrajectoryPoints & trajectory)
{
  return !trajectory.empty();
}

double calculate_distance_to_last_point(
  const TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & src_pose)
{
  if (traj_points.empty()) {
    return 0.0;
  }

  return autoware::motion_utils::calcSignedArcLength(
    traj_points, src_pose.position, traj_points.size() - 1);
}

void replace_trajectory_with_stop_point(
  TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & ego_pose, const double time_step)
{
  TrajectoryPoint stop_point;

  stop_point.pose = ego_pose;
  stop_point.longitudinal_velocity_mps = 0.0;
  stop_point.lateral_velocity_mps = 0.0;
  stop_point.acceleration_mps2 = 0.0;
  stop_point.heading_rate_rps = 0.0;
  stop_point.front_wheel_angle_rad = 0.0;
  stop_point.rear_wheel_angle_rad = 0.0;
  stop_point.time_from_start = rclcpp::Duration::from_seconds(0.0);

  traj_points.clear();

  // Three points are added since that is the minimum handled by Control.
  traj_points.push_back(stop_point);
  stop_point.time_from_start = rclcpp::Duration::from_seconds(time_step);
  stop_point.pose = autoware_utils_geometry::calc_offset_pose(stop_point.pose, 1e-3, 0.0, 0.0);
  traj_points.push_back(stop_point);
  stop_point.time_from_start = rclcpp::Duration::from_seconds(2.0 * time_step);
  stop_point.pose = autoware_utils_geometry::calc_offset_pose(stop_point.pose, 1e-3, 0.0, 0.0);
  traj_points.push_back(stop_point);
}

bool is_ego_vehicle_moving(const geometry_msgs::msg::Twist & twist, const double velocity_threshold)
{
  const double current_velocity = std::sqrt(
    twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y +
    twist.linear.z * twist.linear.z);

  return current_velocity > velocity_threshold;
}

double clamp_stop_point_arc_length(
  const double stop_point_arc_length, const double max_length, const double ego_vel,
  const double ego_accel, const double decel_limit, const double jerk_limit)
{
  auto min_stopping_distance =
    motion_utils::calculate_stop_distance(ego_vel, ego_accel, decel_limit, jerk_limit, 0.0);
  if (!min_stopping_distance) min_stopping_distance = 0.0;
  return std::clamp(stop_point_arc_length, min_stopping_distance.value(), max_length);
}

bool stop_point_exists(
  const TrajectoryPoints & traj_points, const double stop_point_arc_length,
  const double duplicate_check_threshold)
{
  constexpr double stop_velocity_threshold = 0.01;
  auto checked_distance = 0.0;
  for (size_t i = 1; i < traj_points.size(); ++i) {
    const auto & curr = traj_points.at(i);
    const auto & prev = traj_points.at(i - 1);
    checked_distance +=
      autoware_utils_geometry::calc_distance2d(curr.pose.position, prev.pose.position);
    if (checked_distance > stop_point_arc_length + duplicate_check_threshold) break;
    if (curr.longitudinal_velocity_mps < stop_velocity_threshold) {
      return true;
    }
  }
  return false;
}

bool insert_stop_point(
  TrajectoryPoints & trajectory, const double stop_point_arc_length, const double time_step)
{
  if (trajectory.empty()) return false;

  if (stop_point_arc_length < 1e-3) {
    replace_trajectory_with_stop_point(trajectory, trajectory.front().pose, time_step);
    return true;
  }

  constexpr double overlap_threshold = 0.1;

  auto distance = 0.0;
  auto seg_length = 0.0;
  size_t idx = 0;
  for (; idx < trajectory.size() - 1; ++idx) {
    seg_length = autoware_utils_geometry::calc_distance2d(
      trajectory[idx].pose.position, trajectory[idx + 1].pose.position);
    if (distance + seg_length > stop_point_arc_length) break;
    distance += seg_length;
  }

  if (idx == trajectory.size() - 1) {
    trajectory.back().longitudinal_velocity_mps = 0.0;
    trajectory.back().acceleration_mps2 = 0.0;
    return true;
  }

  auto stop_idx = idx;
  if (std::abs(distance - stop_point_arc_length) < overlap_threshold) {
    stop_idx = idx;
  } else if (std::abs(distance + seg_length - stop_point_arc_length) < overlap_threshold) {
    stop_idx = idx + 1;
  } else {
    auto target_idx =
      motion_utils::insertStopPoint(idx, stop_point_arc_length - distance, trajectory);
    stop_idx = target_idx ? target_idx.value() : idx;
  }

  if (stop_idx + 1 < trajectory.size()) {
    trajectory.erase(trajectory.begin() + stop_idx + 1, trajectory.end());
  }
  trajectory.back().longitudinal_velocity_mps = 0.0;
  trajectory.back().acceleration_mps2 = 0.0;

  return true;
}

bool is_stop_trajectory(const TrajectoryPoints & traj_points, const double stopped_vel_th)
{
  if (traj_points.empty()) return false;

  if (calculate_distance_to_last_point(traj_points, traj_points.front().pose) > 1.0) {
    return false;
  }

  return std::find_if(traj_points.begin(), traj_points.end(), [&](const TrajectoryPoint & point) {
           return point.longitudinal_velocity_mps > stopped_vel_th;
         }) == traj_points.end();
}

}  // namespace autoware::trajectory_modifier::utils
