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

#include "autoware/trajectory_modifier/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <cmath>

namespace autoware::trajectory_modifier::utils
{

bool validate_trajectory(const TrajectoryPoints & trajectory)
{
  return !trajectory.empty();
}

double calculate_distance_to_last_point(
  const TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & ego_pose)
{
  if (traj_points.empty()) {
    return 0.0;
  }

  return autoware::motion_utils::calcSignedArcLength(
    traj_points, ego_pose.position, traj_points.back().pose.position);
}

void replace_trajectory_with_stop_point(
  TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & ego_pose)
{
  TrajectoryPoint stop_point;

  stop_point.pose = ego_pose;
  stop_point.longitudinal_velocity_mps = 0.0;
  stop_point.lateral_velocity_mps = 0.0;
  stop_point.acceleration_mps2 = 0.0;
  stop_point.heading_rate_rps = 0.0;
  stop_point.front_wheel_angle_rad = 0.0;
  stop_point.rear_wheel_angle_rad = 0.0;

  traj_points.clear();

  // Two points are added since that is the minimum handled by Control.
  traj_points.push_back(stop_point);
  traj_points.push_back(stop_point);
}

bool is_ego_vehicle_moving(const geometry_msgs::msg::Twist & twist, const double velocity_threshold)
{
  const double current_velocity = std::sqrt(
    twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y +
    twist.linear.z * twist.linear.z);

  return current_velocity > velocity_threshold;
}

}  // namespace autoware::trajectory_modifier::utils
