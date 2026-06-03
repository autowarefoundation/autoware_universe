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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__UTILS_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__UTILS_HPP_

#include <tl_expected/expected.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <vector>

namespace autoware::trajectory_modifier::utils
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

bool validate_trajectory(const TrajectoryPoints & trajectory);

double calculate_distance_to_last_point(
  const TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & ego_pose);

void replace_trajectory_with_stop_point(
  TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & ego_pose);

bool is_ego_vehicle_moving(
  const geometry_msgs::msg::Twist & twist, const double velocity_threshold);

double clamp_stop_point_arc_length(
  const double stop_point_arc_length, const double max_length, const double ego_vel,
  const double ego_accel, const double decel_limit, const double jerk_limit);

bool stop_point_exists(
  const TrajectoryPoints & traj_points, const double stop_point_arc_length,
  const double duplicate_check_threshold = 0.0);

bool insert_stop_point(
  TrajectoryPoints & trajectory, const double stop_point_arc_length, const double traj_length);

}  // namespace autoware::trajectory_modifier::utils

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__UTILS_HPP_
