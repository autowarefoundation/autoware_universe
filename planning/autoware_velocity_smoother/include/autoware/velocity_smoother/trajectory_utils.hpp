// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__VELOCITY_SMOOTHER__TRAJECTORY_UTILS_HPP_
#define AUTOWARE__VELOCITY_SMOOTHER__TRAJECTORY_UTILS_HPP_

#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <map>
#include <optional>
#include <tuple>
#include <vector>

namespace autoware::velocity_smoother::trajectory_utils
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using geometry_msgs::msg::Pose;

TrajectoryPoint calc_interpolated_trajectory_point(
  const TrajectoryPoints & trajectory, const Pose & target_pose, const size_t seg_idx);

TrajectoryPoints extract_path_around_index(
  const TrajectoryPoints & trajectory, const size_t index, const double & ahead_length,
  const double & behind_length);

std::vector<double> calc_arclength_array(const TrajectoryPoints & trajectory);

std::vector<double> calc_trajectory_interval_distance(const TrajectoryPoints & trajectory);

std::vector<double> calc_trajectory_curvature_from_3_points(
  const TrajectoryPoints & trajectory, size_t idx_dist);

void apply_maximum_velocity_limit(
  const size_t from, const size_t to, const double max_vel, TrajectoryPoints & trajectory);

// std::optional<size_t> search_zero_velocity_idx(const TrajectoryPoints & trajectory);

bool calc_stop_dist_with_jerk_constraints(
  const double v0, const double a0, const double jerk_acc, const double jerk_dec,
  const double min_acc, const double target_vel, std::map<double, double> & jerk_profile,
  double & stop_dist);

bool is_valid_stop_dist(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin);

std::optional<std::tuple<double, double, double, double>> update_state_with_jerk_constraint(
  const double v0, const double a0, const std::map<double, double> & jerk_profile, const double t);

std::vector<double> calc_velocity_profile_with_constant_jerk_and_acceleration_limit(
  const TrajectoryPoints & trajectory, const double v0, const double a0, const double jerk,
  const double acc_max, const double acc_min);

double calc_stop_distance(const TrajectoryPoints & trajectory, const size_t closest);

}  // namespace autoware::velocity_smoother::trajectory_utils

#endif  // AUTOWARE__VELOCITY_SMOOTHER__TRAJECTORY_UTILS_HPP_
