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

#ifndef AUTOWARE__TRAJECTORY_RANKER__UTILS_HPP_
#define AUTOWARE__TRAJECTORY_RANKER__UTILS_HPP_

#include <rclcpp/duration.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <optional>
#include <vector>

namespace autoware::trajectory_ranker::utils
{

using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

/**
 * @brief Samples trajectory points at fixed time intervals from the ego position
 * @param points Input trajectory to sample from
 * @param p_ego Current ego vehicle pose
 * @param sample_num Number of samples to extract
 * @param resolution Time interval between samples [s]
 * @return Resampled trajectory with uniform time spacing
 */
TrajectoryPoints sampling(
  const TrajectoryPoints & points, const Pose & p_ego, const size_t sample_num,
  const double resolution);

/**
 * @brief Interpolates between two trajectory points
 * @param curr_pt Current trajectory point
 * @param next_pt Next trajectory point
 * @param ratio Interpolation ratio (0.0 = curr_pt, 1.0 = next_pt)
 * @param use_zero_order_hold_for_twist If true, keeps curr_pt velocity/acceleration
 * @return Interpolated trajectory point
 */
TrajectoryPoint calc_interpolated_point(
  const TrajectoryPoint & curr_pt, const TrajectoryPoint & next_pt, const double ratio,
  const bool use_zero_order_hold_for_twist);

}  // namespace autoware::trajectory_ranker::utils

#endif  // AUTOWARE__TRAJECTORY_RANKER__UTILS_HPP_
