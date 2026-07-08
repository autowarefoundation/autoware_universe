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

#ifndef AUTOWARE__DIFFUSION_PLANNER__UTILS__EGO_STATE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__UTILS__EGO_STATE_HPP_

#include <rclcpp/time.hpp>

#include "nav_msgs/msg/odometry.hpp"

#include <deque>
#include <utility>

namespace autoware::diffusion_planner::utils
{

/**
 * @brief Selects the current ego state from the odometry buffer at frame_time.
 *
 * When use_time_interpolation is set, the pose and twist are linearly interpolated between the two
 * buffered samples that bracket frame_time. Otherwise, or when frame_time falls outside the
 * buffered range, the sample nearest to frame_time is returned.
 *
 * @param ego_history Odometry buffer ordered oldest-to-newest (must be non-empty).
 * @param frame_time Object-anchored synchronization time at which to sample the ego state.
 * @param use_time_interpolation Interpolate between bracketing samples instead of taking the
 *        nearest one.
 * @return The selected ego state and its absolute time offset from frame_time [s] (0 when
 *         interpolated within the buffer range).
 */
std::pair<nav_msgs::msg::Odometry, double> select_ego_state(
  const std::deque<nav_msgs::msg::Odometry> & ego_history, const rclcpp::Time & frame_time,
  const bool use_time_interpolation);

}  // namespace autoware::diffusion_planner::utils
#endif  // AUTOWARE__DIFFUSION_PLANNER__UTILS__EGO_STATE_HPP_
