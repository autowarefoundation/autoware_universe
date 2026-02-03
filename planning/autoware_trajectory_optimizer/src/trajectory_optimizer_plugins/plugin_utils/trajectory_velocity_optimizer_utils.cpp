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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_velocity_optimizer_utils.hpp"

#include "autoware/trajectory_optimizer/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <rclcpp/logging.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::trajectory_optimizer::plugin::trajectory_velocity_optimizer_utils
{

void clamp_velocities(
  TrajectoryPoints & input_trajectory_array, const float min_velocity, const float min_acceleration)
{
  std::for_each(
    input_trajectory_array.begin(), input_trajectory_array.end(),
    [min_velocity, min_acceleration](TrajectoryPoint & point) {
      point.longitudinal_velocity_mps = std::max(point.longitudinal_velocity_mps, min_velocity);
      point.acceleration_mps2 = std::max(point.acceleration_mps2, min_acceleration);
    });
}

void set_max_velocity(TrajectoryPoints & input_trajectory_array, const float max_velocity)
{
  if (input_trajectory_array.empty()) {
    return;
  }

  // Handle single-point trajectory
  if (input_trajectory_array.size() == 1) {
    input_trajectory_array[0].longitudinal_velocity_mps =
      std::min(input_trajectory_array[0].longitudinal_velocity_mps, max_velocity);
    input_trajectory_array[0].acceleration_mps2 = 0.0f;
    return;
  }

  std::vector<std::pair<size_t, size_t>> modified_segment_indices;
  std::vector<double> original_dts;
  original_dts.reserve(input_trajectory_array.size() - 1);

  // Lambda to compute dt from velocity change and acceleration
  auto compute_dt_using_velocity_and_acc =
    [](const TrajectoryPoint & from, const TrajectoryPoint & to) -> double {
    const auto dv = static_cast<double>(to.longitudinal_velocity_mps) -
                    static_cast<double>(from.longitudinal_velocity_mps);
    const auto acc = static_cast<double>(from.acceleration_mps2);
    constexpr double epsilon_acceleration = 1e-6;
    const auto denominator_acc = std::max(std::abs(acc), epsilon_acceleration);
    return std::abs(dv) / denominator_acc;
  };

  // Store original dt values computed from velocity and acceleration
  for (size_t i = 0; i < input_trajectory_array.size() - 1; ++i) {
    original_dts.push_back(
      compute_dt_using_velocity_and_acc(input_trajectory_array[i], input_trajectory_array[i + 1]));
  }

  // Identify segments where velocity exceeds max_velocity
  size_t segment_start = 0;
  bool in_segment = false;

  for (size_t i = 0; i < input_trajectory_array.size(); ++i) {
    const bool exceeds_max = input_trajectory_array[i].longitudinal_velocity_mps > max_velocity;

    if (exceeds_max && !in_segment) {
      // Start of new segment
      segment_start = i;
      in_segment = true;
    } else if (!exceeds_max && in_segment) {
      // End of segment
      modified_segment_indices.emplace_back(segment_start, i - 1);
      in_segment = false;
    }
  }

  // Handle case where segment extends to end of trajectory
  if (in_segment) {
    modified_segment_indices.emplace_back(segment_start, input_trajectory_array.size() - 1);
  }

  // Cap velocities and set accelerations in offending segments
  for (const auto & [start, end] : modified_segment_indices) {
    // Cap velocities for all points in segment
    for (size_t i = start; i <= end; ++i) {
      input_trajectory_array[i].longitudinal_velocity_mps = max_velocity;
    }

    // Set acceleration to 0 for all intermediate points (constant velocity)
    // Points from start to end-1 have no velocity change to next point
    for (size_t i = start; i < end; ++i) {
      input_trajectory_array[i].acceleration_mps2 = 0.0f;
    }
  }

  // Recalculate accelerations at segment boundaries (transitions)
  for (const auto & [start, end] : modified_segment_indices) {
    // Update acceleration for point right before segment start (transition INTO segment)
    // Skip if segment starts at index 0 (no point before it)
    if (start > 0) {
      const size_t idx_before = start - 1;
      const double dt = original_dts[idx_before];
      const auto v_before =
        static_cast<double>(input_trajectory_array[idx_before].longitudinal_velocity_mps);
      const auto v_start =
        static_cast<double>(input_trajectory_array[start].longitudinal_velocity_mps);
      const double new_acc = (v_start - v_before) / dt;
      input_trajectory_array[idx_before].acceleration_mps2 = static_cast<float>(new_acc);
    }

    // Update acceleration for last point of segment (transition OUT OF segment)
    // Skip if segment ends at last trajectory point (should remain 0.0)
    if (end < input_trajectory_array.size() - 1) {
      const double dt = original_dts[end];
      const auto v_end = static_cast<double>(input_trajectory_array[end].longitudinal_velocity_mps);
      const auto v_after =
        static_cast<double>(input_trajectory_array[end + 1].longitudinal_velocity_mps);
      const double new_acc = (v_after - v_end) / dt;
      input_trajectory_array[end].acceleration_mps2 = static_cast<float>(new_acc);
    }
  }

  // Ensure last point always has zero acceleration
  input_trajectory_array.back().acceleration_mps2 = 0.0f;
}

std::vector<double> limit_lateral_acceleration(
  TrajectoryPoints & input_trajectory_array, std::vector<double> & max_velocity_per_point,
  const double max_lateral_accel_mps2, const double min_limited_speed_mps,
  const Odometry & current_odometry, const int skip_head_points, const int skip_tail_points)
{
  const size_t traj_size = input_trajectory_array.size();

  auto get_delta_time = [](const auto & next, const auto & current) -> double {
    return next->time_from_start.sec + next->time_from_start.nanosec * 1e-9 -
           (current->time_from_start.sec + current->time_from_start.nanosec * 1e-9);
  };

  const auto & current_position = current_odometry.pose.pose.position;
  motion_utils::calculate_time_from_start(input_trajectory_array, current_position);

  const auto closest_index =
    motion_utils::findNearestIndex(input_trajectory_array, current_position);

  // Calculate start and end indices, excluding head and tail points
  const size_t effective_skip_head = static_cast<size_t>(std::max(0, skip_head_points));
  const size_t effective_skip_tail = static_cast<size_t>(std::max(0, skip_tail_points));

  // Start index: max of closest_index and skip_head_points
  const size_t start_index = std::max(closest_index, effective_skip_head);

  // End index: (size - 1 - skip_tail_points) to avoid the last skip_tail_points transitions
  // The loop processes pairs (i, i+1), so we stop at (size - 1 - skip_tail_points)
  const size_t end_index =
    (traj_size > effective_skip_tail + 1) ? (traj_size - 1 - effective_skip_tail) : start_index;

  // Ensure we have a valid range
  if (start_index >= traj_size || start_index > end_index) {
    return max_velocity_per_point;
  }

  const auto start_itr = std::next(
    input_trajectory_array.begin(),
    static_cast<std::vector<TrajectoryPoint>::difference_type>(start_index));

  size_t current_index = start_index;
  for (auto itr = start_itr; itr < std::prev(input_trajectory_array.end());
       ++itr, ++current_index) {
    // Skip tail points to avoid orientation discontinuity issues
    if (current_index > end_index) {
      continue;
    }

    const auto current_pose = itr->pose;
    const auto next_pose = std::next(itr)->pose;
    const auto delta_time = get_delta_time(std::next(itr), itr);

    tf2::Quaternion q_current;
    tf2::Quaternion q_next;
    tf2::convert(current_pose.orientation, q_current);
    tf2::convert(next_pose.orientation, q_next);
    double delta_theta = q_current.angleShortestPath(q_next);
    // Handle wrap-around
    if (delta_theta > M_PI) {
      delta_theta -= 2.0 * M_PI;
    } else if (delta_theta < -M_PI) {
      delta_theta += 2.0 * M_PI;
    }

    constexpr double epsilon_yaw_rate = 1.0e-5;
    const double yaw_rate = std::max(std::abs(delta_theta / delta_time), epsilon_yaw_rate);
    const double current_speed = std::abs(itr->longitudinal_velocity_mps);
    // Compute lateral acceleration
    const double lateral_acceleration = current_speed * yaw_rate;

    if (lateral_acceleration < max_lateral_accel_mps2) {
      // No modification needed
      continue;
    }

    // Compute velocity limit based on lateral acceleration constraint
    const double limited_velocity =
      std::max(max_lateral_accel_mps2 / yaw_rate, min_limited_speed_mps);

    if (current_speed < limited_velocity) {
      // No modification needed
      continue;
    }

    // Update max velocity constraint for this point
    max_velocity_per_point.at(current_index) =
      std::min(max_velocity_per_point.at(current_index), limited_velocity);
  }

  return max_velocity_per_point;
}

void filter_velocity(
  TrajectoryPoints & input_trajectory, const InitialMotion & initial_motion,
  const double nearest_dist_threshold_m, const double nearest_yaw_threshold_rad,
  const std::shared_ptr<ContinuousJerkSmoother> & smoother, const Odometry & current_odometry,
  const std::vector<double> & max_velocity_per_point)
{
  if (!smoother) {
    auto clock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("trajectory_velocity_optimizer"), *clock, 5000,
      "ContinuousJerkSmoother is not initialized");
    return;
  }

  if (input_trajectory.size() < 2) {
    return;
  }

  const auto & initial_motion_speed = initial_motion.speed_mps;
  const auto & initial_motion_acc = initial_motion.acc_mps2;

  const size_t traj_closest = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    input_trajectory, current_odometry.pose.pose, nearest_dist_threshold_m,
    nearest_yaw_threshold_rad);

  // Clip trajectory from closest point
  TrajectoryPoints clipped;
  clipped.insert(
    clipped.end(),
    input_trajectory.begin() + static_cast<TrajectoryPoints::difference_type>(traj_closest),
    input_trajectory.end());
  input_trajectory = clipped;

  // Also clip the max velocity array if provided
  std::vector<double> clipped_max_vel;
  if (!max_velocity_per_point.empty() && max_velocity_per_point.size() > traj_closest) {
    clipped_max_vel.insert(
      clipped_max_vel.end(),
      max_velocity_per_point.begin() +
        static_cast<std::vector<double>::difference_type>(traj_closest),
      max_velocity_per_point.end());
  }

  // Apply continuous jerk smoother with per-point velocity constraints
  if (!smoother->apply(
        initial_motion_speed, initial_motion_acc, input_trajectory, input_trajectory,
        clipped_max_vel)) {
    auto clock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("trajectory_velocity_optimizer"), *clock, 5000,
      "Continuous jerk smoother optimization failed.");
  }
}

}  // namespace autoware::trajectory_optimizer::plugin::trajectory_velocity_optimizer_utils
