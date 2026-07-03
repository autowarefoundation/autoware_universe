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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_mpt_optimizer_utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::trajectory_optimizer::plugin::trajectory_mpt_optimizer_utils
{

double calculate_acceleration_from_velocity_and_distance(
  const TrajectoryPoint & p_curr, const TrajectoryPoint & p_next)
{
  const double delta_s =
    autoware_utils_geometry::calc_distance2d(p_curr.pose.position, p_next.pose.position);
  if (delta_s < 1e-6) {
    return 0.0;
  }

  // Use kinematic formula: a = (v² - v₀²) / (2s)
  const double v_next_sq = p_next.longitudinal_velocity_mps * p_next.longitudinal_velocity_mps;
  const double v_curr_sq = p_curr.longitudinal_velocity_mps * p_curr.longitudinal_velocity_mps;
  return (v_next_sq - v_curr_sq) / (2.0 * delta_s);
}

double calculate_time_interval(
  const double v, const double a, const TrajectoryPoint & p_curr, const TrajectoryPoint & p_next)
{
  const double delta_s =
    autoware_utils_geometry::calc_distance2d(p_curr.pose.position, p_next.pose.position);
  constexpr double min_velocity = 1e-3;  // 1mm/s threshold

  if (std::abs(a) < 1e-6) {
    // Constant velocity model
    if (std::abs(v) < min_velocity) {
      // Vehicle nearly stopped, return small dt
      return 0.1;
    }
    return std::abs(delta_s / v);
  }

  const double discriminant = v * v + 2.0 * a * delta_s;
  if (discriminant < 0.0) {
    // Physically invalid scenario - fallback to constant velocity
    if (std::abs(v) < min_velocity) {
      return 0.1;
    }
    return std::abs(delta_s / v);
  }

  const double v_next = std::sqrt(discriminant);
  return (v_next - v) / a;
}

void recalculate_trajectory_dynamics(TrajectoryPoints & traj_points, const int smoothing_window)
{
  if (traj_points.empty()) {
    return;
  }

  // Set first point time_from_start to zero
  traj_points.front().time_from_start.sec = 0;
  traj_points.front().time_from_start.nanosec = 0;

  // Calculate acceleration and time for each segment
  for (size_t i = 1; i < traj_points.size(); ++i) {
    auto & next_point = traj_points[i];
    auto & curr_point = traj_points[i - 1];

    // Calculate acceleration from velocity change and distance
    const auto velocity = static_cast<double>(curr_point.longitudinal_velocity_mps);
    const double acceleration =
      calculate_acceleration_from_velocity_and_distance(curr_point, next_point);

    // Calculate time interval
    const double dt = calculate_time_interval(velocity, acceleration, curr_point, next_point);

    // Update time_from_start using rclcpp::Duration for safe conversion
    const double curr_time = rclcpp::Duration(curr_point.time_from_start).seconds();
    const double new_time = curr_time + dt;
    next_point.time_from_start = rclcpp::Duration::from_seconds(new_time);

    // Update acceleration
    curr_point.acceleration_mps2 = static_cast<float>(acceleration);
  }

  // Apply moving average filter to acceleration
  const int window_size = std::max(1, smoothing_window);
  std::vector<float> original_accelerations;
  original_accelerations.reserve(traj_points.size());
  for (const auto & point : traj_points) {
    original_accelerations.push_back(point.acceleration_mps2);
  }

  for (size_t i = 0; i < traj_points.size() - 1; ++i) {
    // Calculate moving average using backward-looking window
    double sum = 0.0;
    int count = 0;
    const int start_idx = std::max(0, static_cast<int>(i) - window_size + 1);
    for (int j = start_idx; j <= static_cast<int>(i); ++j) {
      sum += original_accelerations[j];
      count++;
    }
    traj_points[i].acceleration_mps2 = static_cast<float>(sum / count);
  }

  // Set last point acceleration to zero
  traj_points.back().acceleration_mps2 = 0.0f;
}

double calculate_curvature_at_point(const TrajectoryPoints & traj_points, const size_t idx)
{
  if (traj_points.size() < 3 || idx == 0 || idx >= traj_points.size() - 1) {
    return 0.0;
  }

  const auto & p_prev = traj_points[idx - 1].pose.position;
  const auto & p_curr = traj_points[idx].pose.position;
  const auto & p_next = traj_points[idx + 1].pose.position;

  const double dx1 = p_curr.x - p_prev.x;
  const double dy1 = p_curr.y - p_prev.y;
  const double dx2 = p_next.x - p_curr.x;
  const double dy2 = p_next.y - p_curr.y;

  const double angle1 = std::atan2(dy1, dx1);
  const double angle2 = std::atan2(dy2, dx2);
  double angle_diff = angle2 - angle1;

  // Handle angle wrapping
  while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

  const double arc_length = std::hypot(dx1, dy1) + std::hypot(dx2, dy2);

  return (arc_length > 1e-6) ? (angle_diff / arc_length) : 0.0;
}

double calculate_corridor_width(
  const double curvature, const double velocity, const double base_width,
  const double curvature_factor, const double velocity_factor)
{
  // Add extra width in curves
  const double curvature_addition = curvature_factor * std::abs(curvature);

  // Add extra width at low speeds
  constexpr double max_velocity = 15.0;  // m/s
  const double velocity_scaling = std::max(0.0, (max_velocity - velocity) / max_velocity);
  const double velocity_addition = velocity_factor * velocity_scaling;

  return base_width + curvature_addition + velocity_addition;
}

BoundsPair generate_bounds(
  const TrajectoryPoints & traj_points, const double corridor_width_m,
  const bool enable_adaptive_width, const double curvature_width_factor,
  const double velocity_width_factor, const double min_clearance_m, const double vehicle_width_m)
{
  BoundsPair bounds;
  bounds.left_bound.reserve(traj_points.size());
  bounds.right_bound.reserve(traj_points.size());

  for (size_t i = 0; i < traj_points.size(); ++i) {
    const auto & point = traj_points[i];
    const auto & pose = point.pose;

    // Calculate corridor width
    double corridor_width = corridor_width_m;

    if (enable_adaptive_width) {
      const double curvature = calculate_curvature_at_point(traj_points, i);
      const double velocity = point.longitudinal_velocity_mps;
      corridor_width = calculate_corridor_width(
        curvature, velocity, corridor_width_m, curvature_width_factor, velocity_width_factor);
    }

    // Ensure minimum clearance on both sides
    // corridor_width is per-side offset, vehicle extends vehicle_width/2 from center
    const double min_width = vehicle_width_m / 2.0 + min_clearance_m;
    corridor_width = std::max(corridor_width, min_width);

    // Get yaw from quaternion
    const double yaw = tf2::getYaw(pose.orientation);

    // Calculate perpendicular offsets
    const double left_yaw = yaw + M_PI_2;
    const double right_yaw = yaw - M_PI_2;

    // Left bound point
    geometry_msgs::msg::Point left_point;
    left_point.x = pose.position.x + corridor_width * std::cos(left_yaw);
    left_point.y = pose.position.y + corridor_width * std::sin(left_yaw);
    left_point.z = pose.position.z;
    bounds.left_bound.push_back(left_point);

    // Right bound point
    geometry_msgs::msg::Point right_point;
    right_point.x = pose.position.x + corridor_width * std::cos(right_yaw);
    right_point.y = pose.position.y + corridor_width * std::sin(right_yaw);
    right_point.z = pose.position.z;
    bounds.right_bound.push_back(right_point);
  }

  return bounds;
}

}  // namespace autoware::trajectory_optimizer::plugin::trajectory_mpt_optimizer_utils
