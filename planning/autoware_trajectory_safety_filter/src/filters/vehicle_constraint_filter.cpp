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

#include "autoware/trajectory_safety_filter/filters/vehicle_constraint_filter.hpp"

#include <builtin_interfaces/msg/duration.hpp>

#include <string>
#include <unordered_map>

namespace autoware::trajectory_safety_filter::plugin
{
namespace
{
/**
 * @brief Convert a Duration message to seconds.
 */
double to_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}

/**
 * @brief Convert TrajectoryPoint to velocity (m/s)
 */
double to_velocity(const TrajectoryPoint & point)
{
  return std::sqrt(
    point.longitudinal_velocity_mps * point.longitudinal_velocity_mps +
    point.lateral_velocity_mps * point.lateral_velocity_mps);
}

/**
 * @brief Convert two TrajectoryPoints to acceleration (m/s^2)
 */
double to_acceleration(const TrajectoryPoint & prev_point, const TrajectoryPoint & curr_point)
{
  double prev_vel = to_velocity(prev_point);
  double curr_vel = to_velocity(curr_point);
  double dt = to_seconds(curr_point.time_from_start) - to_seconds(prev_point.time_from_start);
  return dt > 0 ? (curr_vel - prev_vel) / dt : 0.0;
}

/**
 * @brief Convert TrajectoryPoint to steering angle (rad)
 */
double to_steering_angle(const TrajectoryPoint & point, const VehicleInfo & vehicle_info)
{
  return std::atan2(point.lateral_velocity_mps, point.longitudinal_velocity_mps) *
         vehicle_info.wheel_base_m;
}

/**
 * @brief Convert two TrajectoryPoints to steering angle rate (rad/s)
 */
double to_steering_angle_rate(
  const TrajectoryPoint & prev_point, const TrajectoryPoint & curr_point,
  const VehicleInfo & vehicle_info)
{
  double prev_steering_angle = to_steering_angle(prev_point, vehicle_info);
  double curr_steering_angle = to_steering_angle(curr_point, vehicle_info);
  double dt = to_seconds(curr_point.time_from_start) - to_seconds(prev_point.time_from_start);
  return dt > 0 ? std::abs(curr_steering_angle - prev_steering_angle) / dt : 0.0;
}
}  // namespace

VehicleConstraintFilter::VehicleConstraintFilter()
: SafetyFilterInterface("VehicleConstraintFilter")
{
}

void VehicleConstraintFilter::set_parameters(
  const std::unordered_map<std::string, std::any> & params)
{
  auto get_value = [&params](const std::string & key, auto & value) {
    auto it = params.find(key);
    if (it != params.end()) {
      try {
        value = std::any_cast<std::decay_t<decltype(value)>>(it->second);
      } catch (const std::bad_any_cast &) {
        // Keep default value if cast fails
      }
    }
  };

  // Update parameters if provided, otherwise keep defaults
  get_value("max_velocity", params_.max_velocity);
  get_value("max_acceleration", params_.max_acceleration);
  get_value("max_deceleration", params_.max_deceleration);
  get_value("max_steering_angle", params_.max_steering_angle);
  get_value("max_steering_angle_rate", params_.max_steering_angle_rate);
}

tl::expected<void, std::string> VehicleConstraintFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext &)
{
  if (!vehicle_info_ptr_) {
    return tl::make_unexpected("Vehicle info not set");
  }

  // NOTE: Feasibility decision logic might be more complex in the future, but for now we just check
  // all constraints and return false if any are violated
  bool is_feasible =
    check_velocity(traj_points, params_.max_velocity) &&
    check_acceleration(traj_points, params_.max_acceleration) &&
    check_deceleration(traj_points, params_.max_deceleration) &&
    check_steering_angle(traj_points, *vehicle_info_ptr_, params_.max_steering_angle) &&
    check_steering_angle_rate(traj_points, *vehicle_info_ptr_, params_.max_steering_angle_rate);

  if (!is_feasible) {
    return tl::make_unexpected("Trajectory violates vehicle constraints");
  }

  return {};
}

// --- Helper functions for constraint checks ---

bool check_velocity(const TrajectoryPoints & traj_points, double max_velocity)
{
  for (const auto & point : traj_points) {
    double vel = to_velocity(point);
    if (vel > max_velocity) {
      return false;
    }
  }
  return true;
}

bool check_acceleration(const TrajectoryPoints & traj_points, double max_acceleration)
{
  for (size_t i = 1; i < traj_points.size(); ++i) {
    double acc = to_acceleration(traj_points[i - 1], traj_points[i]);
    if (acc > max_acceleration) {
      return false;
    }
  }
  return true;
}

bool check_deceleration(const TrajectoryPoints & traj_points, double max_deceleration)
{
  for (size_t i = 1; i < traj_points.size(); ++i) {
    double dec = to_acceleration(traj_points[i - 1], traj_points[i]);
    if (dec < 0 && std::abs(dec) > max_deceleration) {
      return false;
    }
  }
  return true;
}

bool check_steering_angle(
  const TrajectoryPoints & traj_points, const VehicleInfo & vehicle_info, double max_steering_angle)
{
  for (const auto & point : traj_points) {
    double steering_angle = to_steering_angle(point, vehicle_info);
    if (std::abs(steering_angle) > max_steering_angle) {
      return false;
    }
  }
  return true;
}

bool check_steering_angle_rate(
  const TrajectoryPoints & traj_points, const VehicleInfo & vehicle_info,
  double max_steering_angle_rate)
{
  for (size_t i = 1; i < traj_points.size(); ++i) {
    double steering_angle_rate =
      to_steering_angle_rate(traj_points[i - 1], traj_points[i], vehicle_info);
    if (steering_angle_rate > max_steering_angle_rate) {
      return false;
    }
  }
  return true;
}
}  // namespace autoware::trajectory_safety_filter::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_safety_filter::plugin::VehicleConstraintFilter,
  autoware::trajectory_safety_filter::plugin::SafetyFilterInterface)
