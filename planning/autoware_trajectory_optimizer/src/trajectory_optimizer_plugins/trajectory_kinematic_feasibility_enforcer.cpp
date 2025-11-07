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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_kinematic_feasibility_enforcer.hpp"

#include "autoware/trajectory_optimizer/utils.hpp"

#include <Eigen/Core>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

namespace
{
// Local helper functions

double normalize_angle(const double angle)
{
  // Normalize angle to [-pi, pi]
  double normalized = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (normalized < 0.0) {
    normalized += 2.0 * M_PI;
  }
  return normalized - M_PI;
}

double compute_average_dt(const TrajectoryPoints & traj_points)
{
  constexpr double default_dt = 0.1;  // Fallback value [s]

  if (traj_points.size() < 2) {
    return default_dt;
  }

  // Compute average dt across all segments
  double total_dt = 0.0;
  for (size_t i = 0; i + 1 < traj_points.size(); ++i) {
    total_dt +=
      autoware::trajectory_optimizer::utils::compute_dt(traj_points[i], traj_points[i + 1]);
  }

  const double avg_dt = total_dt / static_cast<double>(traj_points.size() - 1);

  // Sanity check: dt should be reasonable (1 microsecond to 1 second)
  if (avg_dt < 1e-6 || avg_dt > 1.0) {
    return default_dt;
  }

  return avg_dt;
}

}  // namespace

void TrajectoryKinematicFeasibilityEnforcer::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
  const TrajectoryOptimizerData & data)
{
  // Check if plugin is enabled
  if (!params.use_kinematic_feasibility_enforcer) {
    return;
  }

  // Need at least 2 points
  if (traj_points.size() < 2) {
    return;
  }

  // Always use ego pose as anchor (current vehicle state)
  const auto & anchor_pose = data.current_odometry.pose.pose;

  // Compute average dt from trajectory time stamps
  const double avg_dt = compute_average_dt(traj_points);

  // Apply kinematic feasibility constraints
  // This adjusts positions and headings while preserving segment distances
  // Velocities and time stamps remain unchanged to preserve dt structure for QP smoother
  enforce_ackermann_yaw_rate_constraints(
    traj_points, anchor_pose, vehicle_info_.wheel_base_m, vehicle_info_.max_steer_angle_rad,
    feasibility_params_.max_yaw_rate_rad_s, avg_dt);
}

void TrajectoryKinematicFeasibilityEnforcer::enforce_ackermann_yaw_rate_constraints(
  TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & anchor_pose,
  const double wheelbase, const double max_steer_rad, const double max_yaw_rate,
  const double dt) const
{
  // Minimum segment distance for numerical stability
  constexpr double min_segment_distance = 1e-6;  // 1 micron

  // Extract initial yaw from anchor pose
  tf2::Quaternion q_anchor;
  tf2::convert(anchor_pose.orientation, q_anchor);
  double current_yaw = tf2::getYaw(q_anchor);

  // Current anchor position
  Eigen::Vector2d anchor_pos(anchor_pose.position.x, anchor_pose.position.y);

  // Maximum curvature from Ackermann constraint
  // κ_max = tan(δ_max) / L
  const double kappa_max = std::tan(max_steer_rad) / wheelbase;

  // Process each trajectory point
  for (auto & point : traj_points) {
    // Extract original point position
    Eigen::Vector2d orig_point(point.pose.position.x, point.pose.position.y);

    // Compute vector from anchor to original point
    Eigen::Vector2d v = orig_point - anchor_pos;

    // Clamp segment distance to minimum for numerical stability
    // Don't skip short segments - that would create duplicates
    const double s = std::max(v.norm(), min_segment_distance);

    // Desired heading from anchor to original point
    const double desired_yaw = std::atan2(v.y(), v.x());

    // Compute desired yaw change (normalized to [-pi, pi])
    double delta_yaw_desired = normalize_angle(desired_yaw - current_yaw);

    // Compute Ackermann geometric constraint
    // Maximum yaw change based on maximum curvature over distance s
    // Δψ_geom = κ_max * s = (tan(δ_max) / L) * s
    const double delta_yaw_geom = kappa_max * s;

    // Compute yaw rate constraint
    // Maximum yaw change based on angular rate limit over time dt
    // Δψ_rate = ψ_dot_max * Δt
    const double delta_yaw_rate = max_yaw_rate * dt;

    // Take tighter bound (both constraints must be satisfied)
    const double delta_yaw_max = std::min(delta_yaw_geom, delta_yaw_rate);

    // Clamp desired change to feasible range
    const double delta_yaw_clamped = std::clamp(delta_yaw_desired, -delta_yaw_max, delta_yaw_max);

    // Update heading
    current_yaw += delta_yaw_clamped;

    // Compute new point position maintaining segment distance s
    // This preserves the implicit dt = s / v_avg between points
    const Eigen::Vector2d new_point =
      anchor_pos + s * Eigen::Vector2d(std::cos(current_yaw), std::sin(current_yaw));

    // Update trajectory point
    point.pose.position.x = new_point.x();
    point.pose.position.y = new_point.y();

    // Update orientation
    tf2::Quaternion q_new;
    q_new.setRPY(0.0, 0.0, current_yaw);
    point.pose.orientation = tf2::toMsg(q_new);

    // Update anchor for next iteration (forward propagation)
    anchor_pos = new_point;
  }
}

void TrajectoryKinematicFeasibilityEnforcer::set_up_params()
{
  auto node_ptr = get_node_ptr();
  using autoware_utils_rclcpp::get_or_declare_parameter;

  // Get vehicle info
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*node_ptr).getVehicleInfo();

  // Plugin-specific parameter
  feasibility_params_.max_yaw_rate_rad_s = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_kinematic_feasibility.max_yaw_rate_rad_s");

  // Log configuration
  RCLCPP_INFO(
    node_ptr->get_logger(),
    "Kinematic Feasibility Enforcer initialized: max_yaw_rate=%.3f rad/s (%.1f deg/s), "
    "wheelbase=%.2f m, max_steer_angle=%.3f rad (%.1f deg)",
    feasibility_params_.max_yaw_rate_rad_s, feasibility_params_.max_yaw_rate_rad_s * 180.0 / M_PI,
    vehicle_info_.wheel_base_m, vehicle_info_.max_steer_angle_rad,
    vehicle_info_.max_steer_angle_rad * 180.0 / M_PI);
}

rcl_interfaces::msg::SetParametersResult TrajectoryKinematicFeasibilityEnforcer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param<double>(
    parameters, "trajectory_kinematic_feasibility.max_yaw_rate_rad_s",
    feasibility_params_.max_yaw_rate_rad_s);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_optimizer::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_optimizer::plugin::TrajectoryKinematicFeasibilityEnforcer,
  autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase)
