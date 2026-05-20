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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/velocity_modifier.hpp"

#include <autoware/trajectory/interpolator/akima_spline.hpp>
#include <autoware/trajectory/interpolator/interpolator.hpp>
#include <autoware/trajectory/trajectory_point.hpp>

#include <algorithm>
#include <utility>

namespace autoware::trajectory_modifier::plugin
{

using autoware::experimental::trajectory::interpolator::AkimaSpline;
using InterpolationTrajectory =
  autoware::experimental::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;

void VelocityModifier::on_initialize(const TrajectoryModifierParams & params)
{
  enabled_ = params.use_velocity_modifier;
  trajectory_time_step_ = params.trajectory_time_step;
  params_ = params.stopping_constraints;
}

bool VelocityModifier::is_trajectory_modification_required(
  const TrajectoryPoints & traj_points, [[maybe_unused]] const InputData & input)
{
  if (traj_points.size() < 2) return false;

  const auto stop_idx = autoware::motion_utils::searchZeroVelocityIndex(traj_points);
  if (!stop_idx.has_value() || stop_idx.value() == 0) return false;

  const auto & stop_point = traj_points.at(stop_idx.value());
  const auto & prev_point = traj_points.at(stop_idx.value() - 1);

  const auto ds =
    autoware_utils_geometry::calc_distance2d(stop_point.pose.position, prev_point.pose.position);

  const auto v_prev = prev_point.longitudinal_velocity_mps;

  static constexpr double epsilon = 1e-3;
  if (ds < epsilon) return v_prev > epsilon;

  const auto expected_accel = -1.0 * (v_prev * v_prev) / (2.0 * ds);
  const auto a_prev = prev_point.acceleration_mps2;

  constexpr double tolerance = 0.1;
  return std::abs(a_prev - expected_accel) > tolerance;
}

bool VelocityModifier::modify_trajectory(TrajectoryPoints & traj_points, const InputData & input)
{
  if (!enabled_ || !is_trajectory_modification_required(traj_points, input)) return false;

  const auto stop_idx = autoware::motion_utils::searchZeroVelocityIndex(traj_points);
  if (!stop_idx.has_value() || stop_idx.value() == 0) return false;

  auto trajectory = traj_points;

  trajectory.erase(trajectory.begin() + stop_idx.value() + 1, trajectory.end());
  trajectory.back().longitudinal_velocity_mps = 0.0;
  trajectory.back().acceleration_mps2 = 0.0;

  const auto traj_length = motion_utils::calcArcLength(trajectory);

  constexpr double min_v_ref = 1.0;
  const auto v_ref = std::max<double>(min_v_ref, trajectory.front().longitudinal_velocity_mps);
  const auto required_decel = std::abs(v_ref * v_ref / (2.0 * traj_length));
  const auto jerk = std::abs(params_.jerk_limit);
  const auto decel = std::clamp(
    required_decel, std::abs(params_.nominal_deceleration), std::abs(params_.maximum_deceleration));

  const auto vel_update_start_index = update_velocities(trajectory, jerk, decel);
  const auto interpolate_start_index = vel_update_start_index > 0 ? vel_update_start_index - 1 : 0;

  auto trajectory_interpolation_util =
    InterpolationTrajectory::Builder{}
      .set_xy_interpolator<AkimaSpline>()  // Set interpolator for x-y plane
      .set_longitudinal_velocity_interpolator<AkimaSpline>()
      .build(trajectory);
  if (!trajectory_interpolation_util) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM VelocityModifier] Failed to build interpolation trajectory");
    return false;
  }

  const auto dt = trajectory_time_step_;

  if (dt < 1e-3) {
    RCLCPP_ERROR_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM VelocityModifier] Invalid trajectory time step: %f, unable to interpolate trajectory",
      dt);
    traj_points = std::move(trajectory);
    return true;
  }

  const auto s_max = trajectory_interpolation_util->length();
  const auto interpolate_start_arc_length =
    trajectory_interpolation_util->get_underlying_bases().at(interpolate_start_index);
  trajectory_interpolation_util->align_orientation_with_trajectory_direction();
  traj_points.erase(traj_points.begin() + interpolate_start_index + 1, traj_points.end());
  double s_curr{interpolate_start_arc_length};
  while (s_curr <= s_max) {
    const auto v_curr = traj_points.back().longitudinal_velocity_mps;
    const auto a_curr = traj_points.back().acceleration_mps2;
    const auto t_curr = rclcpp::Duration(traj_points.back().time_from_start);
    if (v_curr < 1e-3) break;

    double ds = v_curr * dt + 0.5 * a_curr * dt * dt;
    if (ds < 1e-3) break;

    s_curr += ds;
    const auto s_clamped = std::min(s_curr, s_max);

    auto p = trajectory_interpolation_util->compute(s_clamped);
    p.acceleration_mps2 = (p.longitudinal_velocity_mps - v_curr) / static_cast<float>(dt);
    p.time_from_start = t_curr + rclcpp::Duration::from_seconds(dt);
    traj_points.push_back(p);
  }

  return true;
}

size_t VelocityModifier::update_velocities(
  TrajectoryPoints & trajectory, const double jerk, const double decel) const
{
  if (trajectory.size() < 2) return 0;

  auto get_vel_accel = [&](
                         const auto & point, const auto & ref_point) -> std::pair<double, double> {
    const auto v_ref = ref_point.longitudinal_velocity_mps;
    const auto a_ref = std::abs(ref_point.acceleration_mps2);

    const auto ds =
      autoware_utils_geometry::calc_distance2d(point.pose.position, ref_point.pose.position);
    const auto da = jerk * (ds / std::max<double>(v_ref, 0.1));
    const auto a_curr = std::min(a_ref + da, decel);
    const auto a_avg = (a_ref + a_curr) / 2.0;
    const auto v_curr = std::sqrt(std::max(0.0, v_ref * v_ref + 2.0 * a_avg * ds));
    return {v_curr, -1.0 * a_curr};
  };

  const auto stop_index = trajectory.size() - 1;
  auto vel_update_start_index = stop_index;
  for (int i = static_cast<int>(stop_index) - 1; i > 0; --i) {
    auto & curr = trajectory.at(i);
    auto & next = trajectory.at(i + 1);
    const auto [v_curr, a_curr] = get_vel_accel(curr, next);

    if (v_curr >= curr.longitudinal_velocity_mps) break;
    curr.longitudinal_velocity_mps = static_cast<float>(v_curr);
    curr.acceleration_mps2 = static_cast<float>(a_curr);
    vel_update_start_index = i;
  }
  return vel_update_start_index;
}

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::VelocityModifier,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
