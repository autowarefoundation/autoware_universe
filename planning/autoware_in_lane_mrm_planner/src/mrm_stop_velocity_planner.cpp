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

#include "mrm_stop_velocity_planner.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>
#include <vector>

namespace autoware::in_lane_mrm_planner
{
namespace
{
constexpr double kStopVelocityThreshold = 0.01;
constexpr double kMinVelocityForTimeCalc = 0.1;
constexpr double kIntegrationDt = 0.02;

double extract_longitudinal_accel(const AccelWithCovarianceStamped & accel)
{
  return accel.accel.accel.linear.x;
}

std::vector<double> calc_arc_lengths(const TrajectoryPoints & points)
{
  std::vector<double> arc_lengths(points.size(), 0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    arc_lengths.at(i) = autoware::motion_utils::calcSignedArcLength(points, 0, i);
  }
  return arc_lengths;
}

TrajectoryPoint sample_point_at_arc_length(const TrajectoryPoints & points, const double s)
{
  if (points.empty()) {
    return TrajectoryPoint{};
  }
  if (s <= 0.0) {
    return points.front();
  }

  const double total_length =
    autoware::motion_utils::calcSignedArcLength(points, 0, points.size() - 1);
  if (s >= total_length) {
    return points.back();
  }

  const auto pose = autoware::motion_utils::calcInterpolatedPose(points, s);
  Trajectory trajectory;
  trajectory.points = points;
  return autoware::motion_utils::calcInterpolatedPoint(trajectory, pose, false);
}

void advance_decel_state(
  double & v, double & a, double & delay_remaining, const double ds, const double jerk,
  const double a_target)
{
  if (ds <= 0.0 || v <= 0.0) {
    v = 0.0;
    a = a_target;
    return;
  }

  double remaining = ds;
  while (remaining > 1e-9 && v > 0.0) {
    const double dt = std::min(kIntegrationDt, remaining / std::max(v, kStopVelocityThreshold));
    if (delay_remaining > 0.0) {
      // Brake command still in flight: hold the current (clamped) acceleration.
      delay_remaining -= dt;
    } else if (a > a_target) {
      a += jerk * dt;
      a = std::max(a, a_target);
    } else {
      a = a_target;
    }
    v += a * dt;
    if (v <= 0.0) {
      v = 0.0;
      a = a_target;
      break;
    }
    remaining -= v * dt;
  }
}

double calc_required_stop_distance(
  const double v0, const double a0, const double delay_time, const double jerk,
  const double a_target)
{
  double v = std::max(0.0, v0);
  double a = a0;
  double delay_remaining = delay_time;
  double distance = 0.0;
  constexpr double max_distance = 500.0;

  while (v > kStopVelocityThreshold && distance < max_distance) {
    const double dt = kIntegrationDt;
    const double ds = std::max(v * dt, kStopVelocityThreshold * dt);
    advance_decel_state(v, a, delay_remaining, ds, jerk, a_target);
    distance += ds;
  }
  return distance;
}

bool can_relax_jerk(const double jerk, const double max_jerk)
{
  return jerk > max_jerk;
}

bool can_relax_decel(const double decel, const double max_decel)
{
  return decel > max_decel;
}

}  // namespace

MrmStopVelocityPlanner::MrmStopVelocityPlanner(const Params & params) : params_(params.mrm_velocity)
{
}

void MrmStopVelocityPlanner::update_params(const Params & params)
{
  params_ = params.mrm_velocity;
}

MrmStopVelocityPlanner::ProfileLimits MrmStopVelocityPlanner::profile_limits(
  const StopProfile profile) const
{
  const auto to_limits = [](const auto & p) {
    return ProfileLimits{
      p.target_jerk, p.target_deceleration, p.max_jerk_relaxation, p.max_deceleration_relaxation};
  };
  switch (profile) {
    case StopProfile::EMERGENCY:
      return to_limits(params_.profiles.emergency);
    case StopProfile::MODERATE:
    default:
      return to_limits(params_.profiles.moderate);
  }
}

double MrmStopVelocityPlanner::effective_initial_accel(const double a0) const
{
  // While the brake command is in flight the drive is assumed to cut immediately, so a
  // positive current acceleration cannot persist; an already-applied brake keeps acting.
  return params_.brake_delay_time > 0.0 ? std::min(a0, 0.0) : a0;
}

std::optional<size_t> MrmStopVelocityPlanner::find_constraint_stop_index(
  const TrajectoryPoints & points)
{
  if (points.size() < 2) {
    return std::nullopt;
  }

  for (size_t i = 0; i + 1 < points.size(); ++i) {
    if (points.at(i).longitudinal_velocity_mps <= kStopVelocityThreshold) {
      return i;
    }
  }
  return points.size() - 1;
}

bool MrmStopVelocityPlanner::is_feasible(
  const TrajectoryPoints & points, const size_t ego_idx, const size_t constraint_idx,
  const double v0, const double a0, const double jerk, const double decel) const
{
  if (points.empty() || ego_idx >= points.size() || constraint_idx >= points.size()) {
    return false;
  }

  const double available_distance =
    autoware::motion_utils::calcSignedArcLength(points, ego_idx, constraint_idx);
  const double required_distance =
    calc_required_stop_distance(v0, a0, params_.brake_delay_time, jerk, decel);
  return available_distance + 1e-3 >= required_distance;
}

double MrmStopVelocityPlanner::required_stop_distance(
  const double v0, const double a0, const double jerk, const double decel) const
{
  return calc_required_stop_distance(
    v0, effective_initial_accel(a0), params_.brake_delay_time, jerk, decel);
}

MrmStopVelocityPlanner::DecelLimits MrmStopVelocityPlanner::select_profile_limits(
  const TrajectoryPoints & points, const size_t ego_idx, const size_t constraint_idx,
  const double v0, const double a0, const StopProfile profile) const
{
  const double a0_eff = effective_initial_accel(a0);
  const auto profile_params = profile_limits(profile);
  DecelLimits limits{profile_params.target_jerk, profile_params.target_deceleration};

  if (is_feasible(points, ego_idx, constraint_idx, v0, a0_eff, limits.jerk, limits.decel)) {
    return limits;
  }

  // Hard iteration cap guarantees termination even if relaxation params are misconfigured
  // (e.g. zero/wrong-sign step) so that is_feasible never becomes true and the relax branches
  // never make progress. The legitimate case needs only a handful of steps (with default params:
  // jerk (20-5)/5 + decel (6-3)/1 = 6), so this leaves ample margin while staying small.
  constexpr int max_relaxation_iterations = 20;
  int iterations = 0;
  while (!is_feasible(points, ego_idx, constraint_idx, v0, a0_eff, limits.jerk, limits.decel)) {
    if (++iterations > max_relaxation_iterations) {
      RCLCPP_ERROR(
        rclcpp::get_logger("mrm_stop_velocity_planner"),
        "Relaxation did not converge after %d iterations (check relaxation params); applying max "
        "relaxation limits",
        max_relaxation_iterations);
      limits.jerk = profile_params.max_jerk_relaxation;
      limits.decel = profile_params.max_deceleration_relaxation;
      break;
    }
    // Clamp each step to the relaxation limit so that a step which does not divide the range
    // evenly (e.g. -1.5 -> -2.5 -> -3.5 with max -3.0) never exceeds max_*_relaxation.
    if (can_relax_jerk(limits.jerk, profile_params.max_jerk_relaxation)) {
      limits.jerk =
        std::max(limits.jerk + params_.step_jerk_relaxation, profile_params.max_jerk_relaxation);
      continue;
    }
    if (can_relax_decel(limits.decel, profile_params.max_deceleration_relaxation)) {
      limits.decel = std::max(
        limits.decel + params_.step_deceleration_relaxation,
        profile_params.max_deceleration_relaxation);
      continue;
    }

    RCLCPP_ERROR(
      rclcpp::get_logger("mrm_stop_velocity_planner"),
      "Cannot stop before constraint index %zu; applying max relaxation limits", constraint_idx);
    limits.jerk = profile_params.max_jerk_relaxation;
    limits.decel = profile_params.max_deceleration_relaxation;
    break;
  }

  return limits;
}

void MrmStopVelocityPlanner::densify_near_arc_length(
  TrajectoryPoints & points, const double center_arc_length) const
{
  if (points.size() < 2 || params_.decel_resample_interval <= 0.0) {
    return;
  }

  const auto arc_lengths = calc_arc_lengths(points);
  const double s_begin = std::max(0.0, center_arc_length - params_.decel_resample_range);
  const double s_end =
    std::min(arc_lengths.back(), center_arc_length + params_.decel_resample_range);

  std::set<double> sample_s;
  sample_s.insert(arc_lengths.front());
  sample_s.insert(arc_lengths.back());

  for (const double s : arc_lengths) {
    if (s < s_begin || s > s_end) {
      sample_s.insert(s);
    }
  }

  for (double s = s_begin; s <= s_end + 1e-6; s += params_.decel_resample_interval) {
    sample_s.insert(s);
  }

  // The accumulated grid samples and the exact terminal (or window-boundary) arc lengths can
  // differ by only a few floating-point rounding steps, in which case sampling both yields two
  // output points at the same position. Downstream consumers (e.g. MPC spline resampling) require
  // strictly increasing arc length, so drop samples closer than half the resample interval to
  // their predecessor. The exact terminal must survive so the stop point / trajectory end is
  // preserved: on collision the preceding grid sample is replaced by the terminal.
  const double back = arc_lengths.back();
  const double min_gap = 0.5 * params_.decel_resample_interval;
  std::vector<double> filtered;
  filtered.reserve(sample_s.size());
  for (const double s : sample_s) {
    const double clamped = std::min(s, back);
    if (filtered.empty() || clamped - filtered.back() >= min_gap) {
      filtered.push_back(clamped);
    }
  }
  if (!filtered.empty() && filtered.back() < back) {
    if (back - filtered.back() < min_gap) {
      filtered.back() = back;
    } else {
      filtered.push_back(back);
    }
  }

  TrajectoryPoints resampled;
  resampled.reserve(filtered.size());
  for (const double s : filtered) {
    resampled.push_back(sample_point_at_arc_length(points, s));
  }
  points = std::move(resampled);
}

void MrmStopVelocityPlanner::fill_zero_velocity_profile(
  TrajectoryPoints & points, const float longitudinal_accel_mps2)
{
  for (auto & point : points) {
    point.longitudinal_velocity_mps = 0.0F;
    point.lateral_velocity_mps = 0.0F;
    point.acceleration_mps2 = longitudinal_accel_mps2;
  }
}

void MrmStopVelocityPlanner::apply_zero_stop_profile(
  TrajectoryPoints & points, const Odometry & odom, const float longitudinal_accel_mps2)
{
  fill_zero_velocity_profile(points, longitudinal_accel_mps2);
  autoware::motion_utils::calculate_time_from_start(
    points, odom.pose.pose.position, static_cast<float>(kMinVelocityForTimeCalc));
}

void MrmStopVelocityPlanner::fill_ego_prefix(
  TrajectoryPoints & points, const size_t ego_idx, const double v0, const double a0)
{
  const size_t last_idx = std::min(ego_idx, points.empty() ? 0U : points.size() - 1);
  for (size_t i = 0; i <= last_idx; ++i) {
    points.at(i).longitudinal_velocity_mps = static_cast<float>(v0);
    points.at(i).acceleration_mps2 = static_cast<float>(a0);
  }
}

void MrmStopVelocityPlanner::fill_forward(
  TrajectoryPoints & points, const size_t ego_idx, const double v0, const double a0,
  const double jerk, const double decel) const
{
  if (points.empty() || ego_idx >= points.size()) {
    return;
  }

  double v = std::max(0.0, v0);
  double a = a0;
  double delay_remaining = params_.brake_delay_time;
  bool stopped = false;

  points.at(ego_idx).longitudinal_velocity_mps = static_cast<float>(v);
  points.at(ego_idx).acceleration_mps2 = static_cast<float>(a);

  for (size_t i = ego_idx + 1; i < points.size(); ++i) {
    if (stopped) {
      points.at(i).longitudinal_velocity_mps = 0.0F;
      points.at(i).acceleration_mps2 = static_cast<float>(decel);
      continue;
    }

    const double ds = autoware::motion_utils::calcSignedArcLength(points, i - 1, i);
    advance_decel_state(v, a, delay_remaining, ds, jerk, decel);

    if (v <= kStopVelocityThreshold) {
      stopped = true;
      v = 0.0;
      a = decel;
    }

    points.at(i).longitudinal_velocity_mps = static_cast<float>(v);
    points.at(i).acceleration_mps2 = static_cast<float>(a);
  }
}

void MrmStopVelocityPlanner::apply(
  TrajectoryPoints & points, const Odometry & odom, const AccelWithCovarianceStamped & accel,
  const StopProfile profile) const
{
  if (points.empty()) {
    return;
  }

  const double v0 = std::max(0.0, odom.twist.twist.linear.x);
  const double a0 = effective_initial_accel(extract_longitudinal_accel(accel));

  if (v0 <= kStopVelocityThreshold) {
    apply_zero_stop_profile(points, odom, static_cast<float>(a0));
    return;
  }

  const size_t ego_idx =
    autoware::motion_utils::findNearestSegmentIndex(points, odom.pose.pose.position);
  const size_t constraint_idx = find_constraint_stop_index(points).value_or(points.size() - 1);

  const auto limits = select_profile_limits(points, ego_idx, constraint_idx, v0, a0, profile);

  if (!is_feasible(points, ego_idx, constraint_idx, v0, a0, limits.jerk, limits.decel)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("mrm_stop_velocity_planner"),
      "Cannot stop before constraint index %zu; applying zero velocity fallback", constraint_idx);
    apply_zero_stop_profile(
      points, odom, static_cast<float>(profile_limits(profile).max_deceleration_relaxation));
    return;
  }

  const double ego_arc_length = autoware::motion_utils::calcSignedArcLength(points, 0, ego_idx);
  const double predicted_stop_arc_length =
    ego_arc_length +
    calc_required_stop_distance(v0, a0, params_.brake_delay_time, limits.jerk, limits.decel);
  densify_near_arc_length(points, predicted_stop_arc_length);

  const size_t ego_idx_after =
    autoware::motion_utils::findNearestSegmentIndex(points, odom.pose.pose.position);

  fill_forward(points, ego_idx_after, v0, a0, limits.jerk, limits.decel);
  fill_ego_prefix(points, ego_idx_after, v0, a0);

  autoware::motion_utils::calculate_time_from_start(
    points, odom.pose.pose.position, static_cast<float>(kMinVelocityForTimeCalc));
}

}  // namespace autoware::in_lane_mrm_planner
