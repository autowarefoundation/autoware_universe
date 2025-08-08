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

#include "slow_down_interpolator.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::experimental::utils
{
using Response = SlowDownInterpolator::Response;
tl::expected<SlowDownInterpolator::SlowDownPlan, std::string>
SlowDownInterpolator::get_interp_to_point(
  const double curr_vel, const double curr_acc, const double lon_dist_to_bound_m,
  const double lat_dist_to_bound_m, const SideKey side_key) const
{
  const auto target_vel = interp_velocity(curr_vel, lat_dist_to_bound_m, side_key);

  if (lon_dist_to_bound_m <= 0.0) return tl::make_unexpected("Point behind ego.");  // already past

  const auto a_comfort = th_trigger_.th_acc_mps2.min;
  const auto j_comfort = th_trigger_.th_jerk_mps3.min;
  const auto a_max = th_trigger_.th_acc_mps2.max;
  const auto j_max = th_trigger_.th_jerk_mps3.max;

  const auto comfort_dist_opt =
    get_comfort_distance(lon_dist_to_bound_m, curr_vel, target_vel, curr_acc);

  if (comfort_dist_opt) {
    const auto v_brake_opt = calc_velocity_with_profile(
      curr_acc, curr_vel, target_vel, j_comfort, a_comfort, lon_dist_to_bound_m);
    return v_brake_opt
             ? tl::expected<SlowDownPlan, std::string>(
                 SlowDownPlan{*comfort_dist_opt, *v_brake_opt, a_comfort})
             : tl::make_unexpected(
                 "Failed to calculate velocity with comfort profile." + v_brake_opt.error());
  }

  const auto v_brake_opt =
    calc_velocity_with_profile(curr_acc, curr_vel, target_vel, j_max, a_max, lon_dist_to_bound_m);

  return v_brake_opt ? tl::expected<SlowDownPlan, std::string>(
                         SlowDownPlan{*comfort_dist_opt, *v_brake_opt, a_comfort})
                     : tl::make_unexpected(
                         "Failed to calculate velocity with max profile: " + v_brake_opt.error());
}

double SlowDownInterpolator::interp_velocity(
  const double curr_vel, const double lat_dist, const SideKey side_key) const
{
  const auto min_vel = th_trigger_.th_vel_mps.min;

  if (curr_vel <= min_vel) {
    return min_vel;
  }

  const auto min_dist = th_trigger_.th_dist_to_boundary_m[side_key].min;
  const auto max_dist = th_trigger_.th_dist_to_boundary_m[side_key].max;
  const auto lat_dist_axis = std::vector{min_dist, max_dist};

  const auto max_vel = th_trigger_.th_vel_mps.max;
  const auto vel_axis = std::vector{min_vel, max_vel};

  if (lat_dist >= max_dist) {
    return max_vel;
  }

  if (lat_dist <= min_dist) {
    return min_vel;
  }

  return autoware::interpolation::lerp(lat_dist_axis, vel_axis, lat_dist);
}

double SlowDownInterpolator::v_t(const double t, const double j, const double a, const double v)
{
  return j * t * t / 2.0 + a * t + v;
}

double SlowDownInterpolator::s_t(const double t, const double j, const double a, const double v)
{
  return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t;
}

tl::expected<double, std::string> SlowDownInterpolator::find_reach_time(
  double j, double a, double v0, double dist, double t_min, double t_max)
{
  const auto d = [&](double t) { return s_t(t, j, a, v0) - dist; };

  if (d(t_min) > 0.0 || d(t_max) < 0.0) return tl::make_unexpected("invalid search range");

  for (int iter = 0; iter < 120; ++iter) {  // ~2-32 precision
    const auto t_mid = 0.5 * (t_min + t_max);
    const auto d_m = d(t_mid);

    if (std::abs(d_m) < std::numeric_limits<double>::epsilon()) {
      return t_mid;
    }

    if (d_m > 0.0) {
      t_max = t_mid;
    } else {
      t_min = t_mid;
    }
  }

  return 0.5 * (t_min + t_max);
}

double SlowDownInterpolator::t_j(const double a_0, const double a, const double j)
{
  return (a - a_0) / j;
}

double SlowDownInterpolator::d_slow(
  const double v_0, const double v_target, const double a_0, const double a_brake,
  const double j_brake)
{
  const auto t_brake = t_j(a_0, a_brake, j_brake);
  const auto s_brake = s_t(t_brake, j_brake, a_0, v_0);
  const auto v_brake = v_t(t_brake, j_brake, a_0, v_0);

  // If the jerk ramp alone already reaches v_target
  if (v_brake <= v_target) {
    const auto a = 0.5 * j_brake;
    const auto b = a_0;
    const auto c = v_0 - v_target;

    const auto quad = b * b - 4.0 * a * c;
    if (quad < 0.0) {
      return 0.0;
    }

    const auto root = (-b + std::sqrt(quad)) / (2.0 * a);
    const auto t_hit = std::max(0.0, root);
    return s_t(t_hit, j_brake, a_0, v_0);
  }

  const auto s_const_acc_stop = (v_brake * v_brake - v_target * v_target) / (-2.0 * a_brake);
  return s_brake + s_const_acc_stop;
}

std::optional<double> SlowDownInterpolator::get_comfort_distance(
  const double lon_dist_to_dpt_pt, const double v_0, const double v_target, const double a_0) const
{
  const auto a_comfort = th_trigger_.th_acc_mps2.min;
  const auto j_comfort = th_trigger_.th_jerk_mps3.min;
  const auto d_comfort = d_slow(v_0, v_target, a_0, a_comfort, j_comfort);
  if (d_comfort <= lon_dist_to_dpt_pt) {
    return d_comfort;
  }
  return std::nullopt;
}

tl::expected<double, std::string> SlowDownInterpolator::calc_velocity_with_profile(
  const double a_0, const double v_0, const double v_target, const double j_brake,
  const double a_brake, const double lon_dist_to_dpt_pt)
{
  const auto a_lim = std::max(a_0, a_brake);

  const auto t_brake = t_j(a_lim, a_brake, j_brake);
  const auto d_brake = s_t(t_brake, j_brake, a_lim, v_0);
  const auto v_brake = v_t(t_brake, j_brake, a_lim, v_0);

  /* ---------- Case 1: Target lies inside jerk ramp ---------- */
  {
    /* find time when distance covered equals 'lon_dist_to_dpt_pt' */
    auto d = [&](double t) { return s_t(t, j_brake, a_lim, v_0) - lon_dist_to_dpt_pt; };
    if (d(0.0) * d(t_brake) < 0.0) {  // sign change ⇒ lies inside
      const auto t_hit_opt = find_reach_time(j_brake, a_lim, v_0, lon_dist_to_dpt_pt, 0.0, t_brake);
      if (!t_hit_opt) {
        return tl::make_unexpected("Failed to find reach time. " + t_hit_opt.error());
      }
      return std::max(v_target, v_t(*t_hit_opt, j_brake, a_lim, v_0));
    }
  }

  /* ---------- Case 2: constant-accel phase to v_target ---------- */
  const auto remaining_dist = lon_dist_to_dpt_pt - d_brake;
  const auto disc =
    v_brake * v_brake - v_target * v_target + 2.0 * a_brake * remaining_dist;  // a_brake ≤ 0

  if (disc < 0.0) return v_target;  // v_target reached before waypoint → hold it

  /* time at constant a_brake to cover remaining_dist */
  const auto t_a = (-v_brake + std::sqrt(disc)) / a_brake;
  return std::max(v_target, v_t(t_a, 0.0, a_brake, v_brake));  // should equal v_target numerically
}
}  // namespace autoware::motion_velocity_planner::experimental::utils
