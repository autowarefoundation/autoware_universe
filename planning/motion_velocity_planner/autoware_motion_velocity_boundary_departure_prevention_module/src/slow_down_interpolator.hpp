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

#include "type_alias.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <tl_expected/expected.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#ifndef SLOW_DOWN_INTERPOLATOR_HPP_
#define SLOW_DOWN_INTERPOLATOR_HPP_

namespace autoware::motion_velocity_planner
{

class SlowDownInterpolator
{
public:
  SlowDownInterpolator(
    const std::shared_ptr<const PlannerData> & planner_data, const TriggerThreshold & th_trigger)
  : planner_data_(planner_data), th_trigger_(th_trigger)
  {
  }

  [[nodiscard]] tl::expected<std::tuple<double, double, double>, std::string> get_interp_to_point(
    const geometry_msgs::msg::Point & slow_down_point, const double lat_dist_to_bound_m,
    const SideKey side_key) const
  {
    const auto arclength_to_point_m = calc_dist_to_point(slow_down_point);

    const auto curr_vel = get_curr_vel();
    const auto interp_vel_mps = interp_velocity(curr_vel, lat_dist_to_bound_m, side_key);
    const auto exp_decel_mps2 = calc_expected_deceleration(interp_vel_mps, arclength_to_point_m);

    const auto max_decel = th_trigger_.th_acc_mps2.max;
    if (std::abs(exp_decel_mps2) > std::abs(max_decel)) {
      return tl::unexpected<std::string>(
        "Expected deceleration for the interpolated velocity exceeded threshold.");
    }

    const auto exp_jerk = interp_jerk(lat_dist_to_bound_m, exp_decel_mps2, side_key);
    const auto exp_vel = calc_new_velocity(exp_decel_mps2, exp_jerk, arclength_to_point_m);

    const auto rel_dist_m =
      (interp_vel_mps * interp_vel_mps - curr_vel * curr_vel) / (2 * exp_decel_mps2);
    fmt::print(
      "vel {:.3f}, curr vel {:.3f}, decel {:.3f}, jerk {:.3f}, rel dist {:.3f}\n", interp_vel_mps,
      exp_vel, exp_decel_mps2, exp_jerk, rel_dist_m);

    return std::make_tuple(rel_dist_m, exp_vel, exp_decel_mps2);
  }

private:
  [[nodiscard]] double calc_expected_deceleration(
    const double target_vel, const double arclength_to_point_m) const
  {
    const auto & th_acc_mps2 = th_trigger_.th_acc_mps2;
    if (arclength_to_point_m < std::numeric_limits<double>::epsilon()) {
      return th_acc_mps2.max;
    }

    const auto curr_vel = get_curr_vel();
    const auto raw_decel =
      (target_vel * target_vel - curr_vel * curr_vel) / (2.0 * arclength_to_point_m);
    return std::clamp(raw_decel, th_acc_mps2.max, th_acc_mps2.min);
  }

  [[nodiscard]] double calc_dist_to_point(const Point2d & slow_down_point) const
  {
    return calc_dist_to_point(autoware_utils::to_msg(slow_down_point.to_3d(0.0)));
  }

  [[nodiscard]] double calc_dist_to_point(const geometry_msgs::msg::Point & slow_down_point) const
  {
    const auto & curr_point = planner_data_->current_odometry.pose.pose.position;
    return autoware_utils::calc_distance2d(curr_point, slow_down_point);
  }

  [[nodiscard]] double interp_velocity(
    const double curr_vel, const double lat_dist, const SideKey side_key) const
  {
    if (curr_vel <= th_trigger_.max_slow_down_vel_mps) {
      return th_trigger_.max_slow_down_vel_mps;
    }

    const auto max_dist = th_trigger_.th_dist_to_boundary_m[side_key].max;
    const auto exp_dist = lat_dist < max_dist ? lat_dist : max_dist;

    const auto lat_dist_axis = get_lat_dist_axis(side_key);
    const auto vel_axis = get_vel_axis(curr_vel);
    return autoware::interpolation::lerp(lat_dist_axis, vel_axis, exp_dist);
  }

  [[nodiscard]] std::vector<double> get_lat_dist_axis(const SideKey side_key) const
  {
    return {
      th_trigger_.th_dist_to_boundary_m[side_key].min,
      th_trigger_.th_dist_to_boundary_m[side_key].max};
  }

  [[nodiscard]] std::vector<double> get_vel_axis(const double curr_vel) const
  {
    return {th_trigger_.max_slow_down_vel_mps, curr_vel};
  }

  [[nodiscard]] double get_curr_vel() const
  {
    return std::abs(planner_data_->current_odometry.twist.twist.linear.x);
  }

  [[nodiscard]] double calc_new_velocity(
    double a_target, double jerk, const double lon_dist_to_point_m) const
  {
    const auto curr_vel = get_curr_vel();
    const auto th_slow_down_vel = th_trigger_.max_slow_down_vel_mps;

    if (curr_vel < th_slow_down_vel) {
      return th_slow_down_vel;
    }

    const double t = lon_dist_to_point_m / std::max(curr_vel, 1e-6);  // avoid div by zero
    const double t_j = std::abs(a_target / std::max(jerk, 1e-6));

    if (t <= t_j) {
      return std::clamp(
        curr_vel + 0.5 * jerk * t * t, th_slow_down_vel,
        std::max(th_slow_down_vel + std::numeric_limits<double>::epsilon(), curr_vel));
    }

    const double delta_v1 = 0.5 * a_target * a_target / jerk;
    const double delta_v2 = a_target * (t - t_j);

    return std::clamp(
      curr_vel + delta_v1 + delta_v2, th_slow_down_vel,
      std::max(th_slow_down_vel + std::numeric_limits<double>::epsilon(), curr_vel));
  };

  [[nodiscard]] double interp_jerk(
    const double lat_dist_to_bound_m, const double expected_decel_mps2,
    const SideKey side_key) const
  {
    const auto th_dist_to_boundary_m = th_trigger_.th_dist_to_boundary_m[side_key];
    const auto lat_ratio = (th_dist_to_boundary_m.max - lat_dist_to_bound_m) /
                           (th_dist_to_boundary_m.max - th_dist_to_boundary_m.min);

    const auto th_decel_mps2 = th_trigger_.th_acc_mps2;
    const auto max_decel = std::abs(th_decel_mps2.max);
    const auto min_decel = std::abs(th_decel_mps2.min);
    const auto lon_ratio = (std::abs(expected_decel_mps2) - min_decel) / (max_decel - min_decel);

    const auto combined_ratio = 0.5 * lat_ratio + 0.5 * lon_ratio;

    const auto min_jerk = th_trigger_.th_jerk_mps3.min;
    const auto max_jerk = th_trigger_.th_jerk_mps3.max;
    if (combined_ratio <= 0.0) return min_jerk;
    if (combined_ratio >= 1.0) return max_jerk;

    return autoware::interpolation::lerp(min_jerk, max_jerk, combined_ratio);
  }
  std::shared_ptr<const PlannerData> planner_data_;
  TriggerThreshold th_trigger_;
};

}  // namespace autoware::motion_velocity_planner

#endif  // SLOW_DOWN_INTERPOLATOR_HPP_
