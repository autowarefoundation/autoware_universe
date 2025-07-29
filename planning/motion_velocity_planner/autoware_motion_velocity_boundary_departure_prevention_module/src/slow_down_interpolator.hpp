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

#ifndef SLOW_DOWN_INTERPOLATOR_HPP_
#define SLOW_DOWN_INTERPOLATOR_HPP_

#include "type_alias.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <tl_expected/expected.hpp>

#include <string>
#include <vector>
namespace autoware::motion_velocity_planner::experimental::utils
{

class SlowDownInterpolator
{
public:
  struct SlowDownPlan
  {
    double rel_dist_m;
    double target_vel_mps;
    double expected_decel;
  };

  enum class Response { COMFORT, HARD };

  explicit SlowDownInterpolator(const TriggerThreshold & th_trigger) : th_trigger_(th_trigger) {}

  /**
   * @brief Calculates a deceleration plan for the ego vehicle toward the boundary.
   *
   * This function computes how far in advance the vehicle should start slowing down,
   * what velocity it should aim for at the slow-down point, and what deceleration value
   * would achieve that, based on the current velocity, lateral and longitudinal distances,
   * and the side of the vehicle approaching the boundary.
   *
   * @param curr_vel Current velocity of the vehicle in meters per second.
   * @param lon_dist_to_bound_m Distance along the trajectory to the boundary point.
   * @param lat_dist_to_bound_m Lateral distance from the ego to the boundary (positive or
   * negative).
   * @param side_key Specifies whether the left or right boundary is used for threshold lookup.
   * @return A tuple containing:
   *         - distance to start slowing down,
   *         - target velocity at the boundary,
   *         - expected deceleration.
   *         Returns an error message if the deceleration exceeds the threshold.
   */
  [[nodiscard]] tl::expected<SlowDownPlan, std::string> get_interp_to_point(
    const double curr_vel, const double curr_acc, const double lon_dist_to_bound_m,
    double lat_dist_to_bound_m, const SideKey side_key) const;

private:
  [[nodiscard]] double interp_velocity(
    const double curr_vel, const double lat_dist, const SideKey side_key) const;

  [[nodiscard]] static double v_t(const double t, const double j, const double a, const double v);

  [[nodiscard]] static double s_t(const double t, const double j, const double a, const double v);

  [[nodiscard]] static double t_j(const double a_0, const double a, const double j);

  [[nodiscard]] static double d_slow(
    const double v_0, const double v_target, const double a_0, const double a_brake,
    const double j_brake);

  static tl::expected<double, std::string> find_reach_time(
    double j, double a, double v0, double dist, double t_min, double t_max);

  [[nodiscard]] std::optional<double> get_comfort_distance(
    const double lon_dist_to_dpt_pt, const double v_0, const double v_target,
    const double a_0) const;

  static tl::expected<double, std::string> calc_velocity_with_profile(
    const double a_0, const double v_0, const double v_target, const double j_brake,
    const double a_brake, const double lon_dist_to_dpt_pt);

  TriggerThreshold th_trigger_;
};

}  // namespace autoware::motion_velocity_planner::experimental::utils

#endif  // SLOW_DOWN_INTERPOLATOR_HPP_
