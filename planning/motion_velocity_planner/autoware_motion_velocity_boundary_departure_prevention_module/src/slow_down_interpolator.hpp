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
    const double curr_vel, const double lon_dist_to_bound_m, double lat_dist_to_bound_m,
    const SideKey side_key) const;

private:
  /**
   * @brief Estimates the deceleration required to reach the target velocity over a given distance.
   *
   * Uses the standard kinematic formula and clamps the result between the minimum and maximum
   * allowed deceleration values.
   *
   * @param curr_vel Current velocity.
   * @param target_vel Desired velocity.
   * @param arclength_to_point_m Distance available for deceleration.
   * @return Estimated deceleration, clamped within configured thresholds.
   */
  [[nodiscard]] double calc_expected_deceleration(
    const double curr_vel, const double target_vel, const double arclength_to_point_m) const;

  /**
   * @brief Calculates the velocity at the slow-down point considering acceleration and jerk.
   *
   * This method simulates velocity change under constant jerk followed by constant acceleration,
   * and clamps the result to ensure it does not drop below the configured minimum velocity.
   *
   * @param curr_vel Current velocity of the ego vehicle.
   * @param a_target Target acceleration after the jerk phase.
   * @param jerk Applied jerk value.
   * @param lon_dist_to_point_m Distance available for deceleration.
   * @return Estimated target velocity at the end of the slow-down interval.
   */
  [[nodiscard]] double calc_new_velocity(
    const double curr_vel, const double a_target, const double jerk,
    const double lon_dist_to_point_m) const;

  [[nodiscard]] double interp_velocity(
    const double curr_vel, const double lat_dist, const SideKey side_key) const;

  [[nodiscard]] std::vector<double> get_lat_dist_axis(const SideKey side_key) const;

  [[nodiscard]] std::vector<double> get_vel_axis() const;

  [[nodiscard]] double interp_jerk(
    const double lat_dist_to_bound_m, const double expected_decel_mps2,
    const SideKey side_key) const;

  TriggerThreshold th_trigger_;
};

}  // namespace autoware::motion_velocity_planner::experimental::utils

#endif  // SLOW_DOWN_INTERPOLATOR_HPP_
