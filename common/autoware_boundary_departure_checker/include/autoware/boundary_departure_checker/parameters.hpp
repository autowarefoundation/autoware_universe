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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_

#include <string>
#include <vector>

namespace autoware::boundary_departure_checker
{
/**
 * @brief Parameters for the uncrossable boundary departure checker.
 */
struct UncrossableBoundaryDepartureParam
{
  // Member Variables
  /**
   * @brief Maximum number of queries to the R-tree for lateral distance calculation.
   *
   * Limits the computational load when searching for nearby boundary segments.
   */
  int max_lateral_rtree_queries{5};

  /**
   * @brief Lateral margin for boundary departure check [m].
   *
   * The safety distance maintained laterally from the boundary.
   */
  double lateral_margin_m{0.01};

  /**
   * @brief Longitudinal margin for boundary departure check [m].
   *
   * The safety distance maintained longitudinally along the path.
   */
  double longitudinal_margin_m{1.0};

  /**
   * @brief Maximum deceleration for braking [m/s^2].
   *
   * Used to calculate the distance required to stop the vehicle.
   */
  double max_deceleration_mps2{-4.0};

  /**
   * @brief Maximum jerk for braking [m/s^3].
   *
   * Used to calculate the rate of change of deceleration for smooth braking.
   */
  double max_jerk_mps3{-5.0};

  /**
   * @brief Time delay until the brake is applied [s].
   *
   * Accounts for system latency and mechanical response time.
   */
  double brake_delay_s{1.0};

  /**
   * @brief Cutoff time for time-to-departure calculation [s].
   *
   * Predictions beyond this threshold are ignored to avoid noise.
   */
  double time_to_departure_cutoff_s{2.0};

  /**
   * @brief Hysteresis buffer time to trigger "trigger" critical departure state [s].
   *
   * The duration a departure condition must persist before being activated.
   */
  double on_time_buffer_s{0.15};

  /**
   * @brief Hysteresis buffer time to trigger "clear" critical departure state [s].
   *
   * The duration a departure condition must be absent before being deactivated.
   */
  double off_time_buffer_s{0.15};

  /**
   * @brief List of boundary types to be detected (e.g., "road_border").
   *
   * Filter for specific lanelet map linestring tags that should be considered uncrossable.
   */
  std::vector<std::string> boundary_types_to_detect{"road_border"};
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_
