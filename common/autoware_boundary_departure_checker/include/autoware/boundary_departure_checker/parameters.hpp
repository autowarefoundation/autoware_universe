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
 * @brief Parameters for checking uncrossable boundary departure.
 */
struct UncrossableBoundaryDepartureParam
{
  int max_lateral_rtree_queries{5};        ///< maximum number of lateral R-tree queries
  double lateral_margin_m{0.01};           ///< lateral margin [m]
  double longitudinal_margin_m{1.0};       ///< longitudinal margin [m]
  double max_deceleration_mps2{-4.0};      ///< maximum deceleration [m/s^2]
  double max_jerk_mps3{-5.0};              ///< maximum jerk [m/s^3]
  double brake_delay_s{1.0};               ///< brake delay [s]
  double time_to_departure_cutoff_s{2.0};  ///< time to departure cutoff [s]
  double on_time_buffer_s{0.15};           ///< buffer for activating departure detection [s]
  double off_time_buffer_s{0.15};          ///< buffer for deactivating departure detection [s]
  bool enable_developer_marker{true};      ///< flag marker only for developer
  std::vector<std::string> boundary_types_to_detect{"road_border"};  ///< boundary types to detect
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_
