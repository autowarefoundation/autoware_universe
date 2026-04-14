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
struct UncrossableBoundaryDepartureParam
{
  int max_lateral_rtree_queries{5};
  double lateral_margin_m{0.01};
  double longitudinal_margin_m{1.0};
  double max_deceleration_mps2{-4.0};
  double max_jerk_mps3{-5.0};
  double brake_delay_s{1.0};
  double time_to_departure_cutoff_s{2.0};
  double on_time_buffer_s{0.15};
  double off_time_buffer_s{0.15};
  std::vector<std::string> boundary_types_to_detect{"road_border"};
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_
