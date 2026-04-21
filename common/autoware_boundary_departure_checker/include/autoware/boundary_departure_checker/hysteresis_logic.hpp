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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__HYSTERESIS_LOGIC_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__HYSTERESIS_LOGIC_HPP_

#include "autoware/boundary_departure_checker/data_structs.hpp"
#include "autoware/boundary_departure_checker/parameters.hpp"

#include <optional>

namespace autoware::boundary_departure_checker
{
/**
 * @brief State for hysteresis logic.
 */
struct HysteresisState
{
  double last_no_critical_dpt_time{0.0};     ///< last time no critical departure was found [s]
  double last_found_critical_dpt_time{0.0};  ///< last time critical departure was found [s]
  Side<ProjectionsToBound> critical_departure_history;  ///< history of critical departures
};

/**
 * @brief Result of hysteresis logic.
 */
struct HysteresisResult
{
  DepartureType status{DepartureType::NONE};
  HysteresisState updated_state;
};

/**
 * @brief Pure function to determine departure status using hysteresis.
 * @param[in] state current hysteresis state
 * @param[in] evaluation_result result from stateless evaluator
 * @param[in] param parameters
 * @param[in] current_time_s current time in seconds
 * @return updated status and state
 */
HysteresisResult update_and_judge(
  const HysteresisState & state,
  const std::optional<Side<std::optional<CriticalPointPair>>> & evaluation_result,
  const UncrossableBoundaryDepartureParam & param, const double current_time_s);

}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__HYSTERESIS_LOGIC_HPP_
