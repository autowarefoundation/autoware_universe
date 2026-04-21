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

#include "autoware/boundary_departure_checker/hysteresis_logic.hpp"

#include "autoware/boundary_departure_checker/severity_evaluator.hpp"

namespace autoware::boundary_departure_checker
{
HysteresisResult update_and_judge(
  const HysteresisState & state,
  const std::optional<Side<std::optional<CriticalPointPair>>> & evaluation_result,
  const UncrossableBoundaryDepartureParam & param, const double current_time_s)
{
  HysteresisResult result;
  result.updated_state = state;
  result.status = DepartureType::NONE;

  if (!evaluation_result.has_value()) {
    result.updated_state.last_no_critical_dpt_time = current_time_s;
    result.updated_state.critical_departure_history.for_each_side(
      [](auto & side) { side.clear(); });
    return result;
  }

  const bool current_is_critical = severity_evaluator::is_critical(*evaluation_result);

  if (current_is_critical) {
    if (current_time_s - state.last_no_critical_dpt_time >= param.on_time_buffer_s) {
      result.updated_state.last_found_critical_dpt_time = current_time_s;
      result.updated_state.critical_departure_history.for_each_side(
        [](auto & side) { side.clear(); });
      evaluation_result->for_each([&](auto key_constant, auto & side_value) {
        if (side_value.has_value() && side_value->safety_buffer_start.is_critical()) {
          result.updated_state.critical_departure_history[key_constant.value].push_back(
            side_value->physical_departure_point);
        }
      });
      result.status = DepartureType::CRITICAL;
      return result;
    }
    return result;
  }

  result.updated_state.last_no_critical_dpt_time = current_time_s;

  if (!state.critical_departure_history.all_empty()) {
    if (current_time_s - state.last_found_critical_dpt_time < param.off_time_buffer_s) {
      result.status = DepartureType::CRITICAL;
      return result;
    }
    result.updated_state.critical_departure_history.for_each_side(
      [](auto & side) { side.clear(); });
  }

  return result;
}

}  // namespace autoware::boundary_departure_checker
