// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__VALIDATION_HPP_
#define AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__VALIDATION_HPP_

#include "autoware/behavior_path_side_shift_module/manager.hpp"

#include <tier4_planning_msgs/srv/detail/set_lateral_offset__struct.hpp>
#include <tier4_planning_msgs/srv/set_lateral_offset.hpp>

#include <optional>
#include <utility>

namespace autoware::behavior_path_planner
{
using SetLateralOffset = tier4_planning_msgs::srv::SetLateralOffset;

/**
 * @brief Example validation for SetLateralOffset service request.
 * @param request SetLateralOffset service request (uses Request::RAW_VALUE, Request::DIRECTION,
 *        Request::RESET, Request::LEFT, Request::RIGHT from the message)
 * @param current_inserted_lateral_offset Current inserted lateral offset [m] (scene's state)
 * @param unit_shift_amount Shift increment in meters for DIRECTION mode (must be >= 0)
 * @param max_raw_shift_magnitude Optional max |shift_value| for RAW_VALUE; nullopt to skip
 * @return Computed lateral offset in meters, and its status_code reflecting the validation results
 */
std::pair<uint16_t, double> validateAndComputeLateralOffset(
  const SetLateralOffset::Request & request, double current_inserted_lateral_offset,
  double unit_shift_amount, double max_shift_magnitude, double min_shift_gap);

/**
 * @brief Get the status message identical to te status code
 *
 * @param status_code
 * @return const char* status message
 */
const char * getStatusMessage(uint16_t status_code);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__VALIDATION_HPP_
