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

#include <autoware_planning_msgs/srv/set_lateral_offset.hpp>

#include <optional>

namespace autoware::behavior_path_planner
{

/** Optional max magnitude for RAW_VALUE (meters). Use nullopt to skip range check. */
constexpr std::optional<double> DEFAULT_MAX_RAW_SHIFT_MAGNITUDE = 3.0;

/**
 * @brief Example validation for SetLateralOffset service request.
 * @param request SetLateralOffset service request (uses Request::RAW_VALUE, Request::DIRECTION,
 *        Request::RESET, Request::LEFT, Request::RIGHT from the message)
 * @param current_inserted_lateral_offset Current inserted lateral offset [m] (scene's state)
 * @param unit_shift_amount Shift increment in meters for DIRECTION mode (must be >= 0)
 * @param max_raw_shift_magnitude Optional max |shift_value| for RAW_VALUE; nullopt to skip
 * @return Computed lateral offset in meters, or nullopt if validation failed
 */
std::optional<double> validateAndComputeLateralOffset(
  const autoware_planning_msgs::srv::SetLateralOffset::Request & request,
  double current_inserted_lateral_offset, double unit_shift_amount,
  std::optional<double> max_raw_shift_magnitude = DEFAULT_MAX_RAW_SHIFT_MAGNITUDE);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__VALIDATION_HPP_
