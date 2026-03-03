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

#include "autoware/behavior_path_side_shift_module/validation.hpp"

#include <cmath>

namespace autoware::behavior_path_planner
{

std::optional<double> validateAndComputeLateralOffset(
  const autoware_planning_msgs::srv::SetLateralOffset::Request & request,
  double current_inserted_lateral_offset, double unit_shift_amount,
  std::optional<double> max_raw_shift_magnitude)
{
  using Request = autoware_planning_msgs::srv::SetLateralOffset::Request;

  if (request.shift_mode == Request::RAW_VALUE) {
    if (
      max_raw_shift_magnitude &&
      std::fabs(static_cast<double>(request.shift_value)) > *max_raw_shift_magnitude) {
      return std::nullopt;
    }
    return static_cast<double>(request.shift_value);
  }

  if (request.shift_mode == Request::DIRECTION) {
    if (request.shift_direction_value == Request::RESET) {
      return 0.0;
    }
    if (request.shift_direction_value == Request::LEFT) {
      if (unit_shift_amount < 0.0) {
        return std::nullopt;
      }
      return current_inserted_lateral_offset + unit_shift_amount;
    }
    if (request.shift_direction_value == Request::RIGHT) {
      if (unit_shift_amount < 0.0) {
        return std::nullopt;
      }
      return current_inserted_lateral_offset - unit_shift_amount;
    }
    return std::nullopt;
  }

  return std::nullopt;
}

}  // namespace autoware::behavior_path_planner
