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
  uint8_t shift_mode, float shift_value, uint8_t shift_direction_value,
  double direction_shift_amount, std::optional<double> max_raw_shift_magnitude)
{
  if (shift_mode == SHIFT_MODE_RAW_VALUE) {
    if (max_raw_shift_magnitude && std::fabs(static_cast<double>(shift_value)) > *max_raw_shift_magnitude) {
      return std::nullopt;
    }
    return static_cast<double>(shift_value);
  }

  if (shift_mode == SHIFT_MODE_DIRECTION) {
    if (shift_direction_value == SHIFT_DIRECTION_RESET) {
      return 0.0;
    }
    if (shift_direction_value == SHIFT_DIRECTION_LEFT) {
      if (direction_shift_amount < 0.0) {
        return std::nullopt;
      }
      return direction_shift_amount;
    }
    if (shift_direction_value == SHIFT_DIRECTION_RIGHT) {
      if (direction_shift_amount < 0.0) {
        return std::nullopt;
      }
      return -direction_shift_amount;
    }
    return std::nullopt;
  }

  return std::nullopt;
}

}  // namespace autoware::behavior_path_planner
