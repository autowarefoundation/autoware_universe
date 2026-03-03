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

#include <optional>

namespace autoware::behavior_path_planner
{

/** Constants matching SetLateralOffset.srv */
constexpr uint8_t SHIFT_MODE_RAW_VALUE = 1;
constexpr uint8_t SHIFT_MODE_DIRECTION = 2;
constexpr uint8_t SHIFT_DIRECTION_RESET = 0;
constexpr uint8_t SHIFT_DIRECTION_LEFT = 1;
constexpr uint8_t SHIFT_DIRECTION_RIGHT = 2;

/** Optional max magnitude for RAW_VALUE (meters). Use nullopt to skip range check. */
constexpr std::optional<double> DEFAULT_MAX_RAW_SHIFT_MAGNITUDE = 3.0;

/**
 * @brief Example validation for SetLateralOffset service request.
 * @param shift_mode RAW_VALUE (1) or DIRECTION (2)
 * @param shift_value Used when shift_mode is RAW_VALUE (meters; positive = left)
 * @param shift_direction_value RESET (0), LEFT (1), RIGHT (2) when shift_mode is DIRECTION
 * @param direction_shift_amount Shift amount in meters for DIRECTION mode (must be >= 0)
 * @param max_raw_shift_magnitude Optional max |shift_value| for RAW_VALUE; nullopt to skip
 * @return Computed lateral offset in meters, or nullopt if validation failed
 */
std::optional<double> validateAndComputeLateralOffset(
  uint8_t shift_mode, float shift_value, uint8_t shift_direction_value,
  double direction_shift_amount,
  std::optional<double> max_raw_shift_magnitude = DEFAULT_MAX_RAW_SHIFT_MAGNITUDE);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__VALIDATION_HPP_
