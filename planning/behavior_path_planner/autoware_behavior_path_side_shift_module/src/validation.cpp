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

#include "autoware/behavior_path_side_shift_module/manager.hpp"

#include <autoware_common_msgs/msg/response_status.hpp>

#include <cmath>
#include <memory>
#include <utility>

namespace autoware::behavior_path_planner
{
using ResponseStatus = autoware_common_msgs::msg::ResponseStatus;

std::pair<uint16_t, double> validateAndComputeLateralOffset(
  const SetLateralOffset::Request & request, const double current_inserted_lateral_offset,
  const std::shared_ptr<SideShiftParameters> & parameters)
{
  if (request.shift_mode == SetLateralOffset::Request::EXPLICIT_LATERAL_OFFSET_AMOUNT) {
    return validateRawValue(
      static_cast<double>(request.shift_value), current_inserted_lateral_offset, parameters);
  }

  if (request.shift_mode == SetLateralOffset::Request::LATERAL_OFFSET_DIRECTION) {
    if (request.shift_direction_value == SetLateralOffset::Request::RESET) {
      return {SetLateralOffset::Response::SUCCESS, 0.0};
    }

    if (parameters->unit_shift_amount <= 0.0) {
      return {ResponseStatus::PARAMETER_ERROR, 0.0};
    }

    if (request.shift_direction_value == SetLateralOffset::Request::LEFT) {
      return validateShiftLeft(current_inserted_lateral_offset, parameters);
    }

    if (request.shift_direction_value == SetLateralOffset::Request::RIGHT) {
      return validateShiftRight(current_inserted_lateral_offset, parameters);
    }

    return {SetLateralOffset::Response::ERROR_INVALID_DIRECTION, 0.0};
  }

  return {SetLateralOffset::Response::ERROR_INVALID_MODE, 0.0};
}

std::pair<uint16_t, double> validateRawValue(
  const double lateral_offset, const double current_inserted_lateral_offset,
  const std::shared_ptr<SideShiftParameters> & parameters)
{
  if (std::fabs(lateral_offset) > parameters->max_shift_magnitude) {
    return {SetLateralOffset::Response::ERROR_EXCEEDED_LIMIT, current_inserted_lateral_offset};
  }

  if (std::fabs(lateral_offset - current_inserted_lateral_offset) < parameters->min_shift_gap) {
    return {SetLateralOffset::Response::ERROR_SHIFT_GAP_TOO_SMALL, current_inserted_lateral_offset};
  }

  return {SetLateralOffset::Response::SUCCESS, lateral_offset};
}

std::pair<uint16_t, double> validateShiftLeft(
  const double current_inserted_lateral_offset,
  const std::shared_ptr<SideShiftParameters> & parameters)
{
  if (current_inserted_lateral_offset >= parameters->max_shift_magnitude) {
    return {SetLateralOffset::Response::ERROR_EXCEEDED_LIMIT, parameters->max_shift_magnitude};
  }

  const double request_offset = current_inserted_lateral_offset + parameters->unit_shift_amount;
  if (std::fabs(request_offset) >= parameters->max_shift_magnitude) {
    return {SetLateralOffset::Response::WARN_EXCEEDED_LIMIT, parameters->max_shift_magnitude};
  }

  if (std::fabs(request_offset - current_inserted_lateral_offset) < parameters->min_shift_gap) {
    return {SetLateralOffset::Response::ERROR_SHIFT_GAP_TOO_SMALL, current_inserted_lateral_offset};
  }

  return {SetLateralOffset::Response::SUCCESS, request_offset};
}

std::pair<uint16_t, double> validateShiftRight(
  const double current_inserted_lateral_offset,
  const std::shared_ptr<SideShiftParameters> & parameters)
{
  if (current_inserted_lateral_offset <= -parameters->max_shift_magnitude) {
    return {SetLateralOffset::Response::ERROR_EXCEEDED_LIMIT, -parameters->max_shift_magnitude};
  }

  const double request_offset = current_inserted_lateral_offset - parameters->unit_shift_amount;
  if (std::fabs(request_offset) >= parameters->max_shift_magnitude) {
    return {SetLateralOffset::Response::WARN_EXCEEDED_LIMIT, -parameters->max_shift_magnitude};
  }

  if (std::fabs(request_offset - current_inserted_lateral_offset) < parameters->min_shift_gap) {
    return {SetLateralOffset::Response::ERROR_SHIFT_GAP_TOO_SMALL, current_inserted_lateral_offset};
  }

  return {SetLateralOffset::Response::SUCCESS, request_offset};
}

const char * getStatusMessage(uint16_t status_code)
{
  switch (status_code) {
    case SetLateralOffset::Response::UNKNOWN:
      return "Unknown or uninitialized status";
    case SetLateralOffset::Response::SUCCESS:
      return "Updated successfully";
    case SetLateralOffset::Response::ERROR_UNKNOWN:
      return "Unknown error occurred";
    case SetLateralOffset::Response::ERROR_INVALID_MODE:
      return "Invalid mode have been requested";
    case SetLateralOffset::Response::ERROR_INVALID_DIRECTION:
      return "Invalid direction have been requested";
    case SetLateralOffset::Response::ERROR_EXCEEDED_LIMIT:
      return "The vehicle cannot shift further than request";
    case SetLateralOffset::Response::ERROR_SHIFT_GAP_TOO_SMALL:
      return "The requested shift length differs too small from the current shift length";
    case SetLateralOffset::Response::ERROR_MODULES_CONFLICTING:
      return "Other modules are running in the behavior_path_planner";
    case SetLateralOffset::Response::WARN_UNKNOWN:
      return "Unknown warning occurred";
    case SetLateralOffset::Response::WARN_EXCEEDED_LIMIT:
      return "The shift length reached limit. Shifted to the possible end.";
    case ResponseStatus::UNKNOWN:
      return "Unknown response status";
    case ResponseStatus::SERVICE_UNREADY:
      return "The side_shift module is not ready";
    case ResponseStatus::PARAMETER_ERROR:
      return "Invalid parameters defined in the side_shift module";
    default:
      return "Unknown response_status";
  }
}

}  // namespace autoware::behavior_path_planner
