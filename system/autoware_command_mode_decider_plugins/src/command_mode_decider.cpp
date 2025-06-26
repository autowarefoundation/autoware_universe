//  Copyright 2025 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "command_mode_decider.hpp"

#include <autoware_command_mode_types/modes.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_system_msgs/srv/change_operation_mode.hpp>

#include <vector>

namespace autoware::command_mode_decider
{

namespace modes = autoware::command_mode_types::modes;
using autoware_adapi_v1_msgs::msg::MrmState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_system_msgs::srv::ChangeOperationMode;

uint16_t CommandModeDecider::from_operation_mode(uint16_t operation_mode)
{
  // clang-format off
  switch (operation_mode) {
    case ChangeOperationMode::Request::STOP:       return modes::stop;
    case ChangeOperationMode::Request::AUTONOMOUS: return modes::autonomous;
    case ChangeOperationMode::Request::LOCAL:      return modes::local;
    case ChangeOperationMode::Request::REMOTE:     return modes::remote;
    default:                                       return modes::unknown;
  }
  // clang-format on
}

uint16_t CommandModeDecider::to_operation_mode(uint16_t command_mode)
{
  // clang-format off
  switch(command_mode) {
    case modes::stop:       return OperationModeState::STOP;
    case modes::autonomous: return OperationModeState::AUTONOMOUS;
    case modes::local:      return OperationModeState::LOCAL;
    case modes::remote:     return OperationModeState::REMOTE;
    default:                return OperationModeState::UNKNOWN;
  }
  // clang-format on
}

uint16_t CommandModeDecider::to_mrm_behavior(uint16_t command_mode)
{
  // clang-format off
  switch(command_mode) {
    case modes::emergency_stop:   return MrmState::EMERGENCY_STOP;
    case modes::comfortable_stop: return MrmState::COMFORTABLE_STOP;
    case modes::pull_over:        return MrmState::PULL_OVER;
    default:                      return MrmState::NONE;
  }
  // clang-format on
}

std::vector<uint16_t> CommandModeDecider::decide(
  const RequestModeStatus & request, const CommandModeStatusTable & table)
{
  const auto create_vector = [](uint16_t mode) {
    std::vector<uint16_t> result;
    result.push_back(mode);
    return result;
  };

  // Use the specified operation mode if available.
  {
    const auto available = table.available(request.operation_mode, !request.autoware_control);
    if (available) {
      return create_vector(request.operation_mode);
    }
  }

  // TODO(Takagi, Isamu): Use the available MRM according to the state transitions.
  // https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/fail-safe/#behavior

  namespace modes = autoware::command_mode_types::modes;

  if (table.available(modes::pull_over, true)) {
    return create_vector(modes::pull_over);
  }
  if (table.available(modes::comfortable_stop, true)) {
    return create_vector(modes::comfortable_stop);
  }
  if (table.available(modes::emergency_stop, true)) {
    return create_vector(modes::emergency_stop);
  }

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "No mrm available");
  return create_vector(modes::unknown);
}

}  // namespace autoware::command_mode_decider

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::command_mode_decider::CommandModeDecider, autoware::command_mode_decider::DeciderPlugin)
