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

#include "redundancy_command_mode_decider.hpp"

#include <autoware_command_mode_types/modes.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_system_msgs/srv/change_operation_mode.hpp>

#include <memory>
#include <vector>

namespace autoware::command_mode_decider
{

namespace modes = autoware::command_mode_types::modes;
using autoware_adapi_v1_msgs::msg::MrmState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_system_msgs::srv::ChangeOperationMode;

void RedundancyCommandModeDecider::initialize()
{
  sub_active_control_unit_ =
    std::make_unique<autoware_utils_rclcpp::InterProcessPollingSubscriber<ActiveControlUnit>>(
      node_, "/system/redundancy/active_control_unit", rclcpp::QoS(1).transient_local());
  main_ecu_id_ = static_cast<uint8_t>(node_->declare_parameter<int>("main_ecu_id"));
  sub_ecu_id_ = static_cast<uint8_t>(node_->declare_parameter<int>("sub_ecu_id"));
}

uint16_t RedundancyCommandModeDecider::from_operation_mode(uint16_t operation_mode)
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

uint16_t RedundancyCommandModeDecider::to_operation_mode(uint16_t command_mode)
{
  // clang-format off
  switch (command_mode) {
    case modes::stop:       return OperationModeState::STOP;
    case modes::autonomous: return OperationModeState::AUTONOMOUS;
    case modes::local:      return OperationModeState::LOCAL;
    case modes::remote:     return OperationModeState::REMOTE;
    default:                return OperationModeState::UNKNOWN;
  }
  // clang-format on
}

uint16_t RedundancyCommandModeDecider::to_mrm_behavior(uint16_t command_mode)
{
  // clang-format off
  switch (command_mode) {
    case modes::main_ecu_in_lane_emergency_stop: return 12;
    case modes::comfortable_stop:                return MrmState::COMFORTABLE_STOP;
    case modes::main_ecu_in_lane_moderate_stop:  return 11;
    case modes::sub_ecu_standby:                 return MrmState::NONE;
    case modes::sub_ecu_in_lane_moderate_stop:   return 21;
    default:                                     return MrmState::NONE;
  }
  // clang-format on
}

std::vector<uint16_t> RedundancyCommandModeDecider::decide(
  const RequestModeStatus & request, const CommandModeStatusTable & table)
{
  const auto modes = decide(request, table, last_modes_);
  last_modes_ = modes;
  return modes;
}

std::vector<uint16_t> RedundancyCommandModeDecider::decide(
  const RequestModeStatus & request, const CommandModeStatusTable & table,
  const std::vector<uint16_t> & last_modes)
{
  const auto create_vector = [](uint16_t mode1, uint16_t mode2) {
    std::vector<uint16_t> result;
    result.push_back(mode1);
    result.push_back(mode2);
    return result;
  };

  // Use the specified operation mode if available.
  {
    const auto mode = request.operation_mode;
    const auto same = std::find(last_modes.begin(), last_modes.end(), mode) != last_modes.end();
    const auto available = table.available(mode, !request.autoware_control || same);
    if (available) {
      // When switching from main ECU control to sub ECU control, keep sub_in_lane_moderate_stop.
      if (is_sub_ecu_control(last_modes)) {
        return create_vector(
          modes::sub_ecu_in_lane_moderate_stop, modes::main_ecu_in_lane_moderate_stop);
      }
      return create_vector(mode, modes::sub_ecu_standby);
    }
  }

  if (table.available(modes::comfortable_stop, true)) {
    return create_vector(modes::comfortable_stop, modes::sub_ecu_standby);
  }
  if (table.available(modes::main_ecu_in_lane_moderate_stop, true)) {
    return create_vector(modes::main_ecu_in_lane_moderate_stop, modes::sub_ecu_standby);
  }
  if (table.available(modes::main_ecu_in_lane_emergency_stop, true)) {
    return create_vector(modes::main_ecu_in_lane_emergency_stop, modes::sub_ecu_standby);
  }
  if (table.available(modes::sub_ecu_in_lane_moderate_stop, true)) {
    return create_vector(
      modes::sub_ecu_in_lane_moderate_stop, modes::main_ecu_in_lane_moderate_stop);
  }

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "No mrm available");
  return create_vector(modes::main_ecu_in_lane_moderate_stop, modes::sub_ecu_in_lane_moderate_stop);
}

bool RedundancyCommandModeDecider::is_sub_ecu_control(
  const std::vector<uint16_t> & last_modes) const
{
  const bool is_sub_ecu_active = [&]() {
    const auto active_control_unit_ptr = sub_active_control_unit_->take_data();
    if (!active_control_unit_ptr) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000, "Not receiving active control unit");
      return false;
    }
    return std::find(
             active_control_unit_ptr->ids.begin(), active_control_unit_ptr->ids.end(),
             main_ecu_id_) == active_control_unit_ptr->ids.end();
  }();

  if (last_modes.size() < 1) {
    return false;
  }

  if (last_modes[0] == modes::sub_ecu_in_lane_moderate_stop && is_sub_ecu_active) {
    return true;
  }
  return false;
}

}  // namespace autoware::command_mode_decider

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::command_mode_decider::RedundancyCommandModeDecider,
  autoware::command_mode_decider::DeciderPlugin)
