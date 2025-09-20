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

#include "command_mode_switcher.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::command_mode_switcher
{

CommandModeSwitcher::CommandModeSwitcher(const rclcpp::NodeOptions & options)
: Node("command_mode_switcher", options),
  loader_("autoware_command_mode_switcher", "autoware::command_mode_switcher::CommandPlugin"),
  control_gate_interface_(*this, [this]() { update(); }),
  network_gate_interface_(*this, [this]() { update(); }),
  vehicle_gate_interface_(*this, [this]() { update(); })
{
  // ECU ID for multiple devices systems.
  {
    const auto multiple_ecu_enabled = declare_parameter<bool>("multiple_ecu_enabled");
    if (multiple_ecu_enabled) {
      ecu_ = declare_parameter<uint8_t>("multiple_ecu_identifier");
    }
  }

  // Create control gate switcher.
  {
    const auto plugins = declare_parameter<std::vector<std::string>>("plugins");

    for (const auto & plugin : plugins) {
      if (!loader_.isClassAvailable(plugin)) {
        RCLCPP_WARN_STREAM(get_logger(), "ignore unknown plugin: " << plugin);
        continue;
      }
      const auto instance = loader_.createSharedInstance(plugin);
      if (autoware_commands_.count(instance->mode())) {
        RCLCPP_WARN_STREAM(get_logger(), "ignore duplicate plugin: " << plugin);
        continue;
      }

      const auto command = std::make_shared<Command>(instance);
      command->plugin->set_plugin_name(plugin);
      autoware_commands_[command->plugin->mode()] = command;
      commands_.push_back(command);
    }
  }

  // Initialize all switchers. Call "construct" first, which acts as the base class constructor.
  for (const auto & command : commands_) {
    command->plugin->construct(this);
    command->plugin->initialize();
  }

  pub_status_ =
    create_publisher<CommandModeStatus>("~/command_mode/status", rclcpp::QoS(1).transient_local());
  sub_request_ = create_subscription<CommandModeRequest>(
    "~/command_mode/request", rclcpp::QoS(1),
    std::bind(&CommandModeSwitcher::on_request, this, std::placeholders::_1));

  const auto period = rclcpp::Rate(declare_parameter<double>("update_rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { update(); });
}

void CommandModeSwitcher::on_request(const CommandModeRequest & msg)
{
  // clang-format off
  const auto get_vehicle_mode = [](uint8_t mode) {
    switch (mode) {
      case CommandModeRequest::AUTOWARE: return VehicleModeRequest::Autoware;
      case CommandModeRequest::MANUAL:   return VehicleModeRequest::Manual;
      default:                           return VehicleModeRequest::None;
    }
  };
  // clang-format on

  // Use the first command mode that this switcher supports.
  std::shared_ptr<Command> command_mode = nullptr;
  for (const auto & item : msg.items) {
    const auto iter = autoware_commands_.find(item.mode);
    if (iter != autoware_commands_.end()) {
      command_mode = iter->second;
      break;
    }
  }
  request_command_mode(command_mode);
  request_vehicle_mode(get_vehicle_mode(msg.vehicle));
  update();
}

void CommandModeSwitcher::request_command_mode(std::shared_ptr<Command> command_mode)
{
  if (!command_mode) {
    return;
  }
  if (command_mode_request_ == command_mode) {
    return;
  }
  command_mode_request_ = command_mode;

  // Start new transition of the target command mode.
  for (const auto & command : commands_) {
    const auto is_request_mode = (command_mode_request_ == command);
    command->status.request = is_request_mode;
    command->status.transition = is_request_mode;
  }
  const auto mode_text = std::to_string(command_mode_request_->plugin->mode());
  RCLCPP_INFO_STREAM(get_logger(), "command mode accepted: " << mode_text);
}

void CommandModeSwitcher::request_vehicle_mode(VehicleModeRequest vehicle_mode)
{
  if (vehicle_mode == VehicleModeRequest::None) {
    return;
  }
  if (vehicle_mode_request_ == vehicle_mode) {
    return;
  }
  vehicle_mode_request_ = vehicle_mode;

  // No need to restart the transition if the vehicle mode is manual.
  if (vehicle_mode_request_ == VehicleModeRequest::Manual) {
    RCLCPP_INFO_STREAM(get_logger(), "vehicle mode accepted: manual");
    return;
  }

  // Restart the transition of the current command mode.
  if (vehicle_mode_request_ == VehicleModeRequest::Autoware) {
    RCLCPP_INFO_STREAM(get_logger(), "vehicle mode accepted: autoware");
    if (command_mode_request_) {
      command_mode_request_->status.request = true;
      command_mode_request_->status.transition = true;
    }
    return;
  }

  RCLCPP_ERROR_STREAM(get_logger(), "invalid vehicle mode");
}

void CommandModeSwitcher::update()
{
  // TODO(Takagi, Isamu): Check call rate.
  detect_override();
  update_status();
  change_modes();
  publish_command_mode_status();
}

void CommandModeSwitcher::detect_override()
{
  const auto curr_manual_control = vehicle_gate_interface_.is_manual_control();
  if (!prev_manual_control_ && curr_manual_control) {
    vehicle_mode_request_ = VehicleModeRequest::None;
    RCLCPP_WARN_STREAM(get_logger(), "override detected");
  }
  prev_manual_control_ = curr_manual_control;
}

void CommandModeSwitcher::update_status()
{
  // NOTE: Update the command enable/disable first since the exclusive depends on.
  for (const auto & command : commands_) {
    auto & status = command->status;
    auto & plugin = command->plugin;
    const auto state = plugin->update_source_state(status.request);
    status.command_enabled = state.enabled;
    status.command_disabled = state.disabled;
  }

  // Within the source group, all sources except for the active must be disabled.
  struct GroupCount
  {
    int total = 0;
    int disabled = 0;
  };
  std::unordered_map<uint16_t, GroupCount> group_count;
  for (const auto & command : commands_) {
    const auto source = command->plugin->source();
    group_count[source].total += 1;
    group_count[source].disabled += command->status.command_disabled ? 1 : 0;
  }

  const auto to_message = [](const MrmState & state) {
    // clang-format off
    using Message = tier4_system_msgs::msg::CommandModeStatusItem;
    switch (state) {
      case MrmState::Normal:     return Message::NORMAL;
      case MrmState::Operating:  return Message::OPERATING;
      case MrmState::Succeeded:  return Message::SUCCEEDED;
      case MrmState::Failed:     return Message::FAILED;
      default:                   return Message::UNDEFINED;
    }
    // clang-format on
  };
  for (const auto & command : commands_) {
    auto & status = command->status;
    auto & plugin = command->plugin;
    const auto count = group_count[plugin->source()];
    status.command_exclusive = status.command_enabled && (count.total <= count.disabled + 1);
    status.command_selected = control_gate_interface_.is_selected(*plugin);
    status.network_selected = network_gate_interface_.is_selected(ecu_);
    status.vehicle_selected = vehicle_gate_interface_.is_selected(*plugin);
    status.mrm = to_message(plugin->update_mrm_state());
    status.transition_completed = plugin->get_transition_completed();
  }
}

void CommandModeSwitcher::publish_command_mode_status()
{
  CommandModeStatus msg;
  msg.stamp = now();
  for (const auto & command : commands_) {
    msg.items.push_back(command->status);
  }
  pub_status_->publish(msg);
}

void CommandModeSwitcher::change_modes()
{
  // Wait if mode change request is in progress.
  if (control_gate_interface_.is_requesting() || vehicle_gate_interface_.is_requesting()) {
    return;
  }

  if (!command_mode_request_) {
    return;
  }

  if (vehicle_mode_request_ == VehicleModeRequest::Manual) {
    return change_vehicle_mode_to_manual();
  }

  if (!command_mode_request_->status.command_selected) {
    return change_command_mode();
  }

  if (vehicle_mode_request_ == VehicleModeRequest::Autoware) {
    return change_vehicle_mode_to_autoware();
  }

  if (vehicle_gate_interface_.is_autoware_control()) {
    if (!command_mode_request_->status.transition_completed) {
      return;
    }
  }

  if (control_gate_interface_.is_in_transition()) {
    control_gate_interface_.request(*command_mode_request_->plugin, false);
    return;
  }

  command_mode_request_->status.transition = false;
}

void CommandModeSwitcher::change_vehicle_mode_to_manual()
{
  vehicle_mode_request_ = VehicleModeRequest::None;
  vehicle_gate_interface_.request_manual_control();
}

void CommandModeSwitcher::change_vehicle_mode_to_autoware()
{
  vehicle_mode_request_ = VehicleModeRequest::None;
  vehicle_gate_interface_.request_autoware_control();
}

void CommandModeSwitcher::change_command_mode()
{
  // Select control gate. Enable transition filter if autoware control is or will be enabled.
  const auto curr_autoware_control = vehicle_gate_interface_.is_autoware_control();
  const auto next_autoware_control = vehicle_mode_request_ == VehicleModeRequest::Autoware;
  const auto filter = curr_autoware_control || next_autoware_control;
  control_gate_interface_.request(*command_mode_request_->plugin, filter);
}

}  // namespace autoware::command_mode_switcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::command_mode_switcher::CommandModeSwitcher)
