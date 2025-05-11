// Copyright 2025 The Autoware Contributors
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

#include "mode_interface.hpp"

namespace autoware::hazard_status_converter
{

ModeInterface::ModeInterface(rclcpp::Node & node) : node_(node)
{
  sub_operation_mode_state_ = node.create_subscription<OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS(1).transient_local(),
    [this](const OperationModeState & msg) { operation_mode_state_ = msg; });
  sub_autoware_state_ = node.create_subscription<AutowareState>(
    "/autoware/state", rclcpp::QoS(1),
    [this](const AutowareState & msg) { autoware_state_ = msg; });
}

RootModeStatus ModeInterface::get_root() const
{
  const auto is_not_autoware_running = [this]() {
    if (autoware_state_->state == AutowareState::INITIALIZING) return true;
    if (autoware_state_->state == AutowareState::FINALIZING) return true;
    return false;
  };

  const auto is_not_autonomous_ready = [this]() {
    if (autoware_state_->state == AutowareState::WAITING_FOR_ROUTE) return true;
    if (autoware_state_->state == AutowareState::PLANNING) return true;
    return false;
  };

  // Returns unknown before receiving state messages.
  if (!operation_mode_state_ || !autoware_state_) {
    return {RootModeStatus::Mode::UNKNOWN, false};
  }

  // During stop mode refer to autonomous mode diagnostics.
  if (operation_mode_state_->mode == OperationModeState::STOP) {
    return {RootModeStatus::Mode::AUTO, is_not_autoware_running() || is_not_autonomous_ready()};
  }
  if (operation_mode_state_->mode == OperationModeState::AUTONOMOUS) {
    return {RootModeStatus::Mode::AUTO, is_not_autoware_running() || is_not_autonomous_ready()};
  }
  if (operation_mode_state_->mode == OperationModeState::REMOTE) {
    return {RootModeStatus::Mode::REMOTE, is_not_autoware_running()};
  }
  if (operation_mode_state_->mode == OperationModeState::LOCAL) {
    return {RootModeStatus::Mode::LOCAL, is_not_autoware_running()};
  }
  return {RootModeStatus::Mode::UNKNOWN, false};
}

}  // namespace autoware::hazard_status_converter
