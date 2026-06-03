//  Copyright 2025 The Autoware Contributors
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
#include "diag_adapter.hpp"

#include "diag_logic.hpp"

#include <redundancy_switcher_interface/detail/overloaded.hpp>
#include <redundancy_switcher_interface/plugin/event_gateway.hpp>

#include <stdexcept>
#include <string>

namespace autoware::redundancy_switcher
{

void DiagAdapter::initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway)
{
  if (!node) throw std::invalid_argument("DiagAdapter: node is null");

  node_ = node;
  gateway_ = gateway;
  transitional_timeout_milli_ = node->declare_parameter<double>("diag.transitional_timeout_milli");

  const bool is_main_ecu = node_->has_parameter("is_main_ecu")
                             ? node_->get_parameter("is_main_ecu").as_bool()
                             : node_->declare_parameter<bool>("is_main_ecu");
  const std::string hardware_id = is_main_ecu ? "main_ecu_redundancy_switcher_interface"
                                              : "sub_ecu_redundancy_switcher_interface";
  updater_ = std::make_unique<diagnostic_updater::Updater>(node);
  updater_->setHardwareID(hardware_id);
  updater_->add("redundancy_switcher_interface_status", this, &DiagAdapter::update_status);
}

void DiagAdapter::execute(const OutputCommand & command)
{
  std::visit(
    overloaded{
      [this](const UpdateStatusDiagCommand &) {
        std::lock_guard<std::mutex> lock(updater_mutex_);
        updater_->force_update();
      },
      [](const auto &) { /* Not this adapter's responsibility — ignore. */ }},
    command);
}

void DiagAdapter::update_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  if (!gateway_) {
    stat.summary(DiagStatus::ERROR, "Internal error: EventGateway is not connected");
    return;
  }
  const auto snap = gateway_->snapshot();

  const double now_ms = node_->now().nanoseconds() / 1e6;
  auto switcher_level = DiagStatus::OK;
  std::string switcher_msg;

  {
    std::lock_guard<std::mutex> lock(transition_mutex_);
    const auto result = compute_switcher_level(
      snap.switcher, now_ms, transitional_start_ms_, transitional_timeout_milli_);
    switcher_level = static_cast<uint8_t>(result.level);
    switcher_msg = result.message;
    transitional_start_ms_ = result.transitional_start_ms;
  }

  auto autoware_level = DiagStatus::OK;
  std::string autoware_msg;
  if (!snap.autoware_ready.has_value()) {
    autoware_level = DiagStatus::WARN;
    autoware_msg = "Startup not yet complete: awaiting Autoware ready signal (WARN)";
  } else {
    switch (snap.autoware_ready->value) {
      case AutowareReady::False:
        autoware_level = DiagStatus::OK;
        autoware_msg = "Autoware is not ready (OK)";
        break;
      case AutowareReady::True:
        autoware_level = DiagStatus::OK;
        autoware_msg = "Autoware is ready (OK)";
        break;
    }
  }

  auto velocity_level = DiagStatus::OK;
  std::string velocity_msg;
  if (!snap.velocity_status.has_value()) {
    velocity_level = DiagStatus::WARN;
    velocity_msg = "Startup not yet complete: awaiting velocity data (WARN)";
  } else {
    switch (snap.velocity_status->value) {
      case VelocityStatus::Stopped:
        velocity_level = DiagStatus::OK;
        velocity_msg = "Vehicle is stopped (OK)";
        break;
      case VelocityStatus::Moving:
        velocity_level = DiagStatus::OK;
        velocity_msg = "Vehicle is moving (OK)";
        break;
    }
  }

  auto control_level = DiagStatus::OK;
  std::string control_msg;
  if (!snap.control_mode.has_value()) {
    control_level = DiagStatus::WARN;
    control_msg = "Startup not yet complete: awaiting control mode data (WARN)";
  } else {
    switch (snap.control_mode->value) {
      case ControlMode::Manual:
        control_level = DiagStatus::OK;
        control_msg = "Manual control mode (OK)";
        break;
      case ControlMode::Auto:
        control_level = DiagStatus::OK;
        control_msg = "Autoware control mode (OK)";
        break;
    }
  }

  stat.add("switcher_signals", switcher_msg);
  stat.add("autoware_ready", autoware_msg);
  stat.add("velocity_status", velocity_msg);
  stat.add("control_mode", control_msg);

  const auto worst = std::max({switcher_level, autoware_level, velocity_level, control_level});
  if (worst == DiagStatus::OK) {
    stat.summary(DiagStatus::OK, "Redundancy switcher interface is running normally");
  } else if (worst == DiagStatus::WARN) {
    stat.summary(DiagStatus::WARN, "Redundancy switcher interface has warnings");
  } else if (worst == DiagStatus::ERROR) {
    stat.summary(DiagStatus::ERROR, "Redundancy switcher interface has errors");
  } else {
    stat.summary(DiagStatus::STALE, "Redundancy switcher interface data is stale");
  }
}

}  // namespace autoware::redundancy_switcher
