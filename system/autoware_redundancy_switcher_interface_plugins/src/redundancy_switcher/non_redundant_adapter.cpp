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
#include "non_redundant_adapter.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <redundancy_switcher_interface/ir/input_events.hpp>

#include <stdexcept>
#include <string>

namespace autoware::redundancy_switcher
{

void NonRedundantSwitcherAdapter::initialize(
  rclcpp::Node * node, std::shared_ptr<EventGateway> gateway)
{
  if (!node) throw std::invalid_argument("NonRedundantSwitcherAdapter: node is null");
  if (!gateway) throw std::invalid_argument("NonRedundantSwitcherAdapter: gateway is null");

  RCLCPP_INFO(node->get_logger(), "NonRedundantSwitcherAdapter: initializing for non-redundant system");

  // Fix the switcher state as permanently stable — no hardware switcher exists.
  const SwitcherSignals signals{true, false, false};
  gateway->submit(InputEvent{SetSwitcherSignalsEvent{
    Annotated<SwitcherSignals>{signals, "non-redundant system"}}});

  // Publish active_control_unit [0, 2] (main ECU + main VCU) once via transient_local.
  // Processor forwards this as UpdateActiveControlUnitCommand to SubSystemAdapter.
  const ActiveControlUnit acu{{0, 2}};
  gateway->submit(InputEvent{SetActiveControlUnitEvent{
    Annotated<ActiveControlUnit>{acu, "non-redundant system"}}});

  // Register the same per-node/link diagnostic keys as RedundancySwitcherAdapter so that
  // downstream diagnostic graphs do not see STALE entries.
  // All checks are skipped — there is no hardware switcher to evaluate.
  const bool is_main_ecu = node->has_parameter("is_main_ecu")
    ? node->get_parameter("is_main_ecu").as_bool()
    : node->declare_parameter<bool>("is_main_ecu");
  const std::string hardware_id =
    is_main_ecu ? "main_ecu_redundancy_switcher" : "sub_ecu_redundancy_switcher";

  updater_ = std::make_unique<diagnostic_updater::Updater>(node);
  updater_->setHardwareID(hardware_id);
  updater_->add("main_ecu_fault",               &NonRedundantSwitcherAdapter::diag_skipped);
  updater_->add("sub_ecu_fault",                &NonRedundantSwitcherAdapter::diag_skipped);
  updater_->add("main_vcu_fault",               &NonRedundantSwitcherAdapter::diag_skipped);
  updater_->add("sub_vcu_fault",                &NonRedundantSwitcherAdapter::diag_skipped);
  updater_->add("main_ecu_to_sub_ecu_link_fault",  &NonRedundantSwitcherAdapter::diag_skipped);
  updater_->add("main_ecu_to_main_vcu_link_fault", &NonRedundantSwitcherAdapter::diag_skipped);
  updater_->add("main_ecu_to_sub_vcu_link_fault",  &NonRedundantSwitcherAdapter::diag_skipped);
  updater_->add("sub_ecu_to_main_vcu_link_fault",  &NonRedundantSwitcherAdapter::diag_skipped);
  updater_->add("sub_ecu_to_sub_vcu_link_fault",   &NonRedundantSwitcherAdapter::diag_skipped);
  updater_->add("main_vcu_to_sub_vcu_link_fault",  &NonRedundantSwitcherAdapter::diag_skipped);
}

void NonRedundantSwitcherAdapter::execute(const OutputCommand &)
{
  // No hardware switcher to interact with. All commands are intentionally ignored.
}

void NonRedundantSwitcherAdapter::diag_skipped(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.summary(
    diagnostic_msgs::msg::DiagnosticStatus::OK, "Check skipped for non-redundant system");
}

}  // namespace autoware::redundancy_switcher

PLUGINLIB_EXPORT_CLASS(
  autoware::redundancy_switcher::NonRedundantSwitcherAdapter,
  autoware::redundancy_switcher::IAdapterPlugin)
