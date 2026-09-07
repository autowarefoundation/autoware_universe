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
#include "redundancy_switcher_interface.hpp"

#include "command_mode_subsystem_adapter.hpp"
#include "driving_mode_subsystem_adapter.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <memory>
#include <stdexcept>
#include <string>

namespace autoware::redundancy_switcher
{

RedundancySwitcherInterface::RedundancySwitcherInterface(const rclcpp::NodeOptions & options)
: Node("redundancy_switcher_interface", options),
  plugin_loader_(
    "autoware_redundancy_switcher_interface", "autoware::redundancy_switcher::IAdapterPlugin")
{
  processor_ = std::make_shared<Processor>();
  command_bus_ = std::make_shared<CommandBus>();
  gateway_ = std::make_shared<EventGateway>(processor_, command_bus_);

  log_adapter_ = std::make_shared<LogAdapter>();
  command_bus_->add_handler(log_adapter_);
  log_adapter_->initialize(this, gateway_);

  diag_adapter_ = std::make_shared<DiagAdapter>();
  command_bus_->add_handler(diag_adapter_);
  diag_adapter_->initialize(this, gateway_);

  const bool use_driving_mode = this->declare_parameter<bool>("use_driving_mode", false);
  if (use_driving_mode) {
    subsystem_adapter_ = std::make_shared<DrivingModeSubSystemAdapter>();
  } else {
    subsystem_adapter_ = std::make_shared<CommandModeSubSystemAdapter>();
  }
  command_bus_->add_handler(subsystem_adapter_);
  subsystem_adapter_->initialize(this, gateway_);

  this->declare_parameter("switcher_plugin", rclcpp::ParameterType::PARAMETER_STRING);
  const auto switcher_plugin_name = this->get_parameter("switcher_plugin").as_string();
  if (switcher_plugin_name.empty()) {
    throw std::runtime_error(
      "switcher_plugin is not set. Specify a valid IAdapterPlugin class name "
      "(e.g., autoware::redundancy_switcher::NonRedundantSwitcherAdapter).");
  }
  try {
    switcher_plugin_ = plugin_loader_.createSharedInstance(switcher_plugin_name);
  } catch (const pluginlib::PluginlibException & e) {
    throw std::runtime_error(
      "Failed to load switcher_plugin '" + switcher_plugin_name + "': " + e.what());
  }
  command_bus_->add_handler(switcher_plugin_);
  switcher_plugin_->initialize(this, gateway_);
}

RedundancySwitcherInterface::~RedundancySwitcherInterface() = default;

}  // namespace autoware::redundancy_switcher

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::redundancy_switcher::RedundancySwitcherInterface)
