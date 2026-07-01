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
#ifndef REDUNDANCY_SWITCHER_INTERFACE_HPP_
#define REDUNDANCY_SWITCHER_INTERFACE_HPP_

#include "diag_adapter.hpp"
#include "log_adapter.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "redundancy_switcher_interface/core_logic/processor.hpp"
#include "redundancy_switcher_interface/ir/input_events.hpp"
#include "redundancy_switcher_interface/plugin/command_bus.hpp"
#include "redundancy_switcher_interface/plugin/event_gateway.hpp"
#include "redundancy_switcher_interface/plugin/i_adapter_plugin.hpp"

#include <memory>
#include <vector>

namespace autoware::redundancy_switcher
{

class RedundancySwitcherInterface : public rclcpp::Node
{
public:
  explicit RedundancySwitcherInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RedundancySwitcherInterface() override;

private:
  std::shared_ptr<Processor> processor_;
  std::shared_ptr<CommandBus> command_bus_;
  std::shared_ptr<EventGateway> gateway_;

  // Built-in adapters that are always active (no pluginlib required).
  std::shared_ptr<LogAdapter> log_adapter_;
  std::shared_ptr<DiagAdapter> diag_adapter_;

  // Subsystem adapter: CommandModeSubSystemAdapter or DrivingModeSubSystemAdapter, selected by
  // use_driving_mode.
  std::shared_ptr<IAdapterPlugin> subsystem_adapter_;

  pluginlib::ClassLoader<IAdapterPlugin> plugin_loader_;

  std::shared_ptr<IAdapterPlugin> switcher_plugin_;
};

}  // namespace autoware::redundancy_switcher
#endif  // REDUNDANCY_SWITCHER_INTERFACE_HPP_
