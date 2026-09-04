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
#ifndef REDUNDANCY_SWITCHER_INTERFACE__PLUGIN__I_ADAPTER_PLUGIN_HPP_
#define REDUNDANCY_SWITCHER_INTERFACE__PLUGIN__I_ADAPTER_PLUGIN_HPP_

#include "redundancy_switcher_interface/ir/output_commands.hpp"

#include <memory>

// Forward declaration only; keeps this header ROS-free for pure GTest of Processor/EventGateway.
namespace rclcpp
{
class Node;
}

namespace autoware::redundancy_switcher
{

class CommandBus;
class EventGateway;

class IAdapterPlugin
{
public:
  virtual ~IAdapterPlugin() = default;

  virtual void initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway) = 0;

  // Process only the effect types this adapter owns; ignore all others.
  // Do not call gateway_->submit() synchronously from here — deadlock risk.
  virtual void execute(const OutputCommand & command) = 0;
};

}  // namespace autoware::redundancy_switcher
#endif  // REDUNDANCY_SWITCHER_INTERFACE__PLUGIN__I_ADAPTER_PLUGIN_HPP_
