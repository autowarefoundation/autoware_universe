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

#ifndef NODE__COMMAND_MODE_MAPPING_HPP_
#define NODE__COMMAND_MODE_MAPPING_HPP_

#include "types/forward.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/command_mode_availability.hpp>

#include <unordered_map>

namespace autoware::diagnostic_graph_aggregator
{

class CommandModeMapping
{
public:
  CommandModeMapping(rclcpp::Node & node, const Graph & graph);
  void update(const rclcpp::Time & stamp) const;

private:
  using Availability = tier4_system_msgs::msg::CommandModeAvailability;
  using AvailabilityItem = tier4_system_msgs::msg::CommandModeAvailabilityItem;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Availability>::SharedPtr pub_;

  std::unordered_map<uint16_t, BaseUnit *> mode_to_unit_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // NODE__COMMAND_MODE_MAPPING_HPP_
