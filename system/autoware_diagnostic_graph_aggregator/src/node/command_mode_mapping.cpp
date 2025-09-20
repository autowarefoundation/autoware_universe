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

#include "command_mode_mapping.hpp"

#include "graph/graph.hpp"
#include "graph/nodes.hpp"

#include <yaml-cpp/yaml.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

CommandModeMapping::CommandModeMapping(rclcpp::Node & node, const Graph & graph)
{
  std::unordered_map<std::string, BaseUnit *> path_to_unit;
  for (const auto & unit : graph.nodes()) {
    path_to_unit[unit->path()] = unit;
  }

  const auto mappings = node.declare_parameter<std::vector<std::string>>("command_mode_mappings");
  for (const auto & mapping : mappings) {
    YAML::Node yaml = YAML::Load(mapping);
    const auto mode = yaml["mode"].as<uint16_t>();
    const auto path = yaml["path"].as<std::string>();
    const auto iter = path_to_unit.find(path);
    if (iter != path_to_unit.end()) {
      mode_to_unit_[mode] = iter->second;
    } else {
      RCLCPP_ERROR_STREAM(node.get_logger(), "Mode path not found: " << mode << " " << path);
    }
  }

  pub_ = node.create_publisher<Availability>("~/availability", rclcpp::QoS(1));
}

void CommandModeMapping::update(const rclcpp::Time & stamp) const
{
  Availability message;
  message.stamp = stamp;
  for (const auto & [mode, unit] : mode_to_unit_) {
    AvailabilityItem item;
    item.mode = mode;
    item.available = unit->level() == DiagnosticStatus::OK;
    message.items.push_back(item);
  }
  pub_->publish(message);
}

}  // namespace autoware::diagnostic_graph_aggregator
