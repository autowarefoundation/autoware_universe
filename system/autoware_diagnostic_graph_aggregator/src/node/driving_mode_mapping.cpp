// Copyright 2026 The Autoware Contributors
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

#include "driving_mode_mapping.hpp"

#include "graph/graph.hpp"
#include "graph/nodes.hpp"

#include <yaml-cpp/yaml.h>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

DrivingModeMapping::DrivingModeMapping(autoware::agnocast_wrapper::Node & node, const Graph & graph)
{
  std::unordered_map<std::string, BaseUnit *> path_to_unit;
  for (const auto & unit : graph.nodes()) {
    path_to_unit[unit->path()] = unit;
  }

  const auto create_mapping = [&node, &path_to_unit](const std::string & param) {
    const auto mappings = node.declare_parameter<std::vector<std::string>>(param);
    std::unordered_map<uint32_t, BaseUnit *> units;
    for (const auto & mapping : mappings) {
      YAML::Node yaml = YAML::Load(mapping);
      const auto mode = yaml["mode"].as<uint32_t>();
      const auto path = yaml["path"].as<std::string>();
      const auto iter = path_to_unit.find(path);
      if (iter != path_to_unit.end()) {
        units[mode] = iter->second;
      } else {
        RCLCPP_ERROR_STREAM(node.get_logger(), "Mode path not found: " << mode << " " << path);
      }
    }
    return units;
  };
  available_units_ = create_mapping("driving_mode_available");
  continuable_units_ = create_mapping("driving_mode_continuable");

  pub_available_ =
    node.create_publisher<DrivingModeFlag>("/system/driving_mode/available", rclcpp::QoS(1));
  pub_continuable_ =
    node.create_publisher<DrivingModeFlag>("/system/driving_mode/continuable", rclcpp::QoS(1));
}

void DrivingModeMapping::update(const rclcpp::Time & stamp) const
{
  const auto create_message = [stamp](const auto & pub, const auto & units) {
    auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub);
    msg->stamp = stamp;
    for (const auto & [mode, unit] : units) {
      tier4_system_msgs::msg::DrivingModeFlagItem item;
      item.mode = mode;
      item.flag = unit->level() == DiagnosticStatus::OK;
      msg->items.push_back(item);
    }
    return msg;
  };

  auto available_msg = create_message(pub_available_, available_units_);
  auto continuable_msg = create_message(pub_continuable_, continuable_units_);
  pub_available_->publish(std::move(available_msg));
  pub_continuable_->publish(std::move(continuable_msg));
}

}  // namespace autoware::diagnostic_graph_aggregator
