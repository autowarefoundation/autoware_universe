// Copyright 2024 TIER IV, Inc.
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
#include "autoware/behavior_path_bidirectional_traffic_module/manager.hpp"

#include "autoware/behavior_path_bidirectional_traffic_module/scene.hpp"

#include <rclcpp/logging.hpp>

#include <memory>
#include <vector>

namespace autoware::behavior_path_planner
{

void BidirectionalTrafficModuleManager::init(rclcpp::Node * node)
{
  initInterface(node, {});
  parameters_ = std::make_shared<BidirectionalTrafficModuleParameters>();
  parameters_->init_from_node(node, "bidirectional_traffic");
}

std::unique_ptr<SceneModuleInterface>
BidirectionalTrafficModuleManager::createNewSceneModuleInstance()
{
  return std::make_unique<BidirectionalTrafficModule>(
    "bidirectional_traffic", *node_, parameters_, rtc_interface_ptr_map_,
    objects_of_interest_marker_interface_ptr_map_, planning_factor_interface_);
}

void BidirectionalTrafficModuleManager::updateModuleParams(
  const std::vector<rclcpp::Parameter> & /*parameters*/)
{
  // Implementation will be added here in the future.
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::BidirectionalTrafficModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
