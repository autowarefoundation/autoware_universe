// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__PLANNING_VALIDATOR__MANAGER_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__MANAGER_HPP_

#include "autoware/planning_validator/plugin_interface.hpp"
#include "autoware/planning_validator/types.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_validator
{
class PlanningValidatorManager
{
public:
  PlanningValidatorManager();
  void load_plugin(
    rclcpp::Node & node, const std::string & name,
    const std::shared_ptr<PlanningValidatorContext> & context);
  void unload_plugin(rclcpp::Node & node, const std::string & name);
  void validate(bool & is_critical);

private:
  pluginlib::ClassLoader<PluginInterface> plugin_loader_;
  std::vector<std::shared_ptr<PluginInterface>> loaded_plugins_;
};
}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR__MANAGER_HPP_
