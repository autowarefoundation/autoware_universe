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

#include "autoware/planning_validator/manager.hpp"

#include <boost/format.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_validator
{

PlanningValidatorManager::PlanningValidatorManager()
: plugin_loader_("autoware_planning_validator", "autoware::planning_validator::PluginInterface")
{
}

void PlanningValidatorManager::load_plugin(
  rclcpp::Node & node, const std::string & name,
  const std::shared_ptr<PlanningValidatorContext> & context)
{
  // Check if the plugin is already loaded.
  if (plugin_loader_.isClassLoaded(name)) {
    RCLCPP_WARN(node.get_logger(), "The plugin '%s' is already loaded.", name.c_str());
    return;
  }
  if (plugin_loader_.isClassAvailable(name)) {
    const auto plugin = plugin_loader_.createSharedInstance(name);
    plugin->init(node, name, context);

    // register
    loaded_plugins_.push_back(plugin);
    RCLCPP_DEBUG(node.get_logger(), "The plugin '%s' has been loaded", name.c_str());
  } else {
    RCLCPP_ERROR(node.get_logger(), "The plugin '%s' is not available", name.c_str());
  }
}

void PlanningValidatorManager::unload_plugin(rclcpp::Node & node, const std::string & name)
{
  auto it = std::remove_if(loaded_plugins_.begin(), loaded_plugins_.end(), [&](const auto plugin) {
    return plugin->get_module_name() == name;
  });

  if (it == loaded_plugins_.end()) {
    RCLCPP_WARN(
      node.get_logger(), "The plugin '%s' is not in the registered modules", name.c_str());
  } else {
    loaded_plugins_.erase(it, loaded_plugins_.end());
    RCLCPP_INFO(node.get_logger(), "The scene plugin '%s' has been unloaded", name.c_str());
  }
}

void PlanningValidatorManager::validate(bool & is_critical)
{
  for (auto & plugin : loaded_plugins_) {
    bool is_critical_error = false;
    plugin->validate(is_critical_error);
    is_critical |= is_critical_error;
  }
}

}  // namespace autoware::planning_validator
