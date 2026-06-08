// Copyright 2026 TIER IV, Inc.
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

#include "autoware/map_loader/map_loader_host.hpp"

#include <rclcpp/logging.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::map_loader
{

MapLoaderHost::MapLoaderHost(const rclcpp::NodeOptions & options)
: Node("map_loader", options),
  plugin_loader_(
    std::make_unique<pluginlib::ClassLoader<plugin::MapLoaderPluginBase>>(
      "autoware_map_host", "autoware::map_loader::plugin::MapLoaderPluginBase"))
{
  declare_parameter<std::vector<std::string>>("plugin_names", std::vector<std::string>{});
  initialize_plugins();
}

void MapLoaderHost::initialize_plugins()
{
  if (initialized_plugins_) {
    return;
  }

  const auto plugin_names = get_parameter("plugin_names").as_string_array();
  for (const auto & plugin_name : plugin_names) {
    load_plugin(plugin_name);
  }

  for (auto & plugin : plugins_) {
    plugin->on_startup(data_);
  }

  initialized_plugins_ = true;
}

void MapLoaderHost::load_plugin(const std::string & plugin_name)
{
  try {
    auto plugin = plugin_loader_->createSharedInstance(plugin_name);

    for (const auto & loaded : plugins_) {
      if (plugin->get_name() == loaded->get_name()) {
        RCLCPP_WARN_STREAM(get_logger(), "Plugin '" << plugin_name << "' is already loaded.");
        return;
      }
    }

    plugin->bind(plugin_name, this);
    plugin->initialize(plugin_name, this, data_);
    plugins_.push_back(plugin);

    RCLCPP_INFO_STREAM(get_logger(), "Loaded map loader plugin: " << plugin_name);
  } catch (const pluginlib::CreateClassException & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Failed to load plugin '" << plugin_name << "': " << e.what());
    throw;
  }
}

}  // namespace autoware::map_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::map_loader::MapLoaderHost)
