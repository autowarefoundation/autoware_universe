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

#include "generic_service_divider/service_divider_plugin_base.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <vector>

namespace generic_service_divider
{

class GenericServiceDividerNode : public rclcpp::Node
{
public:
  explicit GenericServiceDividerNode(const rclcpp::NodeOptions & options)
  : Node("generic_service_divider", options),
    plugin_loader_(
      "autoware_generic_service_divider", "generic_service_divider::ServiceDividerPluginBase")
  {
    const auto plugin_names =
      declare_parameter<std::vector<std::string>>("plugins", std::vector<std::string>{});

    if (plugin_names.empty()) {
      RCLCPP_WARN(get_logger(), "No plugins configured. Node will do nothing.");
      return;
    }

    auto this_shared = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    for (const auto & name : plugin_names) {
      RCLCPP_INFO(get_logger(), "Loading plugin: %s", name.c_str());
      try {
        auto plugin = plugin_loader_.createSharedInstance(name);
        plugin->initialize(this_shared);
        plugin->setup_service_division();
        plugins_.push_back(plugin);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to load plugin '%s': %s", name.c_str(), e.what());
      }
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu service divider plugin(s)", plugins_.size());
  }

private:
  pluginlib::ClassLoader<ServiceDividerPluginBase> plugin_loader_;
  std::vector<std::shared_ptr<ServiceDividerPluginBase>> plugins_;
};

}  // namespace generic_service_divider

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(generic_service_divider::GenericServiceDividerNode)
