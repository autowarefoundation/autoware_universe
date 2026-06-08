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

#include "autoware/map_loader/plugins/lanelet2_visualization_plugin.hpp"

#include "autoware/map_loader/map_loader_plugin_base.hpp"

// #include <autoware/lanelet2_map_visualizer/lanelet2_map_visualization.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <string>

namespace autoware::map_loader::plugin
{

class Lanelet2VisualizationPlugin : public MapLoaderPluginBase
{
public:
  void initialize(
    const std::string & /*name*/, rclcpp::Node * node, MapLoaderData & /*data*/) override
  {
    pub_marker_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "vector_map_marker", rclcpp::QoS{1}.transient_local());
  }

  void on_startup(MapLoaderData & data) override
  {
    if (!data.vector_map_msg.has_value()) {
      RCLCPP_WARN(
        get_node_ptr()->get_logger(), "Vector map is not loaded; skipping lanelet2 visualization.");
      return;
    }

    const auto markers = autoware::lanelet2_map_visualizer::create_vector_map_marker_array(
      data.vector_map_msg.value());
    pub_marker_->publish(markers);
    RCLCPP_INFO(get_node_ptr()->get_logger(), "Published vector map markers.");
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
};

}  // namespace autoware::map_loader::plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::map_loader::plugin::Lanelet2VisualizationPlugin,
  autoware::map_loader::plugin::MapLoaderPluginBase)
