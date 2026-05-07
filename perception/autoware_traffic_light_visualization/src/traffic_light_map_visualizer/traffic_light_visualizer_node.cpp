// Copyright 2020 Tier IV, Inc.
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

#include "traffic_light_visualizer_node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>

namespace autoware::traffic_light
{
TrafficLightMapVisualizerNode::TrafficLightMapVisualizerNode(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node("traffic_light_map_visualizer_node", node_options)
{
  using std::placeholders::_1;

  light_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/output/traffic_light", 1);
  tl_state_sub_ = create_subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>(
    "~/input/tl_state", 1,
    std::bind(&TrafficLightMapVisualizerNode::traffic_lights_callback, this, _1));
  vector_map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightMapVisualizerNode::bin_map_callback, this, _1));
}

void TrafficLightMapVisualizerNode::traffic_lights_callback(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr traffic_lights)
{
  if (!visualizer_) {
    return;
  }
  visualization_msgs::msg::MarkerArray output_msg;
  const builtin_interfaces::msg::Time current_time = now();
  output_msg.markers = visualizer_->generate_markers(*traffic_lights, current_time);
  light_marker_pub_->publish(output_msg);
}

void TrafficLightMapVisualizerNode::bin_map_callback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_map_msg)
{
  lanelet::LaneletMapPtr viz_lanelet_map = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*input_map_msg));
  RCLCPP_DEBUG(get_logger(), "Map is loaded");

  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(viz_lanelet_map);
  visualizer_.emplace(lanelet::utils::query::autowareTrafficLights(all_lanelets));
}
}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightMapVisualizerNode)
