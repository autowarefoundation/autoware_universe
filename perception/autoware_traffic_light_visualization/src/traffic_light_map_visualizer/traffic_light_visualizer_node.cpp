// Copyright 2020-2026 Tier IV, Inc.
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

  traffic_light_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/output/traffic_light", 1);
  detected_traffic_lights_sub_ = create_subscription<TrafficLightGroupArray>(
    "~/input/tl_state", 1,
    std::bind(&TrafficLightMapVisualizerNode::detected_traffic_lights_callback, this, _1));
  lanelet_map_sub_ = create_subscription<LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightMapVisualizerNode::lanelet_map_callback, this, _1));
}

void TrafficLightMapVisualizerNode::detected_traffic_lights_callback(
  const TrafficLightGroupArray::ConstSharedPtr detected_traffic_lights)
{
  if (!visualizer_) {
    return;
  }
  visualization_msgs::msg::MarkerArray output_msg;
  const builtin_interfaces::msg::Time stamp = now();
  output_msg.markers = visualizer_->generate_markers(*detected_traffic_lights, stamp);
  traffic_light_marker_pub_->publish(output_msg);
}

void TrafficLightMapVisualizerNode::lanelet_map_callback(
  const LaneletMapBin::ConstSharedPtr lanelet_map_msg)
{
  lanelet::LaneletMapPtr lanelet_map = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*lanelet_map_msg));

  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map);
  auto map_traffic_lights = lanelet::utils::query::autowareTrafficLights(all_lanelets);
  visualizer_.emplace(extract_bulbs(map_traffic_lights));
}
}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightMapVisualizerNode)
