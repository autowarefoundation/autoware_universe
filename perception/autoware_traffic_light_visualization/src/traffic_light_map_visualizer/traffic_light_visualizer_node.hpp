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

#ifndef TRAFFIC_LIGHT_MAP_VISUALIZER__TRAFFIC_LIGHT_VISUALIZER_NODE_HPP_
#define TRAFFIC_LIGHT_MAP_VISUALIZER__TRAFFIC_LIGHT_VISUALIZER_NODE_HPP_

#include "traffic_light_visualizer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <optional>

namespace autoware::traffic_light
{
class TrafficLightMapVisualizerNode : public rclcpp::Node
{
public:
  explicit TrafficLightMapVisualizerNode(const rclcpp::NodeOptions & node_options);

private:
  using TrafficLightGroupArray = autoware_perception_msgs::msg::TrafficLightGroupArray;
  using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;

  void detected_traffic_lights_callback(
    const TrafficLightGroupArray::ConstSharedPtr detected_traffic_lights);
  void lanelet_map_callback(const LaneletMapBin::ConstSharedPtr lanelet_map_msg);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traffic_light_marker_pub_;
  rclcpp::Subscription<TrafficLightGroupArray>::SharedPtr detected_traffic_lights_sub_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr lanelet_map_sub_;

  std::optional<TrafficLightVisualizer> visualizer_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_MAP_VISUALIZER__TRAFFIC_LIGHT_VISUALIZER_NODE_HPP_
