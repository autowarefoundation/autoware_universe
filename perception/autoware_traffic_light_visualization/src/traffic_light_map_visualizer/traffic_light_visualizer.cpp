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

#include "traffic_light_visualizer.hpp"

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{

struct BulbColor
{
  uint8_t map_bulb_color;
  std_msgs::msg::ColorRGBA marker_color;
};

bool has_bulb_color(const lanelet::ConstPoint3d & point, const std::string & expected_value)
{
  lanelet::Attribute attr = point.attribute("color");
  return attr.value() == expected_value;
}

std::optional<BulbColor> resolve_bulb_color(const lanelet::ConstPoint3d & point)
{
  using autoware_perception_msgs::msg::TrafficLightElement;

  BulbColor bulb;
  constexpr float marker_alpha = 0.999f;
  bulb.marker_color.a = marker_alpha;

  if (has_bulb_color(point, "red")) {
    bulb.map_bulb_color = TrafficLightElement::RED;
    bulb.marker_color.r = 1.0f;
  } else if (has_bulb_color(point, "green")) {
    bulb.map_bulb_color = TrafficLightElement::GREEN;
    bulb.marker_color.g = 1.0f;
  } else if (has_bulb_color(point, "yellow")) {
    bulb.map_bulb_color = TrafficLightElement::AMBER;
    bulb.marker_color.r = 1.0f;
    bulb.marker_color.g = 1.0f;
  } else {
    return std::nullopt;
  }
  return bulb;
}

bool is_color_detected(
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & detected_elements,
  uint8_t bulb_color)
{
  for (const auto & element : detected_elements) {
    if (element.color == bulb_color) {
      return true;
    }
  }
  return false;
}

visualization_msgs::msg::Marker create_bulb_marker(
  const lanelet::ConstPoint3d & point, const std_msgs::msg::ColorRGBA & color,
  const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.frame_locked = true;
  marker.ns = "traffic_light";
  marker.id = point.id();
  constexpr uint32_t marker_lifetime_ns = 200000000u;  // 200 ms
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = marker_lifetime_ns;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.pose.position.x = point.x();
  marker.pose.position.y = point.y();
  marker.pose.position.z = point.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  constexpr float marker_scale = 0.3f;
  marker.scale.x = marker_scale;
  marker.scale.y = marker_scale;
  marker.scale.z = marker_scale;

  marker.color = color;

  return marker;
}

std::vector<visualization_msgs::msg::Marker> create_markers_for_active_bulbs(
  const std::vector<lanelet::ConstPoint3d> & bulb_points,
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & detected_elements,
  const builtin_interfaces::msg::Time & stamp)
{
  std::vector<visualization_msgs::msg::Marker> markers;
  for (const auto & point : bulb_points) {
    auto bulb = resolve_bulb_color(point);
    if (!bulb) {
      continue;
    }
    if (!is_color_detected(detected_elements, bulb->map_bulb_color)) {
      continue;
    }
    markers.push_back(create_bulb_marker(point, bulb->marker_color, stamp));
  }
  return markers;
}

}  // namespace

namespace autoware::traffic_light
{
TrafficLightVisualizer::TrafficLightVisualizer(
  const std::vector<lanelet::AutowareTrafficLightConstPtr> & regulatory_elements)
{
  for (const auto & regulatory_element : regulatory_elements) {
    std::vector<lanelet::ConstPoint3d> points;
    // A lightBulbs linestring with "traffic_light_id" represents a bulb group.
    // Points with "color" attribute represent individual bulbs.
    for (const auto & light_bulbs : regulatory_element->lightBulbs()) {
      if (!light_bulbs.hasAttribute("traffic_light_id")) {
        continue;
      }
      for (const auto & point : light_bulbs) {
        if (point.hasAttribute("color")) {
          points.push_back(point);
        }
      }
    }
    if (!points.empty()) {
      bulb_points_by_group_id_.emplace(regulatory_element->id(), std::move(points));
    }
  }
}

std::vector<visualization_msgs::msg::Marker> TrafficLightVisualizer::generate_markers(
  const autoware_perception_msgs::msg::TrafficLightGroupArray & detected_traffic_lights,
  const builtin_interfaces::msg::Time & stamp) const
{
  std::vector<visualization_msgs::msg::Marker> markers;

  for (const auto & traffic_light_group : detected_traffic_lights.traffic_light_groups) {
    auto it = bulb_points_by_group_id_.find(traffic_light_group.traffic_light_group_id);
    if (it == bulb_points_by_group_id_.end()) {
      continue;
    }
    auto group_markers =
      create_markers_for_active_bulbs(it->second, traffic_light_group.elements, stamp);
    markers.insert(markers.end(), group_markers.begin(), group_markers.end());
  }

  return markers;
}
}  // namespace autoware::traffic_light
