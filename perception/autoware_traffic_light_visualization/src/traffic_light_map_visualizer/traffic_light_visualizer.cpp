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
using autoware::traffic_light::Bulb;
using autoware_perception_msgs::msg::TrafficLightElement;

// --- helpers for generate_markers ---

bool is_color_detected(
  const std::vector<TrafficLightElement> & detected_elements, uint8_t bulb_color)
{
  for (const auto & element : detected_elements) {
    if (element.color == bulb_color) {
      return true;
    }
  }
  return false;
}

std_msgs::msg::ColorRGBA marker_color_for(uint8_t bulb_color)
{
  std_msgs::msg::ColorRGBA color;
  constexpr float marker_alpha = 0.999f;
  color.a = marker_alpha;
  if (bulb_color == TrafficLightElement::RED) {
    color.r = 1.0f;
  } else if (bulb_color == TrafficLightElement::GREEN) {
    color.g = 1.0f;
  } else if (bulb_color == TrafficLightElement::AMBER) {
    color.r = 1.0f;
    color.g = 1.0f;
  }
  return color;
}

visualization_msgs::msg::Marker create_bulb_marker(
  const Bulb & bulb, builtin_interfaces::msg::Time stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.frame_locked = true;
  marker.ns = "traffic_light";
  marker.id = bulb.id;
  constexpr uint32_t marker_lifetime_ns = 200000000u;  // 200 ms
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = marker_lifetime_ns;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.pose.position = bulb.position;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  constexpr float marker_scale = 0.3f;
  marker.scale.x = marker_scale;
  marker.scale.y = marker_scale;
  marker.scale.z = marker_scale;

  marker.color = marker_color_for(bulb.color);

  return marker;
}

std::vector<visualization_msgs::msg::Marker> create_markers_for_active_bulbs(
  const std::vector<Bulb> & bulbs, const std::vector<TrafficLightElement> & detected_elements,
  builtin_interfaces::msg::Time stamp)
{
  std::vector<visualization_msgs::msg::Marker> markers;
  for (const auto & bulb : bulbs) {
    if (!is_color_detected(detected_elements, bulb.color)) {
      continue;
    }
    markers.push_back(create_bulb_marker(bulb, stamp));
  }
  return markers;
}

// --- helpers for extract_bulbs ---

std::optional<uint8_t> parse_bulb_color(const std::string & color_attribute)
{
  if (color_attribute == "red") return TrafficLightElement::RED;
  if (color_attribute == "green") return TrafficLightElement::GREEN;
  if (color_attribute == "yellow") return TrafficLightElement::AMBER;
  return std::nullopt;
}

}  // namespace

namespace autoware::traffic_light
{
BulbsByGroupId extract_bulbs(
  const std::vector<lanelet::AutowareTrafficLightConstPtr> & map_traffic_lights)
{
  BulbsByGroupId result;
  for (const auto & map_traffic_light : map_traffic_lights) {
    std::vector<Bulb> bulbs;
    // A lightBulbs linestring with "traffic_light_id" represents a bulb group.
    // Points with "color" attribute represent individual bulbs.
    for (const auto & light_bulbs : map_traffic_light->lightBulbs()) {
      if (!light_bulbs.hasAttribute("traffic_light_id")) {
        continue;
      }
      for (const auto & point : light_bulbs) {
        if (!point.hasAttribute("color")) {
          continue;
        }
        const auto color = parse_bulb_color(point.attribute("color").value());
        if (!color) {
          continue;
        }
        Bulb bulb;
        bulb.id = point.id();
        bulb.position.x = point.x();
        bulb.position.y = point.y();
        bulb.position.z = point.z();
        bulb.color = *color;
        bulbs.push_back(bulb);
      }
    }
    if (!bulbs.empty()) {
      result.emplace(map_traffic_light->id(), std::move(bulbs));
    }
  }
  return result;
}

TrafficLightVisualizer::TrafficLightVisualizer(BulbsByGroupId bulbs_by_group_id)
: bulbs_by_group_id_(std::move(bulbs_by_group_id))
{
}

std::vector<visualization_msgs::msg::Marker> TrafficLightVisualizer::generate_markers(
  const autoware_perception_msgs::msg::TrafficLightGroupArray & detected_traffic_lights,
  builtin_interfaces::msg::Time stamp) const
{
  std::vector<visualization_msgs::msg::Marker> markers;

  for (const auto & traffic_light_group : detected_traffic_lights.traffic_light_groups) {
    auto it = bulbs_by_group_id_.find(traffic_light_group.traffic_light_group_id);
    if (it == bulbs_by_group_id_.end()) {
      continue;
    }
    auto group_markers =
      create_markers_for_active_bulbs(it->second, traffic_light_group.elements, stamp);
    markers.insert(markers.end(), group_markers.begin(), group_markers.end());
  }

  return markers;
}
}  // namespace autoware::traffic_light
