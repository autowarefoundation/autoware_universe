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

#ifndef TRAFFIC_LIGHT_MAP_VISUALIZER__TRAFFIC_LIGHT_VISUALIZER_HPP_
#define TRAFFIC_LIGHT_MAP_VISUALIZER__TRAFFIC_LIGHT_VISUALIZER_HPP_

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace autoware::traffic_light
{
struct Bulb
{
  lanelet::Id id;
  geometry_msgs::msg::Point position;
  uint8_t color;
};

// Key is the AutowareTrafficLight regulatory element ID, which equals
// the `traffic_light_group_id` carried by TrafficLightGroupArray messages.
using BulbsByGroupId = std::unordered_map<lanelet::Id, std::vector<Bulb>>;

BulbsByGroupId extract_bulbs(
  const std::vector<lanelet::AutowareTrafficLightConstPtr> & map_traffic_lights);

class TrafficLightVisualizer
{
public:
  explicit TrafficLightVisualizer(BulbsByGroupId bulbs_by_group_id);

  std::vector<visualization_msgs::msg::Marker> generate_markers(
    const autoware_perception_msgs::msg::TrafficLightGroupArray & detected_traffic_lights,
    builtin_interfaces::msg::Time stamp) const;

private:
  BulbsByGroupId bulbs_by_group_id_;
};
}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_MAP_VISUALIZER__TRAFFIC_LIGHT_VISUALIZER_HPP_
