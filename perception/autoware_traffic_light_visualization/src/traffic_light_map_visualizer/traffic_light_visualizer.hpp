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

#ifndef TRAFFIC_LIGHT_MAP_VISUALIZER__TRAFFIC_LIGHT_VISUALIZER_HPP_
#define TRAFFIC_LIGHT_MAP_VISUALIZER__TRAFFIC_LIGHT_VISUALIZER_HPP_

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <unordered_map>
#include <vector>

namespace autoware::traffic_light
{
class TrafficLightVisualizer
{
public:
  explicit TrafficLightVisualizer(
    const std::vector<lanelet::AutowareTrafficLightConstPtr> & regulatory_elements);

  std::vector<visualization_msgs::msg::Marker> generate_markers(
    const autoware_perception_msgs::msg::TrafficLightGroupArray & detected_traffic_lights,
    const builtin_interfaces::msg::Time & stamp) const;

private:
  std::unordered_map<lanelet::Id, std::vector<lanelet::ConstPoint3d>> bulb_points_by_group_id_;
};
}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_MAP_VISUALIZER__TRAFFIC_LIGHT_VISUALIZER_HPP_
