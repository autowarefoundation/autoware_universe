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

#include "autoware/trajectory_traffic_rule_filter/filters/traffic_light_filter.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace autoware::trajectory_traffic_rule_filter::plugin
{

namespace
{
bool is_red(const autoware_perception_msgs::msg::TrafficLightElement & element)
{
  using autoware_perception_msgs::msg::TrafficLightElement;
  return element.color == TrafficLightElement::RED;
}
}  // namespace

TrafficLightFilter::TrafficLightFilter() : TrafficRuleFilterInterface("TrafficLightFilter")
{
}

bool TrafficLightFilter::is_feasible(const TrajectoryPoints & trajectory_points)
{
  if (!traffic_lights_ || !lanelet_map_ || trajectory_points.empty()) {
    return true;
  }

  return !check_traffic_light_collision(trajectory_points);
}

void TrafficLightFilter::set_traffic_lights()
{
  const auto traffic_signal_msg = traffic_signals_subscriber_.take_data();

  if (traffic_signal_msg) {
    traffic_light_id_map_.clear();
    for (const auto & signal : traffic_signal_msg->traffic_light_groups) {
      TrafficSignalStamped traffic_signal;
      traffic_signal.stamp = traffic_signal_msg->stamp;
      traffic_signal.signal = signal;
      traffic_light_id_map_[signal.traffic_light_group_id] = traffic_signal;
    }
  }
}

void TrafficLightFilter::set_lanelet_map(const lanelet::LaneletMapPtr & lanelet_map)
{
  lanelet_map_ = lanelet_map;
}

bool TrafficLightFilter::check_traffic_light_collision(
  const TrajectoryPoints & trajectory_points) const
{
  if (!traffic_lights_ || !lanelet_map_) {
    return false;  // No collision if data unavailable
  }

  // Check each traffic light group
  for (const auto & traffic_light_group : traffic_lights_->traffic_light_groups) {
    // Check if any element indicates stop is required
    bool needs_to_stop = false;
    for (const auto & element : traffic_light_group.elements) {
      if (is_red_or_yellow(element)) {
        needs_to_stop = true;
        break;
      }
    }

    if (!needs_to_stop) {
      continue;  // Green light, no need to check
    }

    // Get traffic light ID
    const auto traffic_light_id = traffic_light_group.traffic_light_group_id;

    // Find lanelets with this traffic light
    for (const auto & lanelet : lanelet_map_->laneletLayer) {
      auto traffic_lights = lanelet.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>();

      for (const auto & tl : traffic_lights) {
        // Check if this matches our traffic light ID
        // Note: ID matching logic may need adjustment based on actual implementation

        // Get stop line
        const auto stop_line = tl->stopLine();
        if (!stop_line) {
          continue;
        }

        // Check distance to stop line
        const double distance =
          calculate_distance_to_stop_line(trajectory_points, stop_line.value());

        // If trajectory passes close to stop line when light is red/yellow, it's a violation
        constexpr double stop_line_margin = 5.0;  // meters
        if (distance < stop_line_margin) {
          return true;  // Traffic light violation detected
        }
      }
    }
  }

  return false;  // No collision detected
}

}  // namespace autoware::trajectory_traffic_rule_filter::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_traffic_rule_filter::plugin::TrafficLightFilter,
  autoware::trajectory_traffic_rule_filter::plugin::TrafficRuleFilterInterface)
