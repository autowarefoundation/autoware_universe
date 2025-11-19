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

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <memory>
#include <vector>

namespace autoware::trajectory_traffic_rule_filter::plugin
{
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;

TrafficLightFilter::TrafficLightFilter() : TrafficRuleFilterInterface("TrafficLightFilter")
{
  boundary_departure_checker_ =
    std::make_unique<autoware::boundary_departure_checker::BoundaryDepartureChecker>();
}

lanelet::ConstLanelets TrafficLightFilter::get_lanelets_from_trajectory(
  const TrajectoryPoints & trajectory_points) const
{
  lanelet::ConstLanelets lanes;
  PathWithLaneId path;
  path.points.reserve(trajectory_points.size());
  for (const auto & point : trajectory_points) {
    PathPointWithLaneId path_point;
    path_point.point.pose = point.pose;
    path_point.point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
    path_point.point.lateral_velocity_mps = point.lateral_velocity_mps;
    path_point.point.heading_rate_rps = point.heading_rate_rps;
    path.points.push_back(path_point);
  }
  const auto lanelet_distance_pair =
    boundary_departure_checker_->getLaneletsFromPath(lanelet_map_, path);
  if (lanelet_distance_pair.empty()) {
    return lanes;
  }

  for (const auto & lanelet_distance : lanelet_distance_pair) {
    const auto & lanelet = lanelet_distance.second;
    lanes.push_back(lanelet);
  }
  return lanes;
}

void TrafficLightFilter::set_traffic_lights(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr & traffic_lights)
{
  traffic_lights_ = traffic_lights;
}

bool TrafficLightFilter::is_feasible(const TrajectoryPoints & trajectory_points)
{
  if (!lanelet_map_ || !traffic_lights_ || trajectory_points.empty()) {
    return true;  // Allow if no data available
  }

  const auto lanes = get_lanelets_from_trajectory(trajectory_points);

  for (const auto & lane : lanes) {
    // Check traffic lights for this lanelet
    for (const auto & element : lane.regulatoryElementsAs<lanelet::TrafficLight>()) {
      // Find corresponding traffic light in received data
      for (const auto & signal : traffic_lights_->traffic_light_groups) {
        if (signal.traffic_light_group_id == static_cast<int64_t>(element->id())) {
          // Check if stop is required using autoware traffic light utils
          if (autoware::traffic_light_utils::isTrafficSignalStop(lane, signal)) {
            return false;  // Reject trajectory if stop is required
          }
        }
      }
    }
  }

  return true;  // Allow if no red lights found
}
}  // namespace autoware::trajectory_traffic_rule_filter::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_traffic_rule_filter::plugin::TrafficLightFilter,
  autoware::trajectory_traffic_rule_filter::plugin::TrafficRuleFilterInterface)
