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
#include <tl_expected/expected.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_core/primitives/LineString.h>

#include <string>
#include <vector>

namespace autoware::trajectory_traffic_rule_filter::plugin
{
TrafficLightFilter::TrafficLightFilter() : TrafficRuleFilterInterface("TrafficLightFilter")
{
}

void TrafficLightFilter::set_traffic_lights(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr & traffic_lights)
{
  traffic_lights_ = traffic_lights;
}

std::vector<lanelet::BasicLineString2d> TrafficLightFilter::get_red_stop_lines(
  const lanelet::ConstLanelet & lanelet) const
{
  std::vector<lanelet::BasicLineString2d> stop_lines;
  for (const auto & element : lanelet.regulatoryElementsAs<lanelet::TrafficLight>()) {
    for (const auto & signal : traffic_lights_->traffic_light_groups) {
      if (
        signal.traffic_light_group_id == element->id() && element->stopLine().has_value() &&
        autoware::traffic_light_utils::isTrafficSignalStop(lanelet, signal)) {
        stop_lines.push_back(lanelet::utils::to2D(element->stopLine()->basicLineString()));
      }
    }
  }
  return stop_lines;
}

tl::expected<void, std::string> TrafficLightFilter::is_feasible(
  const TrajectoryPoints & trajectory_points)
{
  if (!lanelet_map_ || !traffic_lights_ || trajectory_points.empty() || !vehicle_info_ptr_) {
    return {};  // Allow if no data available
  }

  lanelet::BasicLineString2d trajectory_ls;
  for (const auto & p : trajectory_points) {
    trajectory_ls.emplace_back(p.pose.position.x, p.pose.position.y);
  }
  if (vehicle_info_ptr_->max_longitudinal_offset_m > 0.0 && trajectory_ls.size() > 1) {
    // extend the trajectory linestring by the vehicle's longitudinal offset
    const lanelet::BasicSegment2d last_segment(
      trajectory_ls[trajectory_ls.size() - 2], trajectory_ls.back());
    const auto last_vector = last_segment.second - last_segment.first;
    const auto last_length = boost::geometry::length(last_segment);
    if (last_length > 0.0) {
      const auto ratio = (last_length + vehicle_info_ptr_->max_longitudinal_offset_m) / last_length;
      lanelet::BasicPoint2d front_vehicle_point = last_segment.first + last_vector * ratio;
      trajectory_ls.emplace_back(front_vehicle_point);
    }
  }

  const auto bbox = boost::geometry::return_envelope<lanelet::BoundingBox2d>(trajectory_ls);
  for (const auto & ll : lanelet_map_->laneletLayer.search(bbox)) {
    for (const auto & stop_line : get_red_stop_lines(ll)) {
      if (boost::geometry::intersects(trajectory_ls, stop_line)) {
        return tl::make_unexpected("crosses stop line of lanelet " + std::to_string(ll.id()));
      }
    }
  }
  return {};  // Allow if no red lights found
}
}  // namespace autoware::trajectory_traffic_rule_filter::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_traffic_rule_filter::plugin::TrafficLightFilter,
  autoware::trajectory_traffic_rule_filter::plugin::TrafficRuleFilterInterface)
