// Copyright 2026 TIER IV, Inc.
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

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <tl_expected/expected.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/for_each.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_core/primitives/LineString.h>

#include <ctime>
#include <string>
#include <utility>
#include <vector>

namespace
{
/// @brief get stop lines where ego need to stop, and corresponding signal matching the given
/// lanelet
std::vector<std::pair<lanelet::BasicLineString2d, autoware_perception_msgs::msg::TrafficLightGroup>>
get_matching_stop_lines(
  const lanelet::Lanelet & lanelet,
  const std::vector<autoware_perception_msgs::msg::TrafficLightGroup> & traffic_light_groups)
{
  std::vector<
    std::pair<lanelet::BasicLineString2d, autoware_perception_msgs::msg::TrafficLightGroup>>
    matching_stop_lines;
  for (const auto & element : lanelet.regulatoryElementsAs<lanelet::TrafficLight>()) {
    for (const auto & signal : traffic_light_groups) {
      const auto is_matching =
        signal.traffic_light_group_id == element->id() && element->stopLine().has_value();
      if (is_matching && autoware::traffic_light_utils::isTrafficSignalStop(lanelet, signal)) {
        matching_stop_lines.emplace_back(
          lanelet::utils::to2D(element->stopLine()->basicLineString()), signal);
      }
    }
  }
  return matching_stop_lines;
}
}  // namespace

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

std::pair<std::vector<lanelet::BasicLineString2d>, std::vector<lanelet::BasicLineString2d>>
TrafficLightFilter::get_stop_lines(const lanelet::Lanelets & lanelets) const
{
  std::vector<lanelet::BasicLineString2d> red_stop_lines;
  std::vector<lanelet::BasicLineString2d> amber_stop_lines;
  for (const auto & lanelet : lanelets) {
    for (const auto & [stop_line, signal] :
         get_matching_stop_lines(lanelet, traffic_lights_->traffic_light_groups)) {
      if (traffic_light_utils::hasTrafficLightCircleColor(
            signal.elements, tier4_perception_msgs::msg::TrafficLightElement::RED)) {
        red_stop_lines.push_back(stop_line);
      }
      if (traffic_light_utils::hasTrafficLightCircleColor(
            signal.elements, tier4_perception_msgs::msg::TrafficLightElement::AMBER)) {
        amber_stop_lines.push_back(stop_line);
      }
    }
  }
  if (params_.traffic_light_filter.treat_amber_light_as_red_light) {
    red_stop_lines.insert(red_stop_lines.end(), amber_stop_lines.begin(), amber_stop_lines.end());
    amber_stop_lines.clear();
  }
  return {red_stop_lines, amber_stop_lines};
}

tl::expected<void, std::string> TrafficLightFilter::is_feasible(
  const TrajectoryPoints & trajectory_points)
{
  TrajectoryPoints trajectory;
  lanelet::BasicLineString2d trajectory_ls;
  for (const auto & p : trajectory_points) {
    // skip points behind ego
    if (rclcpp::Duration(p.time_from_start).seconds() < 0.0) {
      continue;
    }
    // skip points beyond the first stop
    if (p.longitudinal_velocity_mps <= 0.0) {
      break;
    }
    trajectory.push_back(p);
    trajectory_ls.emplace_back(p.pose.position.x, p.pose.position.y);
  }

  if (!lanelet_map_ || !traffic_lights_ || trajectory_ls.size() < 2 || !vehicle_info_ptr_) {
    return tl::make_unexpected("No data available");  // Reject if no data available
  }

  if (vehicle_info_ptr_->max_longitudinal_offset_m > 0.0) {
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
  const lanelet::Lanelets candidate_lanelets = lanelet_map_->laneletLayer.search(bbox);
  const auto [red_stop_lines, amber_stop_lines] = get_stop_lines(candidate_lanelets);
  for (const auto & red_stop_line : red_stop_lines) {
    if (boost::geometry::intersects(trajectory_ls, red_stop_line)) {
      return tl::make_unexpected("crosses red light");  // Reject trajectory (cross red light)
    }
  }
  for (const auto & amber_stop_line : amber_stop_lines) {
    auto distance_to_stop_line = 0.0;
    std::optional<double> amber_stop_line_crossing_time;
    for (size_t i = 0; i + 1 < trajectory.size(); ++i) {
      lanelet::BasicPoints2d intersection_points;
      const lanelet::BasicLineString2d segment{trajectory_ls[i], trajectory_ls[i + 1]};
      const auto segment_length = static_cast<double>(boost::geometry::length(segment));
      boost::geometry::intersection(segment, amber_stop_line, intersection_points);
      if (!intersection_points.empty()) {
        const auto distance_to_intersection =
          boost::geometry::distance(segment.front(), intersection_points.front());
        distance_to_stop_line += distance_to_intersection;
        const auto ratio = distance_to_intersection / segment_length;
        amber_stop_line_crossing_time = interpolation::lerp(
          rclcpp::Duration(trajectory[i].time_from_start).seconds(),
          rclcpp::Duration(trajectory[i + 1].time_from_start).seconds(), ratio);
        break;
      }
      distance_to_stop_line += segment_length;
    }
    const auto current_velocity = trajectory.front().longitudinal_velocity_mps;
    const auto current_acceleration = trajectory.front().acceleration_mps2;
    if (
      amber_stop_line_crossing_time && !can_pass_amber_light(
                                         distance_to_stop_line, current_velocity,
                                         current_acceleration, *amber_stop_line_crossing_time)) {
      return tl::make_unexpected("crosses amber light");  // Reject trajectory (cross amber light)
    }
  }
  return {};  // Allow trajectory
}

bool TrafficLightFilter::can_pass_amber_light(
  const double distance_to_stop_line, const double current_velocity,
  const double current_acceleration, const double time_to_cross_stop_line) const
{
  const double decel_limit = params_.traffic_light_filter.deceleration_limit;
  const double jerk_limit = params_.traffic_light_filter.jerk_limit;
  const double delay_response_time = params_.traffic_light_filter.delay_response_time;
  const auto distance_for_ego_to_stop = motion_utils::calculate_stop_distance(
    current_velocity, current_acceleration, decel_limit, jerk_limit, delay_response_time);

  const bool can_stop =
    distance_for_ego_to_stop.has_value() && *distance_for_ego_to_stop <= distance_to_stop_line;
  const bool can_pass_in_time =
    time_to_cross_stop_line <= params_.traffic_light_filter.crossing_time_limit;
  const bool can_pass = !can_stop && can_pass_in_time;
  return can_pass;
}
}  // namespace autoware::trajectory_traffic_rule_filter::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_traffic_rule_filter::plugin::TrafficLightFilter,
  autoware::trajectory_traffic_rule_filter::plugin::TrafficRuleFilterInterface)
