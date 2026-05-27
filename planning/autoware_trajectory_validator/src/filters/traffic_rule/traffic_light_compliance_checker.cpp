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

#include "autoware/trajectory_validator/filters/traffic_rule/traffic_light_compliance_checker.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <rclcpp/duration.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
using autoware::trajectory_validator::traffic_light_filter::StopLineInfo;

/// @brief get stop lines where ego need to stop, and their corresponding signals from the given
/// traffic light groups
std::vector<std::pair<StopLineInfo, autoware_perception_msgs::msg::TrafficLightGroup>>
collect_stop_lines(
  const lanelet::LaneletMap & lanelet_map, const autoware_planning_msgs::msg::LaneletRoute & route,
  const std::vector<autoware_perception_msgs::msg::TrafficLightGroup> & traffic_light_groups)
{
  std::vector<std::pair<StopLineInfo, autoware_perception_msgs::msg::TrafficLightGroup>> stop_lines;
  std::unordered_map<lanelet::Id, lanelet::Id> route_lanelet_id_per_traffic_light_id;
  for (const auto & segment : route.segments) {
    for (const auto & tl : lanelet_map.laneletLayer.get(segment.preferred_primitive.id)
                             .regulatoryElementsAs<lanelet::TrafficLight>()) {
      route_lanelet_id_per_traffic_light_id.emplace(tl->id(), segment.preferred_primitive.id);
    }
  }

  for (const auto & signal : traffic_light_groups) {
    const auto hit = route_lanelet_id_per_traffic_light_id.find(signal.traffic_light_group_id);
    if (hit == route_lanelet_id_per_traffic_light_id.end()) {
      continue;
    }
    const auto traffic_light_it =
      lanelet_map.regulatoryElementLayer.find(signal.traffic_light_group_id);
    if (traffic_light_it == lanelet_map.regulatoryElementLayer.end()) {
      continue;
    }

    if (!autoware::traffic_light_utils::isTrafficSignalStop(
          lanelet_map.laneletLayer.get(hit->second), signal)) {
      continue;
    }

    const auto traffic_light =
      std::dynamic_pointer_cast<const lanelet::TrafficLight>(*traffic_light_it);
    if (!traffic_light || !traffic_light->stopLine().has_value()) {
      continue;
    }
    stop_lines.emplace_back(
      StopLineInfo{
        lanelet::utils::to2D(traffic_light->stopLine()->basicLineString()),
        signal.traffic_light_group_id},
      signal);
  }
  return stop_lines;
}
}  // namespace

namespace autoware::trajectory_validator::traffic_light_filter
{

TrafficLightComplianceChecker::TrafficLightComplianceChecker(
  const Parameters & parameters, const vehicle_info_utils::VehicleInfo & vehicle_info)
: params_(parameters), vehicle_info_(vehicle_info)
{
}

void TrafficLightComplianceChecker::update_parameters(const Parameters & parameters)
{
  params_ = parameters;
}

tl::expected<ComplianceResult, std::string> TrafficLightComplianceChecker::check(
  const Inputs & input) const
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> trajectory;
  lanelet::BasicLineString2d trajectory_ls;
  const auto distance_for_ego_to_stop = autoware::motion_utils::calculate_stop_distance(
    input.current_velocity, input.current_acceleration,
    params_.checked_trajectory_length.deceleration_limit,
    params_.checked_trajectory_length.jerk_limit, params_.delay_response_time);
  const auto max_trajectory_length = distance_for_ego_to_stop.value_or(0.0);
  auto length = 0.0;
  bool is_stopping_trajectory = false;
  for (const auto & p : input.trajectory) {
    // skip points behind ego
    if (rclcpp::Duration(p.time_from_start).seconds() < 0.0) {
      continue;
    }
    const lanelet::BasicPoint2d lanelet_p(p.pose.position.x, p.pose.position.y);
    if (!trajectory_ls.empty()) {
      length += lanelet::geometry::distance2d(trajectory_ls.back(), lanelet_p);
    }

    trajectory.push_back(p);
    trajectory_ls.emplace_back(lanelet_p);

    // skip points beyond the first stop, or skip once we reach the maximum length
    is_stopping_trajectory = p.longitudinal_velocity_mps <= 0.0;
    if (is_stopping_trajectory || length > max_trajectory_length) {
      break;
    }
  }

  if (trajectory_ls.size() < 2) {
    return ComplianceResult{};  // allow empty or stopped trajectories as they do not cross traffic
                                // lights
  }

  if (vehicle_info_.max_longitudinal_offset_m > 0.0) {
    // extend the trajectory linestring by the vehicle's longitudinal offset
    const lanelet::BasicSegment2d last_segment(
      trajectory_ls[trajectory_ls.size() - 2], trajectory_ls.back());
    const auto last_vector = last_segment.second - last_segment.first;
    const auto last_length = boost::geometry::length(last_segment);
    if (last_length > 0.0) {
      const auto ratio = (last_length + vehicle_info_.max_longitudinal_offset_m) / last_length;
      lanelet::BasicPoint2d front_vehicle_point = last_segment.first + last_vector * ratio;
      trajectory_ls.emplace_back(front_vehicle_point);
    }
  }
  std::optional<lanelet::BasicPoint2d> stop_point;
  if (is_stopping_trajectory) {
    stop_point = trajectory_ls.back();
  }

  const auto [red_stop_lines, amber_stop_lines] =
    get_stop_lines(*input.map, input.route, input.signals);

  ComplianceResult result;

  // Check for red light crossings
  for (const auto & red_stop_line : red_stop_lines) {
    if (boost::geometry::intersects(trajectory_ls, red_stop_line.line)) {
      if (is_stop_point_within_margin_from_stop_line(stop_point, red_stop_line.line)) {
        continue;
      }
      result.violations.push_back(
        {ViolationType::RED_LIGHT, red_stop_line.line, red_stop_line.traffic_light_id});
    }
  }

  // Check for amber light crossings
  for (const auto & amber_stop_line : amber_stop_lines) {
    auto distance_to_stop_line = 0.0;
    std::optional<double> amber_stop_line_crossing_time;
    for (size_t i = 0; i + 1 < trajectory.size(); ++i) {
      lanelet::BasicPoints2d intersection_points;
      const lanelet::BasicLineString2d segment{trajectory_ls[i], trajectory_ls[i + 1]};
      const auto segment_length = static_cast<double>(boost::geometry::length(segment));
      boost::geometry::intersection(segment, amber_stop_line.line, intersection_points);
      if (!intersection_points.empty()) {
        const auto distance_to_intersection =
          boost::geometry::distance(segment.front(), intersection_points.front());
        distance_to_stop_line += distance_to_intersection;
        const auto ratio = distance_to_intersection / segment_length;
        amber_stop_line_crossing_time = autoware::interpolation::lerp(
          rclcpp::Duration(trajectory[i].time_from_start).seconds(),
          rclcpp::Duration(trajectory[i + 1].time_from_start).seconds(), ratio);
        break;
      }
      distance_to_stop_line += segment_length;
    }

    const auto current_velocity = trajectory.front().longitudinal_velocity_mps;
    const auto current_acceleration = trajectory.front().acceleration_mps2;
    if (amber_stop_line_crossing_time) {
      if (is_stop_point_within_margin_from_stop_line(stop_point, amber_stop_line.line)) {
        continue;
      }

      bool is_force_reject = false;
      if (
        std::find(
          input.force_reject_amber_ids.begin(), input.force_reject_amber_ids.end(),
          amber_stop_line.traffic_light_id) != input.force_reject_amber_ids.end()) {
        is_force_reject = true;
      }

      if (
        is_force_reject || !can_pass_amber_light(
                             distance_to_stop_line, current_velocity, current_acceleration,
                             *amber_stop_line_crossing_time)) {
        result.violations.push_back(
          {ViolationType::AMBER_LIGHT, amber_stop_line.line, amber_stop_line.traffic_light_id});
      }
    }
  }

  return result;
}

std::pair<std::vector<StopLineInfo>, std::vector<StopLineInfo>>
TrafficLightComplianceChecker::get_stop_lines(
  const lanelet::LaneletMap & lanelet_map, const autoware_planning_msgs::msg::LaneletRoute & route,
  const autoware_perception_msgs::msg::TrafficLightGroupArray & traffic_lights) const
{
  std::vector<StopLineInfo> red_stop_lines;
  std::vector<StopLineInfo> amber_stop_lines;
  for (const auto & [stop_line_info, signal] :
       collect_stop_lines(lanelet_map, route, traffic_lights.traffic_light_groups)) {
    if (autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
          signal.elements, autoware_perception_msgs::msg::TrafficLightElement::CIRCLE,
          autoware_perception_msgs::msg::TrafficLightElement::RED)) {
      red_stop_lines.push_back(stop_line_info);
    }
    if (autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
          signal.elements, autoware_perception_msgs::msg::TrafficLightElement::CIRCLE,
          autoware_perception_msgs::msg::TrafficLightElement::AMBER)) {
      amber_stop_lines.push_back(stop_line_info);
    }
  }
  if (params_.treat_amber_light_as_red_light) {
    red_stop_lines.insert(red_stop_lines.end(), amber_stop_lines.begin(), amber_stop_lines.end());
    amber_stop_lines.clear();
  }
  return {red_stop_lines, amber_stop_lines};
}

bool TrafficLightComplianceChecker::is_stop_point_within_margin_from_stop_line(
  const std::optional<lanelet::BasicPoint2d> & stop_point,
  const lanelet::BasicLineString2d & stop_line) const
{
  if (stop_point.has_value()) {
    if (boost::geometry::distance(*stop_point, stop_line) <= params_.stop_overshoot_margin) {
      return true;
    }
  }
  return false;
}

bool TrafficLightComplianceChecker::can_pass_amber_light(
  const double distance_to_stop_line, const double current_velocity,
  const double current_acceleration, const double time_to_cross_stop_line) const
{
  const double decel_limit = params_.deceleration_limit;
  const double jerk_limit = params_.jerk_limit;
  const double delay_response_time = params_.delay_response_time;
  const auto distance_for_ego_to_stop = autoware::motion_utils::calculate_stop_distance(
    current_velocity, current_acceleration, decel_limit, jerk_limit, delay_response_time);

  const bool can_stop =
    distance_for_ego_to_stop.has_value() && *distance_for_ego_to_stop <= distance_to_stop_line;
  const bool can_pass_in_time = time_to_cross_stop_line <= params_.crossing_time_limit;
  const bool can_pass = !can_stop && can_pass_in_time;
  return can_pass;
}

}  // namespace autoware::trajectory_validator::traffic_light_filter
