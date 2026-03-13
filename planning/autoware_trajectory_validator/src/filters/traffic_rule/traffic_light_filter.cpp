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

#include "autoware/trajectory_validator/filters/traffic_rule/traffic_light_filter.hpp"

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
#include <memory>
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

std::optional<std::string> is_invalid_input(
  const autoware::trajectory_validator::FilterContext & context,
  const std::shared_ptr<autoware::vehicle_info_utils::VehicleInfo> & vehicle_info)
{
  if (!context.lanelet_map) {
    return "Lanelet map is not available in the context.";
  }

  if (!vehicle_info) {
    return "Vehicle info is not set.";
  }

  if (!context.traffic_light_signals) {
    return "Traffic light signals are not available in the context.";
  }

  return std::nullopt;
}
}  // namespace

namespace autoware::trajectory_validator::plugin::traffic_rule
{

TrafficLightFilter::TrafficLightFilter() : ValidatorInterface("TrafficLightFilter")
{
}

void TrafficLightFilter::set_parameters(rclcpp::Node & node)
{
  using autoware_utils_rclcpp::get_or_declare_parameter;
  params_.deceleration_limit =
    get_or_declare_parameter<double>(node, "traffic_light.deceleration_limit");
  params_.jerk_limit = get_or_declare_parameter<double>(node, "traffic_light.jerk_limit");
  params_.delay_response_time =
    get_or_declare_parameter<double>(node, "traffic_light.delay_response_time");
  params_.crossing_time_limit =
    get_or_declare_parameter<double>(node, "traffic_light.crossing_time_limit");
  params_.treat_amber_light_as_red_light =
    get_or_declare_parameter<bool>(node, "traffic_light.treat_amber_light_as_red_light");
}

void TrafficLightFilter::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param<double>(parameters, "traffic_light.deceleration_limit", params_.deceleration_limit);
  update_param<double>(parameters, "traffic_light.jerk_limit", params_.jerk_limit);
  update_param<double>(
    parameters, "traffic_light.delay_response_time", params_.delay_response_time);
  update_param<double>(
    parameters, "traffic_light.crossing_time_limit", params_.crossing_time_limit);
  update_param<bool>(
    parameters, "traffic_light.treat_amber_light_as_red_light",
    params_.treat_amber_light_as_red_light);
}

std::pair<std::vector<lanelet::BasicLineString2d>, std::vector<lanelet::BasicLineString2d>>
TrafficLightFilter::get_stop_lines(
  const lanelet::Lanelets & lanelets,
  const autoware_perception_msgs::msg::TrafficLightGroupArray & traffic_lights) const
{
  std::vector<lanelet::BasicLineString2d> red_stop_lines;
  std::vector<lanelet::BasicLineString2d> amber_stop_lines;
  for (const auto & lanelet : lanelets) {
    for (const auto & [stop_line, signal] :
         get_matching_stop_lines(lanelet, traffic_lights.traffic_light_groups)) {
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
  if (params_.treat_amber_light_as_red_light) {
    red_stop_lines.insert(red_stop_lines.end(), amber_stop_lines.begin(), amber_stop_lines.end());
    amber_stop_lines.clear();
  }
  return {red_stop_lines, amber_stop_lines};
}

tl::expected<void, std::string> TrafficLightFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  if (const auto has_invalid_input = is_invalid_input(context, vehicle_info_ptr_)) {
    return tl::make_unexpected(*has_invalid_input);
  }
  TrajectoryPoints trajectory;
  lanelet::BasicLineString2d trajectory_ls;
  for (const auto & p : traj_points) {
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

  if (trajectory_ls.size() < 2) {
    return {};  // allow empty or stopped trajectories as they do not cross traffic lights
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
  const lanelet::Lanelets candidate_lanelets = context.lanelet_map->laneletLayer.search(bbox);
  const auto [red_stop_lines, amber_stop_lines] =
    get_stop_lines(candidate_lanelets, *context.traffic_light_signals);
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
  const double decel_limit = params_.deceleration_limit;
  const double jerk_limit = params_.jerk_limit;
  const double delay_response_time = params_.delay_response_time;
  const auto distance_for_ego_to_stop = motion_utils::calculate_stop_distance(
    current_velocity, current_acceleration, decel_limit, jerk_limit, delay_response_time);

  const bool can_stop =
    distance_for_ego_to_stop.has_value() && *distance_for_ego_to_stop <= distance_to_stop_line;
  const bool can_pass_in_time = time_to_cross_stop_line <= params_.crossing_time_limit;
  const bool can_pass = !can_stop && can_pass_in_time;
  return can_pass;
}
}  // namespace autoware::trajectory_validator::plugin::traffic_rule

#include <pluginlib/class_list_macros.hpp>
namespace traffic_rule = autoware::trajectory_validator::plugin::traffic_rule;
PLUGINLIB_EXPORT_CLASS(
  traffic_rule::TrafficLightFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
