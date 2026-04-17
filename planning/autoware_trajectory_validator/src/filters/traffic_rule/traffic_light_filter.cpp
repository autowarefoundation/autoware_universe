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

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware/trajectory/temporal_trajectory.hpp>
#include <autoware/trajectory/threshold.hpp>
#include <autoware/trajectory/utils/add_offset.hpp>
#include <autoware/trajectory/utils/crop.hpp>
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware/trajectory/utils/find_intervals.hpp>
#include <tl_expected/expected.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LineString.h>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{
/// @brief get stop lines where ego need to stop, and their corresponding signals from the given
/// traffic light groups
std::vector<std::pair<lanelet::BasicLineString2d, autoware_perception_msgs::msg::TrafficLightGroup>>
collect_stop_lines(
  const lanelet::LaneletMap & lanelet_map,
  const std::vector<autoware_perception_msgs::msg::TrafficLightGroup> & traffic_light_groups)
{
  std::vector<
    std::pair<lanelet::BasicLineString2d, autoware_perception_msgs::msg::TrafficLightGroup>>
    stop_lines;

  for (const auto & signal : traffic_light_groups) {
    const auto traffic_light_it =
      lanelet_map.regulatoryElementLayer.find(signal.traffic_light_group_id);
    if (traffic_light_it == lanelet_map.regulatoryElementLayer.end()) {
      continue;
    }

    const auto traffic_light =
      std::dynamic_pointer_cast<const lanelet::TrafficLight>(*traffic_light_it);
    if (!traffic_light || !traffic_light->stopLine().has_value()) {
      continue;
    }
    stop_lines.emplace_back(
      lanelet::utils::to2D(traffic_light->stopLine()->basicLineString()), signal);
  }
  return stop_lines;
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

void TrafficLightFilter::update_parameters(const validator::Params & params)
{
  params_ = params.traffic_light;
}

std::pair<std::vector<lanelet::BasicLineString2d>, std::vector<lanelet::BasicLineString2d>>
TrafficLightFilter::get_stop_lines(
  const lanelet::LaneletMap & lanelet_map,
  const autoware_perception_msgs::msg::TrafficLightGroupArray & traffic_lights) const
{
  std::vector<lanelet::BasicLineString2d> red_stop_lines;
  std::vector<lanelet::BasicLineString2d> amber_stop_lines;
  for (const auto & [stop_line, signal] :
       collect_stop_lines(lanelet_map, traffic_lights.traffic_light_groups)) {
    if (traffic_light_utils::hasTrafficLightCircleColor(
          signal.elements, tier4_perception_msgs::msg::TrafficLightElement::RED)) {
      red_stop_lines.push_back(stop_line);
    }
    if (traffic_light_utils::hasTrafficLightCircleColor(
          signal.elements, tier4_perception_msgs::msg::TrafficLightElement::AMBER)) {
      amber_stop_lines.push_back(stop_line);
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

  if (traj_points.empty()) {
    return {};  // allow empty trajectory
  }

  constexpr auto delay_response_time = 0.0;
  const auto distance_for_ego_to_stop = motion_utils::calculate_stop_distance(
    context.odometry->twist.twist.linear.x, context.acceleration->accel.accel.linear.x,
    params_.checked_trajectory_length.deceleration_limit,
    params_.checked_trajectory_length.jerk_limit, delay_response_time);
  const auto max_trajectory_length = distance_for_ego_to_stop.value_or(0.0);

  auto trajectory_opt = experimental::trajectory::TemporalTrajectory::Builder().build(traj_points);

  if (!trajectory_opt) {
    return tl::make_unexpected("Failed to build TemporalTrajectory");
  }

  auto trajectory = trajectory_opt.value();

  trajectory = experimental::trajectory::crop_time(
    trajectory, 0.0, trajectory.end_time());  // Only consider future trajectory points

  trajectory = experimental::trajectory::crop_distance(
    trajectory, 0.0,
    std::min(
      trajectory.length(),
      max_trajectory_length));  // Only consider trajectory points within the maximum length

  auto stopped_trajectory = experimental::trajectory::find_intervals(
    trajectory, [](const auto & point) { return point.longitudinal_velocity_mps <= 0.0; });

  auto stopped_intervals =
    stopped_trajectory.empty() ? trajectory.length() : stopped_trajectory.front().start.distance;

  trajectory = experimental::trajectory::crop_distance(
    trajectory, 0.0, stopped_intervals);  // Only consider trajectory points before the first stop

  if (trajectory.length() <= experimental::trajectory::k_points_minimum_dist_threshold) {
    return {};  // allow trajectories that are too short as they do not cross traffic lights
  }

  auto offset_trajectory = experimental::trajectory::add_offset(
    trajectory, vehicle_info_ptr_->max_longitudinal_offset_m, 0.0);

  const auto [red_stop_lines, amber_stop_lines] =
    get_stop_lines(*context.lanelet_map, *context.traffic_light_signals);

  for (const auto & red_stop_line : red_stop_lines) {
    if (!experimental::trajectory::crossed(offset_trajectory, red_stop_line).empty()) {
      return tl::make_unexpected("crosses red light");  // Reject trajectory (cross red light)
    }
  }

  const auto current_point = offset_trajectory.compute_from_time(offset_trajectory.start_time());
  for (const auto & amber_stop_line : amber_stop_lines) {
    const auto crossed_points =
      experimental::trajectory::crossed(offset_trajectory, amber_stop_line);
    if (crossed_points.empty()) {
      continue;
    }

    const auto & crossed_point = crossed_points.front();
    const auto current_velocity = current_point.longitudinal_velocity_mps;
    const auto current_acceleration = current_point.acceleration_mps2;
    if (!can_pass_amber_light(
          crossed_point.distance, current_velocity, current_acceleration, crossed_point.time)) {
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
