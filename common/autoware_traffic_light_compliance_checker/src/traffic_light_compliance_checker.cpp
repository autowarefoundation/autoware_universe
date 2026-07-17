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

#include "autoware/traffic_light_compliance_checker/traffic_light_compliance_checker.hpp"

#include "autoware/traffic_light_compliance_checker/structs.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/duration.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
autoware::traffic_light_compliance_checker::StatusTrackerParameters to_status_tracker_parameters(
  const autoware::traffic_light_compliance_checker::Parameters & parameters)
{
  autoware::traffic_light_compliance_checker::StatusTrackerParameters p{};
  p.stable_duration_threshold_red = parameters.stable_duration_threshold_red;
  p.stable_duration_threshold_amber = parameters.stable_duration_threshold_amber;
  p.stable_duration_threshold_unknown = parameters.stable_duration_threshold_unknown;
  return p;
}

using autoware::traffic_light_compliance_checker::StopLineInfo;

struct CollectedStopLine
{
  StopLineInfo stop_line_info;
  autoware_perception_msgs::msg::TrafficLightGroup signal;
  bool is_stop_signal;
};

/// @brief get route stop lines and their corresponding signals from the given traffic light groups
std::vector<CollectedStopLine> collect_stop_lines(
  const lanelet::LaneletMap & lanelet_map, const autoware_planning_msgs::msg::LaneletRoute & route,
  const std::vector<autoware_perception_msgs::msg::TrafficLightGroup> & traffic_light_groups)
{
  std::vector<CollectedStopLine> stop_lines;
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

    const auto traffic_light =
      std::dynamic_pointer_cast<const lanelet::TrafficLight>(*traffic_light_it);
    if (!traffic_light || !traffic_light->stopLine().has_value()) {
      continue;
    }
    stop_lines.push_back(
      CollectedStopLine{
        StopLineInfo{
          lanelet::utils::to2D(traffic_light->stopLine()->basicLineString()),
          signal.traffic_light_group_id},
        signal,
        autoware::traffic_light_utils::isTrafficSignalStop(
          lanelet_map.laneletLayer.get(hit->second), signal)});
  }
  return stop_lines;
}

std::optional<double> distance_between_stop_point_and_stop_line(
  const std::optional<lanelet::BasicPoint2d> & stop_point,
  const lanelet::BasicLineString2d & stop_line)
{
  if (stop_point.has_value()) {
    return boost::geometry::distance(*stop_point, stop_line);
  }
  return std::nullopt;
}

}  // namespace

namespace autoware::traffic_light_compliance_checker
{

TrafficLightComplianceChecker::TrafficLightComplianceChecker(
  const Parameters & parameters, const vehicle_info_utils::VehicleInfo & vehicle_info)
: params_(parameters),
  vehicle_info_(vehicle_info),
  status_tracker_(
    std::make_unique<TrafficLightStatusTracker>(to_status_tracker_parameters(parameters)))
{
}

void TrafficLightComplianceChecker::update_parameters(const Parameters & parameters)
{
  params_ = parameters;
  status_tracker_->update_parameters(to_status_tracker_parameters(parameters));
  if (params_.crossing_commitment_duration <= 0.0) {
    crossing_commitment_history_.clear();
  }
}

std::vector<CrossingCommitmentDebugInfo>
TrafficLightComplianceChecker::get_crossing_commitment_debug_info(
  const rclcpp::Time & current_time) const
{
  std::vector<CrossingCommitmentDebugInfo> debug_info;
  if (params_.crossing_commitment_duration <= 0.0) {
    return debug_info;
  }

  debug_info.reserve(crossing_commitment_history_.size());
  for (const auto & [traffic_light_id, commitment] : crossing_commitment_history_) {
    const auto elapsed_time = (current_time - commitment.committed_at).seconds();
    if (elapsed_time < 0.0 || elapsed_time > params_.crossing_commitment_duration) {
      continue;
    }
    debug_info.push_back(
      CrossingCommitmentDebugInfo{
        traffic_light_id, commitment.stop_line, commitment.committed_at, elapsed_time,
        params_.crossing_commitment_duration - elapsed_time});
  }
  return debug_info;
}

tl::expected<ComplianceResult, std::string> TrafficLightComplianceChecker::check(
  const Inputs & input, const bool check_red_lights, const bool check_amber_lights)
{
  const bool is_ego_stopped =
    std::abs(input.current_velocity) < params_.ego_stopped_velocity_threshold;
  const auto filtered_signals =
    status_tracker_->filter_signals(input.signals, input.current_time, is_ego_stopped);

  const auto force_reject_amber_ids =
    get_force_reject_amber_ids(input.current_time, is_ego_stopped);
  cleanup_crossing_commitment_history(input.current_time);

  auto result = get_crossings_with_filtered_signals(
    input, filtered_signals, force_reject_amber_ids, check_red_lights, check_amber_lights);
  if (!result) {
    return result;
  }

  if (params_.crossing_commitment_duration > 0.0 && !input.trajectory.empty()) {
    const auto front_pose = autoware_utils_geometry::calc_offset_pose(
      input.trajectory.front().pose, vehicle_info_.max_longitudinal_offset_m, 0.0, 0.0);
    const lanelet::BasicPoint2d ego_base_point(
      input.trajectory.front().pose.position.x, input.trajectory.front().pose.position.y);
    const lanelet::BasicPoint2d ego_front_point(front_pose.position.x, front_pose.position.y);
    update_commitments(
      result->crossings, input.current_time, params_.crossing_commitment_distance, ego_base_point,
      ego_front_point);
    remove_violations_for_commited_stop_lines(result->crossings);
  }

  result->crossings.erase(
    std::remove_if(
      result->crossings.begin(), result->crossings.end(),
      [&](const StopLineCrossing & crossing) {
        return crossing.violation_type == ViolationType::NONE;
      }),
    result->crossings.end());

  update_amber_rejection_history(*result, input.current_time, force_reject_amber_ids);
  cleanup_amber_rejection_history(input.current_time);

  return result;
}

std::vector<int64_t> TrafficLightComplianceChecker::get_force_reject_amber_ids(
  const rclcpp::Time & current_time, const bool is_ego_stopped) const
{
  std::vector<int64_t> force_reject_amber_ids;
  if (is_ego_stopped) {
    return force_reject_amber_ids;
  }
  for (const auto & [id, rejected_time] : amber_rejection_history_) {
    if ((current_time - rejected_time).seconds() <= params_.amber_rejection_hysteresis_duration) {
      force_reject_amber_ids.push_back(id);
    }
  }
  return force_reject_amber_ids;
}

void TrafficLightComplianceChecker::update_amber_rejection_history(
  const ComplianceResult & result, const rclcpp::Time & current_time,
  const std::vector<int64_t> & force_reject_amber_ids)
{
  for (const auto & crossing : result.crossings) {
    if (crossing.violation_type != ViolationType::AMBER_LIGHT) {
      continue;
    }
    if (
      std::find(
        force_reject_amber_ids.begin(), force_reject_amber_ids.end(), crossing.traffic_light_id) ==
      force_reject_amber_ids.end()) {
      amber_rejection_history_[crossing.traffic_light_id] = current_time;
    }
  }
}

void TrafficLightComplianceChecker::cleanup_amber_rejection_history(
  const rclcpp::Time & current_time)
{
  for (auto it = amber_rejection_history_.begin(); it != amber_rejection_history_.end();) {
    if ((current_time - it->second).seconds() > params_.amber_rejection_hysteresis_duration) {
      it = amber_rejection_history_.erase(it);
    } else {
      ++it;
    }
  }
}

void TrafficLightComplianceChecker::cleanup_crossing_commitment_history(
  const rclcpp::Time & current_time)
{
  for (auto it = crossing_commitment_history_.begin(); it != crossing_commitment_history_.end();) {
    const auto elapsed = (current_time - it->second.committed_at).seconds();
    if (elapsed < 0.0 || elapsed > params_.crossing_commitment_duration) {
      it = crossing_commitment_history_.erase(it);
    } else {
      ++it;
    }
  }
}

void TrafficLightComplianceChecker::update_commitments(
  const std::vector<StopLineCrossing> & crossings, const rclcpp::Time & current_time,
  const double crossing_commitment_distance, const lanelet::BasicPoint2d & ego_base_point,
  const lanelet::BasicPoint2d & ego_front_point)
{
  const lanelet::BasicLineString2d ego_to_front{ego_base_point, ego_front_point};
  for (const auto & crossing : crossings) {
    if (
      crossing_commitment_history_.find(crossing.traffic_light_id) !=
      crossing_commitment_history_.end()) {
      continue;
    }
    if (
      crossing.violation_type != ViolationType::NONE ||
      crossing.distance_between_stop_point_and_stop_line.has_value()) {
      continue;
    }

    auto front_has_reached_stop_line = false;
    if (vehicle_info_.max_longitudinal_offset_m > 0.0) {
      lanelet::BasicPoints2d intersection_points;
      boost::geometry::intersection(ego_to_front, crossing.stop_line, intersection_points);
      front_has_reached_stop_line = !intersection_points.empty();
    }
    const bool is_within_commitment_distance =
      front_has_reached_stop_line ||
      boost::geometry::distance(ego_front_point, crossing.stop_line) <=
        crossing_commitment_distance;
    if (is_within_commitment_distance) {
      crossing_commitment_history_[crossing.traffic_light_id] =
        CrossingCommitment{current_time, crossing.stop_line};
    }
  }
}

std::vector<StopLineCrossing> TrafficLightComplianceChecker::get_stop_line_crossings(
  const std::vector<StopLineInfo> & stop_lines, const lanelet::BasicLineString2d & trajectory_ls,
  const std::optional<lanelet::BasicPoint2d> & stop_point, const double distance_offset,
  const ViolationType violation_type_for_crossing) const
{
  std::vector<StopLineCrossing> crossings;
  for (const auto & stop_line : stop_lines) {
    auto distance_to_stop_line = 0.0;
    lanelet::BasicPoints2d intersection_points;
    for (size_t i = 0; i + 1 < trajectory_ls.size(); ++i) {
      const lanelet::BasicLineString2d segment{trajectory_ls[i], trajectory_ls[i + 1]};
      boost::geometry::intersection(segment, stop_line.line, intersection_points);
      if (!intersection_points.empty()) {
        distance_to_stop_line += static_cast<double>(
          boost::geometry::distance(segment.front(), intersection_points.front()));
        break;
      }
      distance_to_stop_line += static_cast<double>(boost::geometry::length(segment));
    }
    if (intersection_points.empty()) {
      continue;
    }
    StopLineCrossing crossing(
      stop_line.line, stop_line.traffic_light_id, intersection_points.front(),
      distance_to_stop_line + distance_offset);
    crossing.distance_between_stop_point_and_stop_line =
      distance_between_stop_point_and_stop_line(stop_point, stop_line.line);
    if (
      violation_type_for_crossing != ViolationType::NONE &&
      (!crossing.distance_between_stop_point_and_stop_line.has_value() ||
       crossing.distance_between_stop_point_and_stop_line.value() >
         params_.stop_overshoot_margin)) {
      crossing.violation_type = violation_type_for_crossing;
    }
    crossings.push_back(crossing);
  }
  return crossings;
}

std::vector<StopLineCrossing> TrafficLightComplianceChecker::get_amber_light_crossings(
  const std::vector<StopLineInfo> & amber_stop_lines,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const lanelet::BasicLineString2d & trajectory_ls,
  const std::optional<lanelet::BasicPoint2d> & stop_point,
  const std::vector<int64_t> & force_reject_amber_ids, const double distance_offset) const
{
  std::vector<StopLineCrossing> crossings;
  for (const auto & amber_stop_line : amber_stop_lines) {
    auto distance_to_stop_line = 0.0;
    std::optional<double> amber_stop_line_crossing_time;
    lanelet::BasicPoint2d intersection_point;
    for (size_t i = 0; i + 1 < trajectory.size(); ++i) {
      lanelet::BasicPoints2d intersection_points;
      const lanelet::BasicLineString2d segment{trajectory_ls[i], trajectory_ls[i + 1]};
      const auto segment_length = static_cast<double>(boost::geometry::length(segment));
      boost::geometry::intersection(segment, amber_stop_line.line, intersection_points);
      if (intersection_points.empty()) {
        distance_to_stop_line += segment_length;
        continue;
      }
      const auto distance_to_intersection =
        boost::geometry::distance(segment.front(), intersection_points.front());
      distance_to_stop_line += distance_to_intersection;
      const auto ratio = distance_to_intersection / segment_length;
      amber_stop_line_crossing_time = autoware::interpolation::lerp(
        rclcpp::Duration(trajectory[i].time_from_start).seconds(),
        rclcpp::Duration(trajectory[i + 1].time_from_start).seconds(), ratio);
      intersection_point = intersection_points.front();
      break;
    }
    if (!amber_stop_line_crossing_time.has_value()) {
      continue;
    }

    StopLineCrossing crossing(
      amber_stop_line.line, amber_stop_line.traffic_light_id, intersection_point,
      distance_to_stop_line + distance_offset);
    crossing.distance_between_stop_point_and_stop_line =
      distance_between_stop_point_and_stop_line(stop_point, amber_stop_line.line);

    const auto current_velocity = trajectory.front().longitudinal_velocity_mps;
    const auto current_acceleration = trajectory.front().acceleration_mps2;

    bool is_force_reject = std::find(
                             force_reject_amber_ids.begin(), force_reject_amber_ids.end(),
                             amber_stop_line.traffic_light_id) != force_reject_amber_ids.end();

    if (
      is_force_reject || !can_pass_amber_light(
                           distance_to_stop_line, current_velocity, current_acceleration,
                           *amber_stop_line_crossing_time)) {
      if (
        !crossing.distance_between_stop_point_and_stop_line.has_value() ||
        crossing.distance_between_stop_point_and_stop_line.value() >
          params_.stop_overshoot_margin) {
        crossing.violation_type = ViolationType::AMBER_LIGHT;
      }
    }
    crossings.push_back(crossing);
  }
  return crossings;
}

tl::expected<ComplianceResult, std::string>
TrafficLightComplianceChecker::get_crossings_with_filtered_signals(
  const Inputs & input,
  const autoware_perception_msgs::msg::TrafficLightGroupArray & filtered_signals,
  const std::vector<int64_t> & force_reject_amber_ids, const bool check_red_lights,
  const bool check_amber_lights)
{
  if (input.trajectory.empty() || (!check_red_lights && !check_amber_lights)) {
    return ComplianceResult{};
  }

  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> trajectory;
  lanelet::BasicLineString2d trajectory_ls;
  const auto ego_stopping_distance = autoware::motion_utils::calculate_stop_distance(
    input.current_velocity, input.current_acceleration,
    params_.checked_trajectory_length.deceleration_limit,
    params_.checked_trajectory_length.jerk_limit, params_.delay_response_time);
  // add the stop_overshoot_margin to not skip points beyond the stop line but within the margin
  const auto commitment_check_length = params_.crossing_commitment_duration > 0.0
                                         ? std::max(0.0, params_.crossing_commitment_distance) +
                                             vehicle_info_.max_longitudinal_offset_m
                                         : 0.0;
  const auto max_trajectory_length =
    std::max(ego_stopping_distance.value_or(0.0) + params_.stop_overshoot_margin, commitment_check_length);
  auto length = 0.0;
  auto backward_length = 0.0;
  std::optional<lanelet::BasicPoint2d> stop_point;
  auto last_p = input.trajectory.front();
  for (const auto & p : input.trajectory) {
    // skip points behind ego
    if (rclcpp::Duration(p.time_from_start).seconds() < 0.0) {
      backward_length += autoware_utils_geometry::calc_distance2d(last_p.pose, p.pose);
      last_p = p;
      continue;
    }

    const lanelet::BasicPoint2d lanelet_p(p.pose.position.x, p.pose.position.y);
    if (!trajectory_ls.empty())
      length += lanelet::geometry::distance2d(trajectory_ls.back(), lanelet_p);

    trajectory.push_back(p);
    trajectory_ls.emplace_back(lanelet_p);

    // search for a stop point beyond the current ego position
    if (length > 0.0 && p.longitudinal_velocity_mps <= params_.ego_stopped_velocity_threshold) {
      stop_point = trajectory_ls.back();
      break;
    }
    if (length > max_trajectory_length) break;
  }

  if (trajectory_ls.size() < 2) {
    return ComplianceResult{};  // allow empty or stopped trajectories as they do not cross traffic
                                // lights
  }

  if (vehicle_info_.max_longitudinal_offset_m > 0.0) {
    // extend the trajectory linestring by the vehicle's longitudinal offset
    const auto offset_pose = autoware_utils_geometry::calc_offset_pose(
      trajectory.back().pose, vehicle_info_.max_longitudinal_offset_m, 0.0, 0.0);
    const lanelet::BasicPoint2d offset_point(offset_pose.position.x, offset_pose.position.y);
    trajectory_ls.emplace_back(offset_point);
    if (stop_point.has_value()) stop_point.value() = offset_point;
  }

  const auto stop_line_infos = get_stop_lines(*input.map, input.route, filtered_signals);

  ComplianceResult result;
  if (check_red_lights) {
    const auto red_light_crossings = get_stop_line_crossings(
      stop_line_infos.red_stop_lines, trajectory_ls, stop_point, backward_length,
      ViolationType::RED_LIGHT);
    result.crossings.insert(
      result.crossings.end(), red_light_crossings.begin(), red_light_crossings.end());
  }
  if (check_amber_lights) {
    const auto amber_light_crossings =
      params_.treat_amber_light_as_red_light
        ? get_stop_line_crossings(
            stop_line_infos.amber_stop_lines, trajectory_ls, stop_point, backward_length,
            ViolationType::RED_LIGHT)
        : get_amber_light_crossings(
            stop_line_infos.amber_stop_lines, trajectory, trajectory_ls, stop_point,
            force_reject_amber_ids, backward_length);
    result.crossings.insert(
      result.crossings.end(), amber_light_crossings.begin(), amber_light_crossings.end());
  }
  const auto green_light_crossings = get_stop_line_crossings(
    stop_line_infos.green_stop_lines, trajectory_ls, stop_point, backward_length);
  result.crossings.insert(
    result.crossings.end(), green_light_crossings.begin(), green_light_crossings.end());
  const auto unknown_light_crossings = get_stop_line_crossings(
    stop_line_infos.unknown_stop_lines, trajectory_ls, stop_point, backward_length);
  result.crossings.insert(
    result.crossings.end(), unknown_light_crossings.begin(), unknown_light_crossings.end());
  return result;
}

StopLineInfos TrafficLightComplianceChecker::get_stop_lines(
  const lanelet::LaneletMap & lanelet_map, const autoware_planning_msgs::msg::LaneletRoute & route,
  const autoware_perception_msgs::msg::TrafficLightGroupArray & traffic_lights) const
{
  StopLineInfos stop_line_infos;
  for (const auto & collected_stop_line :
       collect_stop_lines(lanelet_map, route, traffic_lights.traffic_light_groups)) {
    const auto & stop_line_info = collected_stop_line.stop_line_info;
    const auto & signal = collected_stop_line.signal;
    const bool is_red = autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
      signal.elements, autoware_perception_msgs::msg::TrafficLightElement::CIRCLE,
      autoware_perception_msgs::msg::TrafficLightElement::RED);
    const bool is_amber = autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
      signal.elements, autoware_perception_msgs::msg::TrafficLightElement::CIRCLE,
      autoware_perception_msgs::msg::TrafficLightElement::AMBER);
    const bool is_unknown = autoware::traffic_light_utils::hasTrafficLightColor(
      signal.elements, autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN);
    const bool is_green = autoware::traffic_light_utils::hasTrafficLightColor(
      signal.elements, autoware_perception_msgs::msg::TrafficLightElement::GREEN);

    const auto is_treated_as_red = is_red ||
                                   (is_unknown && params_.treat_unknown_light_as_red_light) ||
                                   (is_amber && params_.treat_amber_light_as_red_light);
    if (is_treated_as_red && collected_stop_line.is_stop_signal) {
      stop_line_infos.red_stop_lines.push_back(stop_line_info);
    } else if (is_amber && collected_stop_line.is_stop_signal) {
      stop_line_infos.amber_stop_lines.push_back(stop_line_info);
    } else if (is_green) {
      stop_line_infos.green_stop_lines.push_back(stop_line_info);
    } else {
      stop_line_infos.unknown_stop_lines.push_back(stop_line_info);
    }
  }
  return stop_line_infos;
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

}  // namespace autoware::traffic_light_compliance_checker
