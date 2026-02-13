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

#include <autoware/boundary_departure_checker/utils.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <tl_expected/expected.hpp>
#include <rclcpp/logger.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/for_each.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_core/primitives/LineString.h>

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
      if (
        signal.traffic_light_group_id == element->id() && element->stopLine().has_value() &&
        autoware::traffic_light_utils::isTrafficSignalStop(lanelet, signal)) {
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
  return {red_stop_lines, amber_stop_lines};
}

tl::expected<void, std::string> TrafficLightFilter::is_feasible(
  const TrajectoryPoints & trajectory_points)
{
  if (!lanelet_map_ || !traffic_lights_ || trajectory_points.empty() || !vehicle_info_ptr_) {
    return {};  // Allow if no data available
  }

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
    bool crosses_amber_stop_line = false;
    boost::geometry::for_each_segment(trajectory_ls, [&](const auto & segment) {
      if (crosses_amber_stop_line) {
        return;
      }  // NOOP if already found the crossing point
      lanelet::BasicPoints2d intersection_points;
      const lanelet::BasicLineString2d segment_ls{segment.first, segment.second};
      boost::geometry::intersection(segment_ls, amber_stop_line, intersection_points);
      if (!intersection_points.empty()) {
        crosses_amber_stop_line = true;
        distance_to_stop_line +=
          boost::geometry::distance(segment.first, intersection_points.front());
      } else {
        distance_to_stop_line += boost::geometry::length(segment);
      }
    });
    const auto current_velocity = trajectory.front().longitudinal_velocity_mps;
    const auto current_acceleration = trajectory.front().acceleration_mps2;
    if (
      crosses_amber_stop_line &&
      !can_pass_amber_light(distance_to_stop_line, current_velocity, current_acceleration)) {
      return tl::make_unexpected("crosses amber light");  // Reject trajectory (cross amber light)
    }
  }
  return {};  // Allow trajectory
}

bool TrafficLightFilter::can_pass_amber_light(
  const double distance_to_stop_line, const double current_velocity,
  const double current_acceleration)
{
  const double max_acc = params_.traffic_light_filter.max_accel;
  const double max_jerk = params_.traffic_light_filter.max_jerk;
  const double delay_response_time = params_.traffic_light_filter.delay_response_time;
  const double yellow_lamp_period = params_.traffic_light_filter.yellow_lamp_period;
  const double yellow_light_stop_velocity = params_.traffic_light_filter.yellow_light_stop_velocity;
  const bool enable_pass_judge = params_.traffic_light_filter.enable_pass_judge;

  const double reachable_distance = current_velocity * yellow_lamp_period;

  // Calculate distance until ego vehicle decide not to stop,
  // taking into account the jerk and acceleration.
  const double pass_judge_line_distance =
    boundary_departure_checker::utils::calc_judge_line_dist_with_jerk_limit(
      current_velocity, current_acceleration, max_acc, max_jerk, delay_response_time);

  const bool distance_stoppable = pass_judge_line_distance < distance_to_stop_line;
  const bool slow_velocity = current_velocity < yellow_light_stop_velocity;
  const bool stoppable = distance_stoppable || slow_velocity;
  const bool reachable = distance_to_stop_line < reachable_distance;

  if (enable_pass_judge && !stoppable) {
    // Cannot stop under acceleration and jerk limits.
    // However, ego vehicle can't enter the intersection while the light is yellow.
    // It is called dilemma zone. Make a stop decision to be safe.
    if (!reachable) {
      // dilemma zone: emergency stop
      auto tmp_clock = rclcpp::Clock();
      RCLCPP_WARN_THROTTLE(
        *logger_, tmp_clock, 1000,
        "[traffic_light] cannot pass through intersection during yellow lamp!");
      return false;
    }
  } else {
    return false;
  }
  return true;
}
}  // namespace autoware::trajectory_traffic_rule_filter::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_traffic_rule_filter::plugin::TrafficLightFilter,
  autoware::trajectory_traffic_rule_filter::plugin::TrafficRuleFilterInterface)
