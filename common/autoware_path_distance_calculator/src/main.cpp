// Copyright 2021 Tier IV, Inc.
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

#include "main.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/LaneletPath.h>

#include <algorithm>

namespace autoware::path_distance_calculator
{
namespace lanelet2_utils = autoware::experimental::lanelet2_utils;

namespace
{
std::optional<lanelet::ConstLanelet> find_lanelet(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & pose)
{
  return lanelet2_utils::get_closest_lanelet(lanelets, pose);
}

double arc_length_to(const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Pose & pose)
{
  return lanelet2_utils::get_arc_coordinates({lanelet}, pose).length;
}

// Sum of the full lengths of the lanelets strictly between current_lane and the last (goal)
// lane in the path.
double sum_lengths_between(
  const std::vector<RouteDistanceCalculator::PathLane>::const_iterator current_lane,
  const std::vector<RouteDistanceCalculator::PathLane> & path)
{
  double sum = 0.0;
  for (auto it = std::next(current_lane); it < std::prev(path.end()); ++it) {
    sum += it->length;
  }
  return sum;
}
}  // namespace

void RouteDistanceCalculator::set_map(const autoware_map_msgs::msg::LaneletMapBin & msg)
{
  lanelet_map_ptr_ = lanelet2_utils::remove_const(lanelet2_utils::from_autoware_map_msgs(msg));
  routing_graph_ptr_ = lanelet2_utils::remove_const(
    lanelet2_utils::instantiate_routing_graph_and_traffic_rules(lanelet_map_ptr_).first);
  road_lanes_ =
    lanelet::utils::query::roadLanelets(lanelet::utils::query::laneletLayer(lanelet_map_ptr_));
  is_map_ready_ = true;
}

void RouteDistanceCalculator::set_route(
  const autoware_planning_msgs::msg::LaneletRoute & msg, const geometry_msgs::msg::Pose & current_pose)
{
  goal_pose_ = msg.goal_pose;
  is_route_ready_ = false;
  shortest_path_lanes_.clear();

  if (!is_map_ready_) {
    return;
  }

  const auto current_lanelet = find_lanelet(road_lanes_, current_pose);
  const auto goal_lanelet = find_lanelet(road_lanes_, goal_pose_);
  if (!current_lanelet || !goal_lanelet) {
    return;
  }
  goal_lanelet_ = goal_lanelet.value();

  const auto route = routing_graph_ptr_->getRoute(current_lanelet.value(), goal_lanelet_, 0);
  if (!route) {
    return;
  }

  for (const auto & lanelet : route->shortestPath()) {
    shortest_path_lanes_.push_back({lanelet, lanelet::geometry::length2d(lanelet)});
  }
  is_route_ready_ = true;
}

std::optional<std::vector<RouteDistanceCalculator::PathLane>::const_iterator>
RouteDistanceCalculator::find_current_lane(const geometry_msgs::msg::Pose & current_pose) const
{
  lanelet::ConstLanelets lanelets;
  lanelets.reserve(shortest_path_lanes_.size());
  for (const auto & lane : shortest_path_lanes_) {
    lanelets.push_back(lane.lanelet);
  }

  const auto current_lanelet = find_lanelet(lanelets, current_pose);
  if (!current_lanelet) {
    return std::nullopt;
  }

  const auto it = std::find_if(
    shortest_path_lanes_.begin(), shortest_path_lanes_.end(),
    [&current_lanelet](const PathLane & lane) { return lane.lanelet.id() == current_lanelet->id(); });
  if (it == shortest_path_lanes_.end()) {
    return std::nullopt;
  }
  return it;
}

std::optional<double> RouteDistanceCalculator::calculate_remaining_distance(
  const geometry_msgs::msg::Pose & current_pose) const
{
  if (!is_route_ready_) {
    return std::nullopt;
  }

  const auto current_lane = find_current_lane(current_pose);
  if (!current_lane) {
    return std::nullopt;
  }
  const auto & current_lanelet = current_lane.value()->lanelet;

  const double arc_in_current_lanelet = arc_length_to(current_lanelet, current_pose);
  const double arc_in_goal_lanelet = arc_length_to(goal_lanelet_, goal_pose_);

  if (current_lanelet.id() == goal_lanelet_.id()) {
    return std::max(arc_in_goal_lanelet - arc_in_current_lanelet, 0.0);
  }

  const double remaining_in_current_lanelet =
    lanelet::geometry::length2d(current_lanelet) - arc_in_current_lanelet;
  const double remaining_in_middle_lanelets =
    sum_lengths_between(current_lane.value(), shortest_path_lanes_);

  return std::max(
    remaining_in_current_lanelet + remaining_in_middle_lanelets + arc_in_goal_lanelet, 0.0);
}

}  // namespace autoware::path_distance_calculator
