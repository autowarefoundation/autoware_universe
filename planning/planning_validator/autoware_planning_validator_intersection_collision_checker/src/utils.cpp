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

#include "autoware/planning_validator_intersection_collision_checker/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/strategies/cartesian/buffer_point_square.hpp>

#include <algorithm>
#include <utility>

namespace autoware::planning_validator::collision_checker_utils
{

namespace
{
bool contains_lanelet(const lanelet::ConstLanelets & lanelets, const lanelet::Id id)
{
  return std::find_if(lanelets.begin(), lanelets.end(), [&](const auto & l) {
           return l.id() == id;
         }) != lanelets.end();
};
}  // namespace

TrajectoryPoints trim_trajectory_points(
  const TrajectoryPoints & trajectory_points, const geometry_msgs::msg::Pose & start_pose)
{
  const auto nearest_idx =
    autoware::motion_utils::findNearestIndex(trajectory_points, start_pose.position);
  return TrajectoryPoints(trajectory_points.begin() + nearest_idx, trajectory_points.end());
}

void set_trajectory_lanelets(
  const TrajectoryPoints & trajectory_points, const RouteHandler & route_handler,
  const geometry_msgs::msg::Pose & ego_pose, CollisionCheckerLanelets & lanelets)
{
  lanelet::ConstLanelet closest_lanelet;
  if (!route_handler.getClosestLaneletWithinRoute(ego_pose, &closest_lanelet)) {
    throw std::logic_error("failed to get closest lanelet within route");
  }

  const auto forward_trajectory_length = autoware::motion_utils::calcSignedArcLength(
    trajectory_points, ego_pose.position, trajectory_points.size() - 1);

  lanelets.trajectory_lanelets =
    route_handler.getLaneletSequence(closest_lanelet, ego_pose, 0.0, forward_trajectory_length);

  lanelet::ConstLanelets prev_lanelets;
  if (route_handler.getPreviousLaneletsWithinRoute(closest_lanelet, &prev_lanelets)) {
    lanelets.connected_lanelets.push_back(prev_lanelets.front());
    for (const auto & connected_ll : route_handler.getNextLanelets(prev_lanelets.front())) {
      lanelets.connected_lanelets.push_back(connected_ll);
    }
  }

  for (const auto & ll : lanelets.trajectory_lanelets) {
    for (const auto & connected_ll : route_handler.getNextLanelets(ll)) {
      lanelets.connected_lanelets.push_back(connected_ll);
    }
  }
}

std::optional<std::pair<size_t, size_t>> get_overlap_index(
  const lanelet::ConstLanelet & ll, const TrajectoryPoints & trajectory_points,
  const autoware_utils::LineString2d & trajectory_ls)
{
  BasicLineString2d overlap_line;
  boost::geometry::intersection(ll.polygon2d().basicPolygon(), trajectory_ls, overlap_line);
  if (overlap_line.empty()) return {};

  const auto nearest_idx_front = autoware::motion_utils::findNearestIndex(
    trajectory_points,
    geometry_msgs::msg::Point().set__x(overlap_line.front()[0]).set__y(overlap_line.front()[1]));
  const auto nearest_idx_back = autoware::motion_utils::findNearestIndex(
    trajectory_points,
    geometry_msgs::msg::Point().set__x(overlap_line.back()[0]).set__y(overlap_line.back()[1]));

  if (nearest_idx_back > nearest_idx_front) return {{nearest_idx_front, nearest_idx_back}};

  return {{nearest_idx_back, nearest_idx_front}};
}

void set_right_turn_target_lanelets(
  const EgoTrajectory & ego_traj, const RouteHandler & route_handler,
  const CollisionCheckerParams & params, CollisionCheckerLanelets & lanelets,
  const double time_horizon)
{
  autoware_utils::LineString2d trajectory_ls;
  for (const auto & p : ego_traj.front_traj) {
    trajectory_ls.emplace_back(p.pose.position.x, p.pose.position.y);
  }

  auto is_road = [](const lanelet::ConstLanelet & ll) {
    return ll.hasAttribute(lanelet::AttributeName::Subtype) &&
           ll.attribute(lanelet::AttributeName::Subtype).value() ==
             lanelet::AttributeValueString::Road;
  };

  auto ignore_turning = [&params](const lanelet::ConstLanelet & ll) {
    if (!ll.hasAttribute("turn_direction")) return false;
    if (ll.attribute("turn_direction") == "straight") return false;
    return !params.right_turn.check_turning_lanes;
  };

  auto extend_lanelet =
    [&](const lanelet::ConstLanelet & ll, const geometry_msgs::msg::Pose & overlap_point) {
      const auto distance_th = params.detection_range;
      lanelet::ConstLanelets extended_lanelets{ll};
      auto current_arc_length =
        lanelet::utils::getArcCoordinates(extended_lanelets, overlap_point).length;
      if (current_arc_length >= distance_th) return extended_lanelets;

      while (current_arc_length < distance_th) {
        const auto prev_lanelets = route_handler.getPreviousLanelets(ll);
        if (prev_lanelets.empty()) break;  // No more previous lanelets to extend
        extended_lanelets.push_back(prev_lanelets.front());
        current_arc_length += lanelet::utils::getLaneletLength2d(prev_lanelets.front());
      }
      std::reverse(extended_lanelets.begin(), extended_lanelets.end());
      return extended_lanelets;
    };

  const auto lanelet_map_ptr = route_handler.getLaneletMapPtr();
  const auto candidates = lanelet_map_ptr->laneletLayer.search(
    boost::geometry::return_envelope<lanelet::BoundingBox2d>(trajectory_ls));
  for (const auto & ll : candidates) {
    const auto id = ll.id();
    if (
      !is_road(ll) || ignore_turning(ll) || contains_lanelet(lanelets.trajectory_lanelets, id) ||
      contains_lanelet(lanelets.connected_lanelets, id))
      continue;

    const auto overlap_index = get_overlap_index(ll, ego_traj.front_traj, trajectory_ls);
    if (!overlap_index || overlap_index->first < ego_traj.front_index) continue;
    const auto mid_idx = (overlap_index->first + overlap_index->second) / 2;
    const auto overlap_point =
      lanelet::utils::getClosestCenterPose(ll, ego_traj.front_traj[mid_idx].pose.position);
    std::pair<double, double> overlap_time;
    overlap_time.first =
      rclcpp::Duration(ego_traj.front_traj[overlap_index->first].time_from_start).seconds();
    overlap_time.second =
      rclcpp::Duration(ego_traj.back_traj[overlap_index->second].time_from_start).seconds();
    if (overlap_time.first > time_horizon) continue;
    lanelets.target_lanelets.emplace_back(
      ll.id(), extend_lanelet(ll, overlap_point), overlap_point, overlap_time);
  }
}

void set_left_turn_target_lanelets(
  const EgoTrajectory & ego_traj, const RouteHandler & route_handler,
  const CollisionCheckerParams & params, CollisionCheckerLanelets & lanelets,
  const double time_horizon)
{
  std::optional<lanelet::ConstLanelet> turn_lanelet;
  for (const auto & lanelet : lanelets.trajectory_lanelets) {
    if (!lanelet.hasAttribute("turn_direction")) {
      if (turn_lanelet) break;  // If we already found a turn lanelet, we can stop checking
      continue;
    }
    if (lanelet.attribute("turn_direction") == "left") {
      turn_lanelet = lanelet;
    } else if (turn_lanelet) {
      break;
    }
  }

  if (!turn_lanelet) return;

  lanelet::ConstLanelet next_lanelet;
  if (!route_handler.getNextLaneletWithinRoute(*turn_lanelet, &next_lanelet)) return;

  autoware_utils::LineString2d trajectory_ls;
  for (const auto & p : ego_traj.front_traj) {
    trajectory_ls.emplace_back(p.pose.position.x, p.pose.position.y);
  }

  auto ignore_turning = [&params](const lanelet::ConstLanelet & ll) {
    if (!ll.hasAttribute("turn_direction")) return false;
    if (ll.attribute("turn_direction") == "straight") return false;
    return !params.left_turn.check_turning_lanes;
  };

  auto extend_lanelet =
    [&](const lanelet::ConstLanelet & ll, const geometry_msgs::msg::Pose & overlap_point) {
      const auto distance_th = params.detection_range;
      lanelet::ConstLanelets extended_lanelets{ll};
      auto current_arc_length =
        lanelet::utils::getArcCoordinates(extended_lanelets, overlap_point).length;
      if (current_arc_length >= distance_th) return extended_lanelets;

      while (current_arc_length < distance_th) {
        const auto prev_lanelets = route_handler.getPreviousLanelets(ll);
        if (prev_lanelets.empty()) break;  // No more previous lanelets to extend
        extended_lanelets.push_back(prev_lanelets.front());
        current_arc_length += lanelet::utils::getLaneletLength2d(prev_lanelets.front());
      }
      std::reverse(extended_lanelets.begin(), extended_lanelets.end());
      return extended_lanelets;
    };

  const auto turn_lanelet_id = turn_lanelet->id();
  for (const auto & lanelet : route_handler.getPreviousLanelets(next_lanelet)) {
    if (lanelet.id() == turn_lanelet_id || ignore_turning(lanelet)) continue;
    const auto overlap_index = get_overlap_index(lanelet, ego_traj.front_traj, trajectory_ls);
    if (!overlap_index || overlap_index->first < ego_traj.front_index) continue;
    const auto overlap_point = lanelet::utils::getClosestCenterPose(
      lanelet, ego_traj.front_traj[overlap_index->first].pose.position);
    std::pair<double, double> overlap_time;
    overlap_time.first =
      rclcpp::Duration(ego_traj.front_traj[overlap_index->first].time_from_start).seconds();
    overlap_time.second =
      rclcpp::Duration(ego_traj.back_traj[overlap_index->first].time_from_start).seconds();
    if (overlap_time.first > time_horizon) continue;
    lanelets.target_lanelets.emplace_back(
      lanelet.id(), extend_lanelet(lanelet, overlap_point), overlap_point, overlap_time);
  }
}

}  // namespace autoware::planning_validator::collision_checker_utils
