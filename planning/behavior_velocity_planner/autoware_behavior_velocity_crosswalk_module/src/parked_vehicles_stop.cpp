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

#include "parked_vehicles_stop.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <boost/geometry/algorithms/correct.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/utility/Utilities.h>

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace
{
/// @brief concatenate the left and right bounds of the given lanelet sequence
std::pair<lanelet::BasicLineString2d, lanelet::BasicLineString2d> get_concatenated_bounds(
  const lanelet::ConstLanelets & lanelets)
{
  lanelet::BasicLineString2d concatenated_left_bound;
  lanelet::BasicLineString2d concatenated_right_bound;
  for (auto & ll : lanelets) {
    auto left_bound = ll.leftBound2d().basicLineString();
    auto left_bound_begin =
      concatenated_left_bound.empty() ? left_bound.begin() : std::next(left_bound.begin());
    concatenated_left_bound.insert(
      concatenated_left_bound.end(), left_bound_begin, left_bound.end());
    auto right_bound = ll.rightBound2d().basicLineString();
    auto right_bound_begin =
      concatenated_right_bound.empty() ? right_bound.begin() : std::next(right_bound.begin());
    concatenated_right_bound.insert(
      concatenated_right_bound.end(), right_bound_begin, right_bound.end());
  }
  return {concatenated_left_bound, concatenated_right_bound};
}

/// @brief get the crosswalk lanelet bound (left/right) that will first be crossed by the ego
/// vehicle
/// @warning assumes the sides of the crosswalk lanelet that are perpendicular to the road only have
/// two points
lanelet::BasicSegment2d get_nearest_crosswalk_bound(
  const lanelet::ConstLanelet & crosswalk_lanelet,
  const geometry_msgs::msg::Point & first_path_point_on_crosswalk)
{
  const auto end_search_point =
    lanelet::BasicPoint2d(first_path_point_on_crosswalk.x, first_path_point_on_crosswalk.y);
  return lanelet::geometry::closestSegment(
    crosswalk_lanelet.polygon2d().basicPolygon(), end_search_point);
}

std::pair<lanelet::BasicPoint2d, lanelet::BasicPoint2d> get_extreme_crosswalk_bound_points(
  const lanelet::BasicLineString2d & path_bound, const lanelet::BasicSegment2d & crosswalk_bound)
{
  const auto dist_front =
    lanelet::geometry::toArcCoordinates(path_bound, crosswalk_bound.first).distance;
  const auto dist_back =
    lanelet::geometry::toArcCoordinates(path_bound, crosswalk_bound.second).distance;
  const auto left_end_point =
    dist_front > dist_back ? crosswalk_bound.first : crosswalk_bound.second;
  const auto right_end_point =
    dist_front <= dist_back ? crosswalk_bound.first : crosswalk_bound.second;
  return {left_end_point, right_end_point};
}

/// @brief expand a bound until the given point
lanelet::BasicLineString2d expand_bound_until_point(
  const lanelet::BasicLineString2d & bound, const lanelet::BasicPoint2d & point)
{
  const auto lateral_distance = lanelet::geometry::toArcCoordinates(bound, point).distance;
  const auto expanded_bound = lanelet::geometry::offsetNoThrow(bound, lateral_distance);
  return expanded_bound;
}

lanelet::BasicLineString2d create_bound_subset(
  const lanelet::BasicLineString2d & bound, const lanelet::BasicPoint2d & end_point,
  const double length)
{
  const auto end_arc_coordinates = lanelet::geometry::toArcCoordinates(bound, end_point);
  const auto start_arc_length = std::max(end_arc_coordinates.length - length, 0.0);
  lanelet::BasicLineString2d subset_bound;
  auto arc_length = 0.0;
  for (auto i = 0UL; i + 1 < bound.size(); ++i) {
    const auto & p1 = bound[i];
    const auto & p2 = bound[i + 1];
    arc_length += lanelet::geometry::distance2d(p1, p2);
    if (arc_length > end_arc_coordinates.length) {
      break;
    }
    if (arc_length >= start_arc_length) {
      subset_bound.emplace_back(p2);
    }
  }
  const auto first_point = lanelet::geometry::fromArcCoordinates(bound, {start_arc_length, 0.0});
  if (
    subset_bound.empty() ||
    lanelet::geometry::distance2d(subset_bound.front(), first_point) > 1e-3) {
    subset_bound.insert(subset_bound.begin(), first_point);
  }
  if (lanelet::geometry::distance2d(subset_bound.back(), end_point) > 1e-3) {
    subset_bound.emplace_back(end_point);
  }
  return subset_bound;
}

}  // namespace

lanelet::BasicPolygon2d create_search_area(
  const lanelet::ConstLanelet & crosswalk_lanelet, const lanelet::ConstLanelets & path_lanelets,
  const geometry_msgs::msg::Point & first_path_point_on_crosswalk, const double search_distance)
{
  const auto [path_left_bound, path_right_bound] = get_concatenated_bounds(path_lanelets);
  const auto nearest_crosswalk_bound =
    get_nearest_crosswalk_bound(crosswalk_lanelet, first_path_point_on_crosswalk);
  const auto [left_crosswalk_point, right_crosswalk_point] =
    get_extreme_crosswalk_bound_points(path_left_bound, nearest_crosswalk_bound);
  const auto expanded_left_bound = expand_bound_until_point(path_left_bound, left_crosswalk_point);
  const auto expanded_right_bound =
    expand_bound_until_point(path_right_bound, right_crosswalk_point);
  const auto search_area_left_bound =
    create_bound_subset(expanded_left_bound, left_crosswalk_point, search_distance);
  const auto search_area_right_bound =
    create_bound_subset(expanded_right_bound, right_crosswalk_point, search_distance);
  // create polygon from the left and right bounds
  lanelet::BasicPolygon2d search_area(search_area_left_bound);
  for (auto it = search_area_right_bound.rbegin(); it != search_area_right_bound.rend(); ++it) {
    search_area.emplace_back(*it);
  }
  boost::geometry::correct(search_area);
  return search_area;
}

std::pair<std::optional<autoware_perception_msgs::msg::PredictedObject>, geometry_msgs::msg::Point>
calculate_furthest_parked_vehicle(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & ego_path,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & parked_vehicles,
  const lanelet::BasicPolygon2d & search_area)
{
  std::optional<autoware_perception_msgs::msg::PredictedObject> furthest_vehicle;
  geometry_msgs::msg::Point furthest_point;
  double furthest_parked_object_arc_length = 0.0;
  const auto & update_search = [&](const auto & p, const auto & v) {
    const auto pt = geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y());
    const auto arc_length = motion_utils::calcSignedArcLength(ego_path, 0UL, pt);
    if (arc_length > furthest_parked_object_arc_length) {
      furthest_parked_object_arc_length = arc_length;
      furthest_vehicle = v;
      furthest_point = pt;
    }
  };
  for (const auto & parked_vehicle : parked_vehicles) {
    const auto footprint = autoware_utils_geometry::to_polygon2d(parked_vehicle);
    if (boost::geometry::disjoint(footprint, search_area)) {
      continue;
    }
    for (const auto & p : footprint.outer()) {
      update_search(p, parked_vehicle);
    }
  }
  return std::make_pair(furthest_vehicle, furthest_point);
}

bool is_planning_to_stop_in_search_area(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & path,
  const size_t ego_idx, const lanelet::BasicPolygon2d & search_area)
{
  for (auto idx = ego_idx + 1; idx < path.size(); ++idx) {
    if (
      path[idx].point.longitudinal_velocity_mps == 0.0 &&
      boost::geometry::within(
        lanelet::BasicPoint2d(path[idx].point.pose.position.x, path[idx].point.pose.position.y),
        search_area)) {
      return true;
    }
  }
  return false;
}

std::optional<tier4_planning_msgs::msg::StopFactor> calculate_parked_vehicles_stop_factor(
  const std::vector<std::optional<geometry_msgs::msg::Pose>> & candidate_stop_poses,
  const std::optional<geometry_msgs::msg::Pose> & previous_stop_pose,
  const std::optional<double> min_stop_distance,
  const std::function<double(geometry_msgs::msg::Pose)> & calc_distance_fn)
{
  auto min_distance = std::numeric_limits<double>::max();
  geometry_msgs::msg::Pose min_pose;
  bool found = false;
  const auto & update_min_pose = [&](const auto & pose) {
    if (pose) {
      const auto dist_to_pose = calc_distance_fn(*pose);
      const auto is_feasible = (!min_stop_distance || dist_to_pose >= min_stop_distance);
      if (dist_to_pose < min_distance && is_feasible) {
        min_distance = dist_to_pose;
        min_pose = *pose;
        found = true;
      }
    }
  };
  for (const auto & pose : candidate_stop_poses) {
    update_min_pose(pose);
  }
  // previous stop pose is reused without caring about the minimum stop distance
  update_min_pose(previous_stop_pose);
  std::optional<tier4_planning_msgs::msg::StopFactor> stop_factor;
  if (found) {
    stop_factor.emplace();
    stop_factor->stop_pose = min_pose;
    stop_factor->dist_to_stop_pose = min_distance;
  }
  return stop_factor;
}

std::vector<autoware_perception_msgs::msg::PredictedObject> get_parked_vehicles(
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects,
  const double parked_velocity_threshold,
  const std::function<bool(autoware_perception_msgs::msg::PredictedObject)> & is_vehicle_fn)
{
  std::vector<autoware_perception_msgs::msg::PredictedObject> vehicles;
  for (const auto & object : objects) {
    if (
      is_vehicle_fn(object) &&
      object.kinematics.initial_twist_with_covariance.twist.linear.x <= parked_velocity_threshold) {
      vehicles.push_back(object);
    }
  }
  return vehicles;
}

void update_previous_stop_pose(
  std::optional<geometry_msgs::msg::Pose> & previous_stop_pose,
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & path)
{
  if (!previous_stop_pose) {
    return;
  }
  const auto stop_arc_length =
    motion_utils::calcSignedArcLength(path, 0UL, previous_stop_pose->position);
  previous_stop_pose = motion_utils::calcInterpolatedPose(path, stop_arc_length);
}
}  // namespace autoware::behavior_velocity_planner
