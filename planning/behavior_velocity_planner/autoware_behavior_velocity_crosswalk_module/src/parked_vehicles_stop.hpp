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

#ifndef PARKED_VEHICLES_STOP_HPP_
#define PARKED_VEHICLES_STOP_HPP_

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tier4_planning_msgs/msg/stop_factor.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
/// @brief create a search area ahead of a crosswalk by the given distance
/// @param [in] crosswalk_lanelet lanelet of the target crosswalk
/// @param [in] path_lanelets sequence of path lanelets
/// @param [in] first_path_point_on_crosswalk first path point on the crosswalk
/// @param [in] search_distance [m] distance ahead of the crosswalk to create the search area
/// @return the search area ahead of the crosswalk
lanelet::BasicPolygon2d create_search_area(
  const lanelet::ConstLanelet & crosswalk_lanelet, const lanelet::ConstLanelets & path_lanelets,
  const geometry_msgs::msg::Point & first_path_point_on_crosswalk, const double search_distance);
/// @brief calculate the furthest parked vehicle inside the search area and its corresponding
/// footprint point
/// @param [in] ego_path ego path to calculate arc lengths of the footprint points
/// @param [in] parked_vehicles all detected parked vehicles
/// @param [in] search_area search area
/// @return pair of an optional object and corresponding furthest footprint point inside the search
/// area
std::pair<std::optional<autoware_perception_msgs::msg::PredictedObject>, geometry_msgs::msg::Point>
calculate_furthest_parked_vehicle(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & ego_path,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & parked_vehicles,
  const lanelet::BasicPolygon2d & search_area);
/// @brief determine if the path already plans to stop inside the search area
/// @param [in] path ego path
/// @param [in] ego_idx ego index in the path
/// @param [in] search_area search area
/// @return true if a stopped point after the ego index was found inside the search area
bool is_planning_to_stop_in_search_area(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & path,
  const size_t ego_idx, const lanelet::BasicPolygon2d & search_area);

/// @brief calculate the stop factor based on the candidate and previous stop poses
/// @param [in] candidate_stop_poses candidate stop poses
/// @param [in] previous_stop_pose previous stop pose calculate for parked vehicles
/// @param [in] min_stop_distance [m] minimum distance where we can comfortably stop
/// @param [in] calc_distance_fn function to calculate the distance of a pose
/// @return calculated stop factor
std::optional<tier4_planning_msgs::msg::StopFactor> calculate_parked_vehicles_stop_factor(
  const std::vector<std::optional<geometry_msgs::msg::Pose>> & candidate_stop_poses,
  const std::optional<geometry_msgs::msg::Pose> & previous_stop_pose,
  const std::optional<double> min_stop_distance,
  const std::function<double(geometry_msgs::msg::Pose)> & calc_distance_fn);

/// @brief get the parked vehicles from the given objects
/// @param [in] objects predicted objects
/// @param [in] parked_velocity_threshold [m/s] velocity under which an object is considered parked
/// @param [in] is_vehicle_fn function to determine if an object is a vehicle
/// @return objects that are parked vehicles
std::vector<autoware_perception_msgs::msg::PredictedObject> get_parked_vehicles(
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects,
  const double parked_velocity_threshold,
  const std::function<bool(autoware_perception_msgs::msg::PredictedObject)> & is_vehicle_fn);

/// @brief update the previous stop pose to adapt to changes in the path shape
/// @param [inout] previous_stop_pose the stop pose to update
/// @param [in] path ego path
void update_previous_stop_pose(
  std::optional<geometry_msgs::msg::Pose> & previous_stop_pose,
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & path);
}  // namespace autoware::behavior_velocity_planner

#endif  // PARKED_VEHICLES_STOP_HPP_
