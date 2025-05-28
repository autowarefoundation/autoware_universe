// Copyright 2024 Tier IV, Inc.
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

#ifndef OCCLUDED_CROSSWALK_HPP_
#define OCCLUDED_CROSSWALK_HPP_

#include "scene_crosswalk.hpp"

#include <grid_map_core/GridMap.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

#include <vector>

namespace autoware::behavior_velocity_planner
{
/// @brief check if the gridmap is occluded at the given index
/// @param [in] grid_map input grid map
/// @param [in] min_nb_of_cells minimum number of occluded cells needed to detect an occlusion (as
/// side of a square centered at the index)
/// @param [in] idx target index in the grid map
/// @param [in] params parameters
/// @return true if the index is occluded
bool is_occluded(
  const grid_map::GridMap & grid_map, const int min_nb_of_cells, const grid_map::Index & idx,
  const autoware::behavior_velocity_planner::CrosswalkModule::PlannerParam & params);

/// @brief interpolate a point beyond the end of the given segment
/// @param [in] segment input segment
/// @param [in] extra_distance desired distance beyond the end of the segment
/// @return interpolated point beyond the end of the segment
lanelet::BasicPoint2d interpolate_point(
  const lanelet::BasicSegment2d & segment, const double extra_distance);

/// @brief check if the crosswalk is occluded
/// @param occupancy_grid occupancy grid with the occlusion information
/// @param detection_areas areas to check for occlusions
/// @param dynamic_objects dynamic objects
/// @param params parameters
/// @return true if the crosswalk is occluded
bool is_crosswalk_occluded(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const std::vector<lanelet::BasicPolygon2d> & detection_areas,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & dynamic_objects,
  const autoware::behavior_velocity_planner::CrosswalkModule::PlannerParam & params);

/// @brief calculate the distance away from the crosswalk that should be checked for occlusions
/// @param occluded_objects_velocity assumed velocity of the objects coming out of occlusions
/// @param dist_ego_to_crosswalk distance between ego and the crosswalk
/// @param ego_velocity current velocity of ego
double calculate_detection_range(
  const double occluded_object_velocity, const double dist_ego_to_crosswalk,
  const double ego_velocity);

/// @brief select a subset of objects meeting the velocity threshold and inflate their shape
/// @param objects input objects
/// @param velocity_threshold minimum velocity for an object to be selected
/// @param skip_pedestrians if true, pedestrians are not selected regardless of their velocities
/// @param inflate_size [m] size by which the shape of the selected objects are inflated
/// @return selected and inflated objects
std::vector<autoware_perception_msgs::msg::PredictedObject> select_and_inflate_objects(
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects,
  const double velocity_threshold, const bool skip_pedestrians, const double inflate_size);

/// @brief clear occlusions behind the given objects
/// @details masks behind the object assuming rays from the center of the grid map
/// @param grid_map grid map
/// @param objects objects
void clear_occlusions_behind_objects(
  grid_map::GridMap & grid_map,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects);

/// @brief calculate areas to check for occlusions around the given crosswalk
/// @param crosswalk_lanelet crosswalk lanelet
/// @param crosswalk_origin crosswalk point from which to calculate the distances
/// @param detection_range [m] desired distance from the crosswalk origin
/// @return detection areas within the detection range of the crosswalk
std::vector<lanelet::BasicPolygon2d> calculate_detection_areas(
  const lanelet::ConstLanelet & crosswalk_lanelet, const lanelet::BasicPoint2d & crosswalk_origin,
  const double detection_range);
}  // namespace autoware::behavior_velocity_planner

#endif  // OCCLUDED_CROSSWALK_HPP_
