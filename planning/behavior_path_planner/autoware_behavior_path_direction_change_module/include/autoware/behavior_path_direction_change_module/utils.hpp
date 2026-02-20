// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__UTILS_HPP_

#include "autoware/behavior_path_direction_change_module/data_structs.hpp"

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <vector>

namespace autoware::route_handler
{
class RouteHandler;
}

namespace autoware::behavior_path_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

/**
 * @brief Detects cusp points in the path where direction changes occur
 * @param [in] path Input path to analyze
 * @param [in] angle_threshold_deg Maximum angle difference to consider as cusp (degrees)
 * @return Vector of cusp point indices in the path
 */
std::vector<size_t> detectCuspPoints(const PathWithLaneId & path, const double angle_threshold_deg);

/**
 * @brief Reverses path point orientations (yaw angles) at cusp points to indicate reverse direction
 * @param [in,out] path Path to modify with reversed orientations
 * @param [in] cusp_indices Indices where cusp points are located
 * @details Reverses yaw (adds π radians) for points after odd-numbered cusps to indicate reverse
 * segments
 */
void reverseOrientationAtCusps(PathWithLaneId * path, const std::vector<size_t> & cusp_indices);

/**
 * @brief Detects lane boundaries/transitions in the path
 * @param [in] path Current path with lane_ids
 * @return Vector of indices where lane transitions occur
 * @note Implementation postponed - placeholder for future enhancement
 */
std::vector<size_t> detectLaneBoundaries(const PathWithLaneId & path);

/**
 * @brief Build reference path from the full centerline of lanelets that have direction_change_lane.
 *        Used to avoid upstream path flip (nearest-point ambiguity) when the path crosses a single
 *        lanelet with multiple branches (e.g. cross with two cusps). No resampling is applied;
 *        the planner manager resamples the final path with output_path_interval.
 * @param [in] path Current path with lane_ids (used only to discover which lane_ids have the tag)
 * @param [in] route_handler Route handler to get centerline from map
 * @return PathWithLaneId Full centerline of direction_change lanelets in path order, or empty if
 * none
 */
PathWithLaneId getReferencePathFromDirectionChangeLanelets(
  const PathWithLaneId & path,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

/**
 * @brief Checks if a lanelet has the direction_change_lane tag set to "yes"
 * @param [in] lanelet Lanelet to check
 * @return True if direction_change_lane attribute is "yes", false otherwise
 */
bool hasDirectionChangeAreaTag(const lanelet::ConstLanelet & lanelet);

/**
 * @brief Safety check for lane continuity when exiting in reverse mode
 * @param [in] path Current path with lane_ids
 * @param [in] cusp_indices Detected cusp point indices
 * @param [in] route_handler Route handler to access lanelet map
 * @return True if safe (no fatal condition), false if fatal condition detected
 * @details Checks if odd number of cusps (reverse exit) conflicts with next lane without
 * direction_change_lane tag
 * @note This is a critical safety check to prevent fatal conditions where vehicle exits
 *       in reverse but next lane expects forward motion (Autoware default)
 */
bool checkLaneContinuitySafety(
  const PathWithLaneId & path, const std::vector<size_t> & cusp_indices,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

void densifyPathByYawAndDistance(
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const double max_yaw_step_rad,  // 例: 5 deg = 0.087 rad
  const double max_dist_step      // 例: 0.5 m
);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__UTILS_HPP_
