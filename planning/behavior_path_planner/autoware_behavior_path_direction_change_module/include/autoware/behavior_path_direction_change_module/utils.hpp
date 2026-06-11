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

#ifndef AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__UTILS_HPP_

#include "autoware/behavior_path_direction_change_module/data_structs.hpp"

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::route_handler
{
class RouteHandler;
}

namespace autoware::behavior_path_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

/**
 * @brief Detects cusp points in the path where direction changes occur (legacy geometric fallback).
 */
std::vector<size_t> getCuspPointIndices(const PathWithLaneId & path, const double angle_threshold_deg);

/**
 * @brief Detect cusp poses (x, y, orientation) from yaw discontinuities on @p path.
 */
std::vector<CuspPoint> detectCuspPointsFromPath(
  const PathWithLaneId & path, const double angle_threshold_deg);

/**
 * @brief True when two cusp poses refer to the same transition point.
 */
bool isSameCuspPoint(
  const CuspPoint & a, const CuspPoint & b, const double position_threshold_m);

/**
 * @brief Append newly visible cusp poses ahead of ego; skip duplicates already tracked.
 */
void mergeNewCuspPointsAheadOfEgo(
  std::vector<CuspPoint> & tracked_cusp_points, const std::vector<CuspPoint> & detected_cusp_points,
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose,
  const double dedup_distance_m);

/**
 * @brief Distance from ego to @p target_pose along @p path, or Euclidean if target is off-path.
 */
double calcDistanceAlongPathToPose(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose,
  const geometry_msgs::msg::Pose & target_pose);

PathWithLaneId getReferencePathFromDirectionChangeLanelets(
  const PathWithLaneId & path,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const DirectionChangeParameters & parameters);

/**
 * @brief Checks if a lanelet has the direction_change_lane tag set to "yes"
 * @param [in] lanelet Lanelet to check
 * @return True if direction_change_lane attribute is "yes", false otherwise
 */
bool hasDirectionChangeAreaTag(const lanelet::ConstLanelet & lanelet);

/**
 * @brief True if ego heading aligns with the reference path orientation at the nearest point.
 * @details Used to set initial forward/reverse maneuver state without map maneuver_direction tags.
 */
bool isEgoDrivingForwardWrtLane(
  const geometry_msgs::msg::Pose & ego_pose, const PathWithLaneId & reference_path);

/**
 * @brief Signed arc length from ego nearest point to the end of the path.
 */
double calcDistanceToPathEnd(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose);

/**
 * @brief Flip path yaw by pi and negate per-point reference speeds for reverse driving.
 */
void flipPathPointOrientation(PathWithLaneId & path);

/**
 * @brief Set the last @p point_count path point velocities to zero.
 */
void setPathPointVelocityToZero(PathWithLaneId & path, size_t point_count = 1);

std::optional<PathWithLaneId> applyGoalLateralShift(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & goal_pose,
  const DirectionChangeParameters & parameters,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__UTILS_HPP_
