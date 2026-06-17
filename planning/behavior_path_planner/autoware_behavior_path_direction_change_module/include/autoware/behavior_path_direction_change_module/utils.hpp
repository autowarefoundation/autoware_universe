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
#include <set>
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
std::vector<size_t> getCuspPointIndices(
  const PathWithLaneId & path, const double angle_threshold_deg);

/**
 * @brief Detect cusp poses (x, y, orientation) from yaw discontinuities on @p path.
 */
std::vector<CuspPoint> detectCuspPointsFromPath(
  const PathWithLaneId & path, const double angle_threshold_deg);

/**
 * @brief Detect cusps on a path and store the path index on each CuspPoint.
 */
std::vector<CuspPoint> detectCuspPointsOnPathWithIndices(
  const PathWithLaneId & path, const double angle_threshold_deg);

/**
 * @brief True when two cusp poses refer to the same transition point.
 */
bool isSameCuspPoint(const CuspPoint & a, const CuspPoint & b, const double position_threshold_m);

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

/**
 * @brief Build route-ordered lanelet groups and the tagged-lane centerline from the mission route.
 */
std::optional<DirectionChangeRouteContext> buildDirectionChangeRouteContext(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

/**
 * @brief Keep path points whose lane_ids intersect @p target_lane_ids.
 */
PathWithLaneId extractPathPointsForLaneIds(
  const PathWithLaneId & path, const std::vector<int64_t> & target_lane_ids);

/**
 * @brief Build a centerline path through @p lane_ids in route order.
 */
PathWithLaneId buildCenterlinePathForLaneIds(
  const std::vector<int64_t> & lane_ids,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

/**
 * @brief Decide whether to prepend prefix lanes, use tagged centerline only, or append suffix lanes.
 */
ReferencePathAssemblyPhase determineReferencePathAssemblyPhase(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const geometry_msgs::msg::Pose & ego_pose, const DirectionChangeRouteContext & route_context,
  bool all_cusps_visited);

/**
 * @brief Combine prefix / tagged centerline / suffix based on assembly phase.
 */
PathWithLaneId assembleReferencePathWithLaneStitching(
  const DirectionChangeRouteContext & route_context, const PathWithLaneId & previous_module_path,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  ReferencePathAssemblyPhase assembly_phase);

/**
 * @brief Build prefix lane path from previous module output or lane centerlines.
 */
PathWithLaneId buildPrefixPathForStitching(
  const DirectionChangeRouteContext & route_context, const PathWithLaneId & previous_module_path,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

/**
 * @brief Trim path points behind the ego nearest index.
 */
PathWithLaneId cropPathFromEgo(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose);

/**
 * @brief Slice @p path between cusp indices on the tagged centerline.
 */
PathWithLaneId slicePathBetweenCuspIndices(
  const PathWithLaneId & path, const std::optional<size_t> start_after_cusp_index,
  size_t end_cusp_index);

/**
 * @brief Slice @p path from after @p start_after_cusp_index through @p goal_pose.
 */
PathWithLaneId slicePathToGoalFromCuspIndex(
  const PathWithLaneId & path, const std::optional<size_t> start_after_cusp_index,
  const geometry_msgs::msg::Pose & goal_pose);

/**
 * @brief Checks if a lanelet has the direction_change tag set to "yes"
 * @param [in] lanelet Lanelet to check
 * @return True if direction_change attribute is "yes", false otherwise
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

/**
 * @brief True when ego is at the route goal using position in the goal frame and route context.
 */
bool isEgoNearRouteGoal(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  double th_arrived_distance, const std::vector<int64_t> & suffix_lanelet_ids = {});

/**
 * @brief True when ego's closest route lanelet is one of the direction_change tagged lanes.
 */
bool isEgoOnTaggedLanelets(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const std::vector<int64_t> & tagged_lanelet_ids);

/**
 * @brief True when ego is on a route prefix lane before the tagged corridor.
 */
bool isEgoOnPrefixLanelets(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const std::vector<int64_t> & prefix_lanelet_ids);

/**
 * @brief Maneuver is finished: all cusps handled and ego left tagged area or reached goal.
 */
bool isDirectionChangeManeuverFinished(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const DirectionChangeRouteContext & route_context, double th_arrived_distance,
  bool all_cusps_visited);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__UTILS_HPP_
