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
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>

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

std::vector<CuspPoint> detectCuspPointsOnPathWithIndices(
  const PathWithLaneId & path, double angle_threshold_deg);

double calcDistanceAlongPathToPose(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose,
  const geometry_msgs::msg::Pose & target_pose);

double calcDistanceToPathEnd(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose);

std::optional<DirectionChangeRouteContext> buildDirectionChangeRouteContext(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

PathWithLaneId buildPathForLaneIds(
  const PathWithLaneId & previous_module_path, const std::vector<int64_t> & lane_ids,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

ReferencePathAssemblyPhase determineReferencePathAssemblyPhase(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const geometry_msgs::msg::Pose & ego_pose, const DirectionChangeRouteContext & route_context,
  bool all_cusps_visited);

PathWithLaneId assembleReferencePathWithLaneStitching(
  const DirectionChangeRouteContext & route_context, const PathWithLaneId & previous_module_path,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  ReferencePathAssemblyPhase assembly_phase);

PathWithLaneId slicePathBetweenCuspIndices(
  const PathWithLaneId & path, const std::optional<size_t> start_after_cusp_index,
  size_t end_cusp_index);

PathWithLaneId slicePathToGoalFromCuspIndex(
  const PathWithLaneId & path, const std::optional<size_t> start_after_cusp_index,
  const geometry_msgs::msg::Pose & goal_pose);

bool hasDirectionChangeAreaTag(const lanelet::ConstLanelet & lanelet);

void flipPathPointOrientation(PathWithLaneId & path);

void setPathPointVelocityToZero(PathWithLaneId & path, size_t point_count = 1);

void clipPathAroundEgo(
  PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose,
  double backward_path_length, double forward_path_length);

std::optional<PathWithLaneId> applyGoalLateralShift(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & goal_pose,
  const DirectionChangeParameters & parameters,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

bool isEgoNearRouteGoal(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  double th_arrived_distance, const std::vector<int64_t> & suffix_lanelet_ids = {});

bool isEgoOnRouteLanelets(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const std::vector<int64_t> & lanelet_ids);

bool isDirectionChangeManeuverFinished(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const DirectionChangeRouteContext & route_context, double th_arrived_distance,
  bool all_cusps_visited);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__UTILS_HPP_
