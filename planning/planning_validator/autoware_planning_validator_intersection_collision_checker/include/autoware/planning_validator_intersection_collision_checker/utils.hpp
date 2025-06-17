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

#ifndef AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__UTILS_HPP_
#define AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__UTILS_HPP_

#include "autoware/planning_validator_intersection_collision_checker/types.hpp"

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <limits>

namespace autoware::planning_validator::collision_checker_utils
{

TrajectoryPoints trim_trajectory_points(
  const TrajectoryPoints & trajectory_points, const geometry_msgs::msg::Pose & start_pose);

void set_trajectory_lanelets(
  const TrajectoryPoints & trajectory_points, const RouteHandler & route_handler,
  const geometry_msgs::msg::Pose & ego_pose, CollisionCheckerLanelets & lanelets);

void set_right_turn_target_lanelets(
  const EgoTrajectory & ego_traj, const RouteHandler & route_handler,
  const CollisionCheckerParams & params, CollisionCheckerLanelets & lanelets,
  const double time_horizon = std::numeric_limits<double>::max());

void set_left_turn_target_lanelets(
  const EgoTrajectory & ego_traj, const RouteHandler & route_handler,
  const CollisionCheckerParams & params, CollisionCheckerLanelets & lanelets,
  const double time_horizon = std::numeric_limits<double>::max());

}  // namespace autoware::planning_validator::collision_checker_utils

#endif  // AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__UTILS_HPP_
