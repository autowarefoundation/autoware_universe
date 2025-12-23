// Copyright 2025 Tier IV, Inc.
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

#ifndef EXPERIMENTAL__UTILS_HPP_
#define EXPERIMENTAL__UTILS_HPP_

#include "../grid_utils.hpp"
#include "../occlusion_spot_utils.hpp"

#include <autoware/trajectory/path_point_with_lane_id.hpp>

#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
namespace utils
{
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using Trajectory = autoware::experimental::trajectory::Trajectory<PathPointWithLaneId>;

// apply current velocity to path
void applyVelocityToPath(Trajectory & path, const double velocity);

//!< @brief wrapper for detection area polygon generation
bool buildDetectionAreaPolygons(
  Polygons2d & polygons, const Trajectory & path, const geometry_msgs::msg::Pose & target_pose,
  const size_t target_seg_idx, const occlusion_spot_utils::PlannerParam & param);

//!< @brief generate possible collisions coming from occlusion spots on the side of the path
bool generatePossibleCollisionsFromGridMap(
  std::vector<occlusion_spot_utils::PossibleCollisionInfo> & possible_collisions,
  const grid_map::GridMap & grid, const Trajectory & path, const double offset_from_start_to_ego,
  const occlusion_spot_utils::PlannerParam & param, occlusion_spot_utils::DebugData & debug_data);

bool generatePossibleCollisionsFromObjects(
  std::vector<occlusion_spot_utils::PossibleCollisionInfo> & possible_collisions,
  const Trajectory & path, const occlusion_spot_utils::PlannerParam & param,
  const double offset_from_start_to_ego, const std::vector<PredictedObject> & dyn_objects);

void calcSlowDownPointsForPossibleCollision(
  const Trajectory & path, const double offset,
  std::vector<occlusion_spot_utils::PossibleCollisionInfo> & possible_collisions);

void applySafeVelocityConsideringPossibleCollision(
  Trajectory & path, std::vector<occlusion_spot_utils::PossibleCollisionInfo> & possible_collisions,
  std::vector<geometry_msgs::msg::Pose> & debug_poses,
  const occlusion_spot_utils::PlannerParam & param);
}  // namespace utils
}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__UTILS_HPP_
