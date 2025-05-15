// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_SEARCHER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_SEARCHER_HPP_

#include "autoware/behavior_path_goal_planner_module/goal_candidate.hpp"
#include "autoware/behavior_path_goal_planner_module/goal_planner_parameters.hpp"
#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/utils/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware_utils::LinearRing2d;
using BasicPolygons2d = std::vector<lanelet::BasicPolygon2d>;
using autoware_utils::MultiPolygon2d;

class GoalSearcher
{
public:
  static GoalSearcher create(
    const GoalPlannerParameters & parameters, const LinearRing2d & vehicle_footprint,
    const std::shared_ptr<const PlannerData> & planner_data);

public:
  GoalCandidates search(
    const std::shared_ptr<const PlannerData> & planner_data, const bool use_bus_stop_area);
  void update(
    GoalCandidates & goal_candidates,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    const std::shared_ptr<const PlannerData> & planner_data,
    const PredictedObjects & objects) const;

  // todo(kosuke55): Functions for this specific use should not be in the interface,
  // so it is better to consider interface design when we implement other goal searchers.
  std::optional<GoalCandidate> getClosestGoalCandidateAlongLanes(
    const GoalCandidates & goal_candidates,
    const std::shared_ptr<const PlannerData> & planner_data) const;
  bool isSafeGoalWithMarginScaleFactor(
    const GoalCandidate & goal_candidate, const double margin_scale_factor,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    const std::shared_ptr<const PlannerData> & planner_data,
    const PredictedObjects & objects) const;

  MultiPolygon2d getAreaPolygons() const { return area_polygons_; }

  bool bus_stop_area_available() const { return !bus_stop_area_polygons_.empty(); }

private:
  GoalSearcher(
    const GoalPlannerParameters & parameters, const LinearRing2d & vehicle_footprint,
    const bool left_side_parking, const lanelet::ConstLanelets & pull_over_lanes,
    const lanelet::BasicPolygons2d & no_parking_area_polygons,
    const lanelet::BasicPolygons2d & no_stopping_area_polygons,
    const lanelet::BasicPolygons2d & bus_stop_area_polygons);

  void countObjectsToAvoid(
    GoalCandidates & goal_candidates, const PredictedObjects & objects,
    const std::shared_ptr<const PlannerData> & planner_data,
    const Pose & reference_goal_pose) const;
  void createAreaPolygons(
    std::vector<Pose> original_search_poses,
    const std::shared_ptr<const PlannerData> & planner_data);
  bool checkCollision(
    const Pose & pose, const PredictedObjects & objects,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map) const;
  bool checkCollisionWithLongitudinalDistance(
    const Pose & ego_pose, const PredictedObjects & objects,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    const std::shared_ptr<const PlannerData> & planner_data) const;

  const GoalPlannerParameters parameters_;
  const LinearRing2d vehicle_footprint_;
  const bool left_side_parking_;
  const lanelet::ConstLanelets pull_over_lanes_;
  const lanelet::BasicPolygons2d no_parking_area_polygons_;
  const lanelet::BasicPolygons2d no_stopping_area_polygons_;
  const lanelet::BasicPolygons2d bus_stop_area_polygons_;

  MultiPolygon2d area_polygons_{};
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_SEARCHER_HPP_
