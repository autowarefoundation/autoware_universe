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

#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__DECISION_STATE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__DECISION_STATE_HPP_

#include "autoware/behavior_path_goal_planner_module/goal_searcher.hpp"
#include "autoware/behavior_path_goal_planner_module/pull_over_planner/pull_over_planner_base.hpp"
#include "autoware/behavior_path_planner_common/utils/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.hpp"

#include <memory>
#include <vector>

namespace autoware::behavior_path_planner
{

class PathDecisionState
{
public:
  enum class DecisionKind {
    NOT_DECIDED,
    DECIDING,
    DECIDED,
  };

  DecisionKind state{DecisionKind::NOT_DECIDED};
  std::optional<rclcpp::Time> deciding_start_time{std::nullopt};
  bool is_stable_safe{false};
  std::optional<rclcpp::Time> safe_start_time{std::nullopt};
};

class PathDecisionStateController
{
public:
  explicit PathDecisionStateController(rclcpp::Logger logger) : logger_(logger) {}

  /**
   * @brief update current state and save old current state to prev state
   */
  void transit_state(
    const std::optional<PullOverPath> & pull_over_path_opt, const bool upstream_module_has_stopline,
    const rclcpp::Time & now,
    const autoware_perception_msgs::msg::PredictedObjects & static_target_objects,
    const autoware_perception_msgs::msg::PredictedObjects & dynamic_target_objects,
    const std::shared_ptr<const PlannerData> planner_data,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    const bool is_current_safe, const bool lane_change_status_changed,
    const GoalPlannerParameters & parameters, const GoalSearcher & goal_searcher,
    std::vector<autoware_utils::Polygon2d> & ego_polygons_expanded);

  PathDecisionState get_current_state() const { return current_state_; }

private:
  rclcpp::Logger logger_;

  PathDecisionState current_state_{};

  /**
   * @brief update current state and save old current state to prev state
   */
  PathDecisionState get_next_state(
    const std::optional<PullOverPath> & pull_over_path_opt, const bool upstream_module_has_stopline,
    const rclcpp::Time & now, const PredictedObjects & static_target_objects,
    const PredictedObjects & dynamic_target_objects,
    const std::shared_ptr<const PlannerData> planner_data,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    const bool is_current_safe, const bool lane_change_status_changed,
    const GoalPlannerParameters & parameters, const GoalSearcher & goal_searcher,
    std::vector<autoware_utils::Polygon2d> & ego_polygons_expanded) const;
};

}  // namespace autoware::behavior_path_planner
#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__DECISION_STATE_HPP_
