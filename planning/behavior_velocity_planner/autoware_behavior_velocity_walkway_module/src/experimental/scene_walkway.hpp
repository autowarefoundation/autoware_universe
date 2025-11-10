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

#ifndef EXPERIMENTAL__SCENE_WALKWAY_HPP_
#define EXPERIMENTAL__SCENE_WALKWAY_HPP_

#include "scene_walkway.hpp"

#include <autoware/behavior_velocity_crosswalk_module/util.hpp>
#include <autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp>
#include <autoware_utils/system/time_keeper.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
class WalkwayModule : public SceneModuleInterface
{
public:
  struct PlannerParam
  {
    double stop_distance_from_crosswalk;
    double stop_duration;
  };
  WalkwayModule(
    const lanelet::Id module_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
    const PlannerParam & planner_param, const bool use_regulatory_element,
    const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);

  bool modifyPathVelocity(
    Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

private:
  const lanelet::Id module_id_;

  [[nodiscard]] std::pair<double, geometry_msgs::msg::Point> getStopLine(
    const PathWithLaneId & ego_path, bool & exist_stopline_in_map,
    const geometry_msgs::msg::Point & first_path_point_on_walkway,
    const PlannerData & planner_data) const;

  enum class State { APPROACH, STOP, SURPASSED };

  lanelet::ConstLanelet walkway_;

  lanelet::ConstLineStrings3d stop_lines_;

  // State machine
  State state_;

  // Parameter
  const PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;

  // flag to use regulatory element
  const bool use_regulatory_element_;
};
}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__SCENE_WALKWAY_HPP_
