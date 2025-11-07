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

#ifndef EXPERIMENTAL__SCENE_OCCLUSION_SPOT_HPP_
#define EXPERIMENTAL__SCENE_OCCLUSION_SPOT_HPP_

#include "../occlusion_spot_utils.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils/system/time_keeper.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
class OcclusionSpotModule : public SceneModuleInterface
{
  using PlannerParam = occlusion_spot_utils::PlannerParam;
  using DebugData = occlusion_spot_utils::DebugData;

public:
  OcclusionSpotModule(
    const lanelet::Id module_id, const PlannerData & planner_data,
    const PlannerParam & planner_param, const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);

  /**
   * @brief plan occlusion spot velocity at unknown area in occupancy grid
   */
  bool modifyPathVelocity(
    Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

private:
  // Parameter
  PlannerParam param_;
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_;
  std::vector<lanelet::BasicPolygon2d> partition_lanelets_;

protected:
  lanelet::Id module_id_{};

  // Debug
  mutable DebugData debug_data_;
};
}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__SCENE_OCCLUSION_SPOT_HPP_
