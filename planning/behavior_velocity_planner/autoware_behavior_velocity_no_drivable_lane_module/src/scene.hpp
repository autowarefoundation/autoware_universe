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

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include "util.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp>
#include <autoware_utils/system/time_keeper.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using Trajectory = autoware::experimental::trajectory::Trajectory<PathPointWithLaneId>;

class NoDrivableLaneModule : public experimental::SceneModuleInterface
{
public:
  enum class State { INIT, APPROACHING, INSIDE_NO_DRIVABLE_LANE, STOPPED };

  struct SegmentIndexWithPose
  {
    size_t index;
    geometry_msgs::msg::Pose pose;
  };

  struct DebugData
  {
    double base_link2front;
    std::optional<geometry_msgs::msg::Point> first_intersection_point;
    std::optional<geometry_msgs::msg::Point> second_intersection_point;
    std::vector<geometry_msgs::msg::Point> no_drivable_lane_polygon;
    geometry_msgs::msg::Pose stop_pose;
  };

  struct PlannerParam
  {
    double stop_margin;
    bool print_debug_info;
  };

  NoDrivableLaneModule(
    const lanelet::Id module_id, const lanelet::Id lane_id, const PlannerParam & planner_param,
    const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock,
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
  const lanelet::Id lane_id_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;

  // State machine
  State state_;

  void handle_init_state(
    const double ego_front_s,
    const no_drivable_lane::PolygonIntersection & path_polygon_intersection);
  void handle_approaching_state(
    Trajectory & path, const double ego_front_s,
    const no_drivable_lane::PolygonIntersection & path_polygon_intersection,
    const PlannerData & planner_data);
  void handle_inside_no_drivable_lane_state(
    Trajectory & path, const double ego_front_s, const PlannerData & planner_data);
  void handle_stopped_state(
    Trajectory & path, const double ego_front_s, const PlannerData & planner_data);

  void initialize_debug_data(
    const Trajectory & path, const lanelet::Lanelet & no_drivable_lane,
    const geometry_msgs::msg::Point & ego_pos,
    const no_drivable_lane::PolygonIntersection & path_polygon_intersection,
    const PlannerData & planner_data);
};
}  // namespace autoware::behavior_velocity_planner

#endif  // SCENE_HPP_
