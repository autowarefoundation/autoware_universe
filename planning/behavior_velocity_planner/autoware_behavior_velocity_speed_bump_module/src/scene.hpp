// Copyright 2025 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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
#include <autoware_lanelet2_extension/regulatory_elements/speed_bump.hpp>
#include <autoware_utils/system/time_keeper.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
class SpeedBumpModule : public experimental::SceneModuleInterface
{
public:
  struct DebugData
  {
    double base_link2front;
    std::optional<geometry_msgs::msg::Point> first_intersection_point;
    std::optional<geometry_msgs::msg::Point> second_intersection_point;
    std::optional<geometry_msgs::msg::Pose> slow_start_pose;
    std::optional<geometry_msgs::msg::Point> slow_end_point;
    std::vector<geometry_msgs::msg::Point> speed_bump_polygon;
  };

  struct PlannerParam
  {
    double slow_start_margin;
    double slow_end_margin;
    bool print_debug_info;
    float speed_calculation_min_height;
    float speed_calculation_max_height;
    float speed_calculation_min_speed;
    float speed_calculation_max_speed;
  };

  SpeedBumpModule(
    const lanelet::Id module_id, const lanelet::autoware::SpeedBump & speed_bump_reg_elem,
    const PlannerParam & planner_param, const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);

  bool modifyPathVelocity(
    experimental::Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

private:
  lanelet::Id module_id_;

  // Speed Bump Regulatory Element
  const lanelet::autoware::SpeedBump & speed_bump_reg_elem_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;

  bool applySlowDownSpeed(
    experimental::Trajectory & path,
    const speed_bump::PolygonIntersection & path_polygon_intersection,
    const PlannerData & planner_data);

  double speed_bump_slow_down_speed_;
};

}  // namespace autoware::behavior_velocity_planner

#endif  // SCENE_HPP_
