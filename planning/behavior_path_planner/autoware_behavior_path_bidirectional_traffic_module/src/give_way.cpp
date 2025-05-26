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
#include "autoware/behavior_path_bidirectional_traffic_module/give_way.hpp"

#include "autoware/behavior_path_bidirectional_traffic_module/oncoming_car.hpp"
#include "autoware/trajectory/utils/shift.hpp"

#include <iostream>
#include <vector>

namespace autoware::behavior_path_planner
{

trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId>
NoNeedToGiveWay::modify_trajectory(
  const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose, bool)
{
  if (!oncoming_cars.empty()) {
    give_way_->decide_ego_stop_pose(ego_pose);
    give_way_->transition_to<ShiftingRoadside>(give_way_);
    // return give_way_->modify_trajectory_for_waiting(trajectory, true);
  }
  return trajectory;
}

trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId>
ShiftingRoadside::modify_trajectory(
  const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const std::vector<OncomingCar> &, const geometry_msgs::msg::Pose &, bool is_vehicle_stopped)
{
  if (is_vehicle_stopped) {
    std::cerr << "Waiting for oncoming cars to pass" << std::endl;
    give_way_->transition_to<WaitingForOncomingCarsToPass>(give_way_);
  }
  return give_way_->modify_trajectory_for_waiting(trajectory, true);
}

trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId>
WaitingForOncomingCarsToPass::modify_trajectory(
  const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose &, bool)
{
  if (oncoming_cars.empty()) {
    give_way_->transition_to<BackToNormalLane>(give_way_);
  }
  return give_way_->modify_trajectory_for_waiting(trajectory, true);
}

trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId>
BackToNormalLane::modify_trajectory(
  const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const std::vector<OncomingCar> &, const geometry_msgs::msg::Pose &, bool)
{
  return give_way_->modify_trajectory_for_waiting(trajectory, false);
  ;
}

}  // namespace autoware::behavior_path_planner
