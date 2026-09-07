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

#ifndef IN_LANE_MRM_TRAJECTORY_MODIFIER_HPP_
#define IN_LANE_MRM_TRAJECTORY_MODIFIER_HPP_

#include "mrm_obstacle_stop_planner.hpp"
#include "type_alias.hpp"

#include <rclcpp/rclcpp.hpp>

namespace autoware::in_lane_mrm_planner
{

class InLaneMrmTrajectoryModifier
{
public:
  void initialize(rclcpp::Node * node, const VehicleInfo & vehicle_info, const Params & params);
  void set_objects(const PredictedObjects & objects);
  void apply(
    TrajectoryPoints & points, const Odometry & odom, const AccelWithCovarianceStamped & accel);
  void publish_planning_factor();

private:
  MrmObstacleStopPlanner obstacle_stop_planner_;
  PredictedObjects objects_;
};

}  // namespace autoware::in_lane_mrm_planner

#endif  // IN_LANE_MRM_TRAJECTORY_MODIFIER_HPP_
