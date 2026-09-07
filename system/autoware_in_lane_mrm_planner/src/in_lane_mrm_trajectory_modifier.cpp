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

#include "in_lane_mrm_trajectory_modifier.hpp"

namespace autoware::in_lane_mrm_planner
{

void InLaneMrmTrajectoryModifier::initialize(
  rclcpp::Node * node, const VehicleInfo & vehicle_info, const Params & params)
{
  obstacle_stop_planner_.initialize(node, vehicle_info, params);
}

void InLaneMrmTrajectoryModifier::set_objects(const PredictedObjects & objects)
{
  objects_ = objects;
}

void InLaneMrmTrajectoryModifier::apply(
  TrajectoryPoints & points, const Odometry & odom, const AccelWithCovarianceStamped & accel)
{
  obstacle_stop_planner_.set_input(odom, accel, objects_);
  obstacle_stop_planner_.apply(points);
}

void InLaneMrmTrajectoryModifier::publish_planning_factor()
{
  obstacle_stop_planner_.publish_planning_factor();
}

}  // namespace autoware::in_lane_mrm_planner
