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

#ifndef TYPE_ALIAS_HPP_
#define TYPE_ALIAS_HPP_

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_optimizer_plugin_base.hpp"

#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware/trajectory_modifier/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_minimum_rule_based_planner/minimum_rule_based_planner_parameters.hpp>
#include <pluginlib/class_loader.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <vector>

namespace autoware::minimum_rule_based_planner
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using unique_identifier_msgs::msg::UUID;
using Params = ::minimum_rule_based_planner::Params;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using PathPointTrajectory = autoware::experimental::trajectory::Trajectory<PathPointWithLaneId>;
using TrajectoryPointTrajectory = autoware::experimental::trajectory::Trajectory<TrajectoryPoint>;

using TrajectoryClass = PathPointTrajectory;

using OptimizerPluginInterface =
  autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase;
using OptimizerPluginLoader = pluginlib::ClassLoader<OptimizerPluginInterface>;

using ModifierPluginLoader =
  pluginlib::ClassLoader<trajectory_modifier::plugin::TrajectoryModifierPluginBase>;

}  // namespace autoware::minimum_rule_based_planner

#endif  // TYPE_ALIAS_HPP_
