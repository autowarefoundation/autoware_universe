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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/stop_point_fixer.hpp"

#include "autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp"

#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/logging.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
namespace autoware::trajectory_modifier::plugin
{

void StopPointFixer::on_initialize(const TrajectoryModifierParams & params)
{
  const auto node_ptr = get_node_ptr();
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      node_ptr, "stop_point_fixer");

  params_ = params.stop_point_fixer;
  enabled_ = params.use_stop_point_fixer;
}

bool StopPointFixer::is_long_stop_trajectory(const TrajectoryPoints & traj_points) const
{
  if (traj_points.empty() || !params_.force_stop_long_stopped_trajectories) {
    return false;
  }

  for (const auto & point : traj_points) {
    const auto time_from_start = static_cast<double>(point.time_from_start.sec) +
                                 static_cast<double>(point.time_from_start.nanosec) * 1e-9;

    if (time_from_start > params_.min_stop_duration) {
      return true;
    }
    if (point.longitudinal_velocity_mps > params_.velocity_threshold) {
      return false;
    }
  }
  return true;
}

bool StopPointFixer::is_stop_point_close_to_ego(const TrajectoryPoints & traj_points) const
{
  if (!params_.force_stop_close_stopped_trajectories) {
    return false;
  }
  return utils::calculate_distance_to_last_point(traj_points, data_->current_odometry->pose.pose) <
         params_.min_distance_threshold;
}

bool StopPointFixer::is_trajectory_modification_required(const TrajectoryPoints & traj_points)
{
  if (!enabled_ || traj_points.empty()) {
    return false;
  }

  if (utils::is_ego_vehicle_moving(
        data_->current_odometry->twist.twist, params_.velocity_threshold)) {
    return false;
  }

  return is_stop_point_close_to_ego(traj_points) || is_long_stop_trajectory(traj_points);
}

bool StopPointFixer::modify_trajectory(TrajectoryPoints & traj_points)
{
  if (!is_trajectory_modification_required(traj_points)) return false;

  utils::replace_trajectory_with_stop_point(traj_points, data_->current_odometry->pose.pose);
  auto clock_ptr = get_node_ptr()->get_clock();
  RCLCPP_DEBUG_THROTTLE(
    get_node_ptr()->get_logger(), *clock_ptr, 5000,
    "StopPointFixer: Replaced trajectory with stop point.");

  // Add PlanningFactor for the stop decision
  const auto & ego_pose = data_->current_odometry->pose.pose;
  planning_factor_interface_->add(
    traj_points, ego_pose, ego_pose, PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{});
  return true;
}

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::StopPointFixer,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
