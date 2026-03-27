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

#include "autoware/trajectory_modifier/utils.hpp"

#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/logging.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
namespace autoware::trajectory_modifier::plugin
{

StopPointFixer::StopPointFixer(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper,
  const TrajectoryModifierParams & params)
: TrajectoryModifierPluginBase(name, node_ptr, time_keeper, params)
{
  set_up_params();
}

bool StopPointFixer::is_long_stop_trajectory(const TrajectoryPoints & traj_points) const
{
  if (traj_points.empty() || !params_.force_stop_long_stopped_trajectories) {
    return false;
  }

  for (const auto & point : traj_points) {
    const auto time_from_start = static_cast<double>(point.time_from_start.sec) +
                                 static_cast<double>(point.time_from_start.nanosec) * 1e-9;

    if (time_from_start > params_.min_stop_duration_s) {
      return true;
    }
    if (point.longitudinal_velocity_mps > params_.velocity_threshold_mps) {
      return false;
    }
  }
  return true;
}

bool StopPointFixer::is_stop_point_close_to_ego(
  const TrajectoryPoints & traj_points, const TrajectoryModifierData & data) const
{
  if (!params_.force_stop_close_stopped_trajectories) {
    return false;
  }
  return utils::calculate_distance_to_last_point(traj_points, data.current_odometry.pose.pose) <
         params_.min_distance_threshold_m;
}

bool StopPointFixer::is_trajectory_modification_required(
  const TrajectoryPoints & traj_points, const TrajectoryModifierParams & params,
  const TrajectoryModifierData & data) const
{
  if (!params.use_stop_point_fixer || traj_points.empty()) {
    return false;
  }

  if (utils::is_ego_vehicle_moving(
        data.current_odometry.twist.twist, params_.velocity_threshold_mps)) {
    return false;
  }

  return is_stop_point_close_to_ego(traj_points, data) || is_long_stop_trajectory(traj_points);
}

void StopPointFixer::modify_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryModifierParams & params,
  const TrajectoryModifierData & data)
{
  if (is_trajectory_modification_required(traj_points, params, data)) {
    utils::replace_trajectory_with_stop_point(traj_points, data.current_odometry.pose.pose);
    auto clock_ptr = get_node_ptr()->get_clock();
    RCLCPP_DEBUG_THROTTLE(
      get_node_ptr()->get_logger(), *clock_ptr, 5000,
      "StopPointFixer: Replaced trajectory with stop point.");
  }
}

void StopPointFixer::set_up_params()
{
  auto * node = get_node_ptr();

  // Declare plugin parameters with descriptors

  rcl_interfaces::msg::ParameterDescriptor force_long_stop_desc;
  force_long_stop_desc.description =
    "Force zero velocity trajectory for trajectories with long stops";
  params_.force_stop_long_stopped_trajectories = node->declare_parameter<bool>(
    "stop_point_fixer.force_stop_long_stopped_trajectories", true, force_long_stop_desc);

  rcl_interfaces::msg::ParameterDescriptor force_close_stop_desc;
  force_close_stop_desc.description =
    "Force zero velocity trajectory for trajectories with stops close to ego";
  params_.force_stop_close_stopped_trajectories = node->declare_parameter<bool>(
    "stop_point_fixer.force_stop_close_stopped_trajectories", true, force_close_stop_desc);

  rcl_interfaces::msg::ParameterDescriptor velocity_desc;
  velocity_desc.description = "Velocity threshold below which ego vehicle is considered stationary";
  params_.velocity_threshold_mps =
    node->declare_parameter<double>("stop_point_fixer.velocity_threshold_mps", 0.25, velocity_desc);

  rcl_interfaces::msg::ParameterDescriptor distance_desc;
  distance_desc.description = "Minimum distance threshold to trigger trajectory replacement";
  params_.min_distance_threshold_m = node->declare_parameter<double>(
    "stop_point_fixer.min_distance_threshold_m", 1.0, distance_desc);

  rcl_interfaces::msg::ParameterDescriptor stop_duration_desc;
  stop_duration_desc.description =
    "Minimum duration of stop to consider for trajectory replacement";
  params_.min_stop_duration_s = node->declare_parameter<double>(
    "stop_point_fixer.min_stop_duration_s", 0.5, stop_duration_desc);
}

rcl_interfaces::msg::SetParametersResult StopPointFixer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    update_param<bool>(
      parameters, "stop_point_fixer.force_stop_long_stopped_trajectories",
      params_.force_stop_long_stopped_trajectories);
    update_param<bool>(
      parameters, "stop_point_fixer.force_stop_close_stopped_trajectories",
      params_.force_stop_close_stopped_trajectories);
    update_param<double>(
      parameters, "stop_point_fixer.velocity_threshold_mps", params_.velocity_threshold_mps);
    update_param<double>(
      parameters, "stop_point_fixer.min_distance_threshold_m", params_.min_distance_threshold_m);
    update_param<double>(
      parameters, "stop_point_fixer.min_stop_duration_s", params_.min_stop_duration_s);
  } catch (const std::exception & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

}  // namespace autoware::trajectory_modifier::plugin
