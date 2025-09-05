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

bool StopPointFixer::is_trajectory_modification_required(
  const TrajectoryPoints & traj_points, const TrajectoryModifierParams & params,
  const TrajectoryModifierData & data) const
{
  if (!params.use_stop_point_fixer) {
    return false;
  }
  if (traj_points.empty()) {
    return false;
  }
  if (utils::is_ego_vehicle_moving(
        data.current_odometry.twist.twist, params_.velocity_threshold_mps)) {
    return false;
  }
  const double distance_to_last_point =
    utils::calculate_distance_to_last_point(traj_points, data.current_odometry.pose.pose);
  return distance_to_last_point < params_.min_distance_threshold_m;
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
      "StopPointFixer: Replaced trajectory with stop point. Distance to last point: %.2f m",
      utils::calculate_distance_to_last_point(traj_points, data.current_odometry.pose.pose));
  }
}

void StopPointFixer::set_up_params()
{
  auto * node = get_node_ptr();

  // Declare plugin parameters with descriptors
  rcl_interfaces::msg::ParameterDescriptor velocity_desc;
  velocity_desc.description = "Velocity threshold below which ego vehicle is considered stationary";
  params_.velocity_threshold_mps =
    node->declare_parameter<double>("stop_point_fixer.velocity_threshold_mps", 0.1, velocity_desc);

  rcl_interfaces::msg::ParameterDescriptor distance_desc;
  distance_desc.description = "Minimum distance threshold to trigger trajectory replacement";
  params_.min_distance_threshold_m = node->declare_parameter<double>(
    "stop_point_fixer.min_distance_threshold_m", 1.0, distance_desc);
}

rcl_interfaces::msg::SetParametersResult StopPointFixer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    update_param<double>(
      parameters, "stop_point_fixer.velocity_threshold_mps", params_.velocity_threshold_mps);
    update_param<double>(
      parameters, "stop_point_fixer.min_distance_threshold_m", params_.min_distance_threshold_m);
  } catch (const std::exception & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

}  // namespace autoware::trajectory_modifier::plugin
