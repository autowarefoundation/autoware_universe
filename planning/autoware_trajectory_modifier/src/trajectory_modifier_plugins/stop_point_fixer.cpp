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

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/update_param.hpp>

#include <cmath>

namespace autoware::trajectory_modifier::plugin
{

StopPointFixer::StopPointFixer(
  const std::string name, rclcpp::Node * node_ptr,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
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
  if (is_ego_vehicle_moving(data)) {
    return false;
  }
  const double distance_to_last_point = calculate_distance_to_last_point(traj_points, data);
  return distance_to_last_point < params_.min_distance_threshold_m;
}

void StopPointFixer::modify_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryModifierParams & params,
  const TrajectoryModifierData & data)
{
  if (is_trajectory_modification_required(traj_points, params, data)) {
    replace_trajectory_with_stop_point(traj_points, data);
    RCLCPP_DEBUG(
      get_node_ptr()->get_logger(),
      "StopPointFixer: Replaced trajectory with stop point. Distance to last point: %.2f m",
      calculate_distance_to_last_point(traj_points, data));
  }
}

void StopPointFixer::set_up_params()
{
  auto * node = get_node_ptr();
  
  // Declare plugin parameters with descriptors
  rcl_interfaces::msg::ParameterDescriptor velocity_desc;
  velocity_desc.description = "Velocity threshold below which ego vehicle is considered stationary";
  params_.velocity_threshold_mps = node->declare_parameter<double>(
    "stop_point_fixer.velocity_threshold_mps", 0.1, velocity_desc);
    
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

bool StopPointFixer::is_ego_vehicle_moving(const TrajectoryModifierData & data) const
{
  const auto & twist = data.current_odometry.twist.twist;
  const double current_velocity = std::sqrt(
    twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y +
    twist.linear.z * twist.linear.z);

  return current_velocity > params_.velocity_threshold_mps;
}

double StopPointFixer::calculate_distance_to_last_point(
  const TrajectoryPoints & traj_points, const TrajectoryModifierData & data)
{
  if (traj_points.empty()) {
    return 0.0;
  }

  const auto & ego_pose = data.current_odometry.pose.pose;
  const auto & last_point = traj_points.back();

  const double dx = last_point.pose.position.x - ego_pose.position.x;
  const double dy = last_point.pose.position.y - ego_pose.position.y;

  return std::hypot(dx, dy);
}

void StopPointFixer::replace_trajectory_with_stop_point(
  TrajectoryPoints & traj_points, const TrajectoryModifierData & data)
{
  TrajectoryPoint stop_point;

  stop_point.pose = data.current_odometry.pose.pose;

  stop_point.longitudinal_velocity_mps = 0.0;
  stop_point.lateral_velocity_mps = 0.0;
  stop_point.acceleration_mps2 = 0.0;
  stop_point.heading_rate_rps = 0.0;
  stop_point.front_wheel_angle_rad = 0.0;
  stop_point.rear_wheel_angle_rad = 0.0;

  traj_points.clear();
  traj_points.push_back(stop_point);
}

}  // namespace autoware::trajectory_modifier::plugin
