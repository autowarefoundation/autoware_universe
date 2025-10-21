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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_extender.hpp"

#include "autoware/trajectory_optimizer/utils.hpp"

#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils_math/unit_conversion.hpp>

#include <vector>

namespace autoware::trajectory_optimizer::plugin
{
void TrajectoryExtender::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
  const TrajectoryOptimizerData & data)
{
  if (params.extend_trajectory_backward) {
    // Note: This function adds the current ego state to a history trajectory. Note that it is ok to
    // call this function several times with the same ego state, since there is a check inside the
    // function to avoid adding the same state multiple times.
    utils::add_ego_state_to_trajectory(
      past_ego_state_trajectory_.points, data.current_odometry,
      extender_params_.nearest_dist_threshold_m,
      autoware_utils_math::deg2rad(extender_params_.nearest_yaw_threshold_deg),
      extender_params_.backward_trajectory_extension_m);
    utils::expand_trajectory_with_ego_history(
      traj_points, past_ego_state_trajectory_.points, data.current_odometry);
  }
}

void TrajectoryExtender::set_up_params()
{
  auto node_ptr = get_node_ptr();
  using autoware_utils_rclcpp::get_or_declare_parameter;

  extender_params_.nearest_dist_threshold_m =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_extender.nearest_dist_threshold_m");
  extender_params_.nearest_yaw_threshold_deg =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_extender.nearest_yaw_threshold_deg");
  extender_params_.backward_trajectory_extension_m = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_extender.backward_trajectory_extension_m");
}

rcl_interfaces::msg::SetParametersResult TrajectoryExtender::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param<double>(
    parameters, "trajectory_extender.nearest_dist_threshold_m",
    extender_params_.nearest_dist_threshold_m);
  update_param<double>(
    parameters, "trajectory_extender.nearest_yaw_threshold_deg",
    extender_params_.nearest_yaw_threshold_deg);
  update_param<double>(
    parameters, "trajectory_extender.backward_trajectory_extension_m",
    extender_params_.backward_trajectory_extension_m);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_optimizer::plugin
