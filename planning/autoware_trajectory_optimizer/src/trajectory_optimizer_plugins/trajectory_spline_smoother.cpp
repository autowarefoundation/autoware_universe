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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_spline_smoother.hpp"

#include "autoware/trajectory_optimizer/utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>

#include <vector>

namespace autoware::trajectory_optimizer::plugin
{
void TrajectorySplineSmoother::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
  [[maybe_unused]] const TrajectoryOptimizerData & data)
{
  // Apply spline to smooth the trajectory
  if (!params.use_akima_spline_interpolation) {
    return;
  }
  utils::apply_spline(
    traj_points, spline_params_.interpolation_resolution_m, spline_params_.max_yaw_discrepancy_deg,
    spline_params_.max_distance_discrepancy_m,
    spline_params_.preserve_input_trajectory_orientation);
}

void TrajectorySplineSmoother::set_up_params()
{
  auto node_ptr = get_node_ptr();
  using autoware_utils_rclcpp::get_or_declare_parameter;

  spline_params_.interpolation_resolution_m = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_spline_smoother.interpolation_resolution_m");
  spline_params_.max_yaw_discrepancy_deg = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_spline_smoother.max_yaw_discrepancy_deg");
  spline_params_.max_distance_discrepancy_m = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_spline_smoother.max_distance_discrepancy_m");
  spline_params_.preserve_input_trajectory_orientation = get_or_declare_parameter<bool>(
    *node_ptr, "trajectory_spline_smoother.preserve_input_trajectory_orientation");
}

rcl_interfaces::msg::SetParametersResult TrajectorySplineSmoother::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param(
    parameters, "trajectory_spline_smoother.interpolation_resolution_m",
    spline_params_.interpolation_resolution_m);
  update_param(
    parameters, "trajectory_spline_smoother.max_yaw_discrepancy_deg",
    spline_params_.max_yaw_discrepancy_deg);
  update_param(
    parameters, "trajectory_spline_smoother.max_distance_discrepancy_m",
    spline_params_.max_distance_discrepancy_m);
  update_param(
    parameters, "trajectory_spline_smoother.preserve_input_trajectory_orientation",
    spline_params_.preserve_input_trajectory_orientation);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_optimizer::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_optimizer::plugin::TrajectorySplineSmoother,
  autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase)
