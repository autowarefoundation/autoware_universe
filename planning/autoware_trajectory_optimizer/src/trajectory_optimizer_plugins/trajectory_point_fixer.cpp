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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_point_fixer.hpp"

#include "autoware/trajectory_optimizer/utils.hpp"

#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>

#include <vector>

namespace autoware::trajectory_optimizer::plugin
{
void TrajectoryPointFixer::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
  [[maybe_unused]] const TrajectoryOptimizerData & data)
{
  if (!params.fix_invalid_points) {
    return;
  }
  utils::remove_invalid_points(traj_points);
}

void TrajectoryPointFixer::set_up_params()
{
  auto node_ptr = get_node_ptr();
  using autoware_utils_rclcpp::get_or_declare_parameter;

  fixer_params_.orientation_threshold_deg =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_point_fixer.orientation_threshold_deg");
}

rcl_interfaces::msg::SetParametersResult TrajectoryPointFixer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param<double>(
    parameters, "trajectory_point_fixer.orientation_threshold_deg",
    fixer_params_.orientation_threshold_deg);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_optimizer::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_optimizer::plugin::TrajectoryPointFixer,
  autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase)
