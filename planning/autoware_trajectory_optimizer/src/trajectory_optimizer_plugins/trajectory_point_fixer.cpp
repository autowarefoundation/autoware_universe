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

#include <vector>

namespace autoware::trajectory_optimizer::plugin
{
void TrajectoryPointFixer::optimize_trajectory(
  TrajectoryPoints & traj_points, [[maybe_unused]] const TrajectoryOptimizerParams & params)
{
  if (!params.fix_invalid_points) {
    return;
  }
  utils::remove_invalid_points(traj_points);
}

void TrajectoryPointFixer::set_up_params()
{
}

rcl_interfaces::msg::SetParametersResult TrajectoryPointFixer::on_parameter(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_optimizer::plugin
