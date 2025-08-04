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

#include <vector>

namespace autoware::trajectory_optimizer::plugin
{
void TrajectoryExtender::optimize_trajectory(
  TrajectoryPoints & traj_points, [[maybe_unused]] const TrajectoryOptimizerParams & params)
{
  if (params.extend_trajectory_backward) {
    // Note: This function adds the current ego state to a history trajectory. Note that it is ok to
    // call this function several times with the same ego state, since there is a check inside the
    // function to avoid adding the same state multiple times.
    utils::add_ego_state_to_trajectory(
      past_ego_state_trajectory_.points, params.current_odometry, params);
    utils::expand_trajectory_with_ego_history(
      traj_points, past_ego_state_trajectory_.points, params.current_odometry, params);
  }
}

void TrajectoryExtender::set_up_params()
{
}

rcl_interfaces::msg::SetParametersResult TrajectoryExtender::on_parameter(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_optimizer::plugin
