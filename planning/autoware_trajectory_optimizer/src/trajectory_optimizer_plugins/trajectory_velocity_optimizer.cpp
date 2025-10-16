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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_velocity_optimizer.hpp"

#include "autoware/trajectory_optimizer/utils.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

TrajectoryVelocityOptimizer::TrajectoryVelocityOptimizer(
  const std::string name, rclcpp::Node * node_ptr,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
  const TrajectoryOptimizerParams & params)
: TrajectoryOptimizerPluginBase(name, node_ptr, time_keeper, params)
{
  if (params.smooth_velocities) {
    set_up_velocity_smoother(node_ptr, time_keeper);
  }
}

void TrajectoryVelocityOptimizer::set_up_velocity_smoother(
  rclcpp::Node * node_ptr, const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
{
  const auto vehicle_info =
    autoware::vehicle_info_utils::VehicleInfoUtils(*node_ptr).getVehicleInfo();
  double wheelbase = vehicle_info.wheel_base_m;  // vehicle_info.wheel_base_m;
  jerk_filtered_smoother_ = std::make_shared<JerkFilteredSmoother>(*node_ptr, time_keeper);
  jerk_filtered_smoother_->setWheelBase(wheelbase);
}

void TrajectoryVelocityOptimizer::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
  const TrajectoryOptimizerData & data)
{
  const auto & current_odometry = data.current_odometry;
  const auto & current_acceleration = data.current_acceleration;
  const auto & current_speed = current_odometry.twist.twist.linear.x;
  const auto & current_linear_acceleration = current_acceleration.accel.accel.linear.x;
  const double & target_pull_out_speed_mps = params.target_pull_out_speed_mps;
  const double & target_pull_out_acc_mps2 = params.target_pull_out_acc_mps2;
  const double & max_speed_mps = params.max_speed_mps;

  // Limit lateral acceleration
  if (params.limit_lateral_acceleration) {
    utils::limit_lateral_acceleration(
      traj_points, params.max_lateral_accel_mps2, data.current_odometry);
  }

  auto initial_motion_speed =
    (current_speed > target_pull_out_speed_mps) ? current_speed : target_pull_out_speed_mps;
  auto initial_motion_acc = (current_speed > target_pull_out_speed_mps)
                              ? current_linear_acceleration
                              : target_pull_out_acc_mps2;

  // Set engage speed and acceleration
  if (params.set_engage_speed && (current_speed < target_pull_out_speed_mps)) {
    utils::clamp_velocities(
      traj_points, static_cast<float>(initial_motion_speed),
      static_cast<float>(initial_motion_acc));
  }

  // Limit ego speed
  if (params.limit_speed) {
    utils::set_max_velocity(traj_points, static_cast<float>(max_speed_mps));
  }

  // Smooth velocity profile
  if (params.smooth_velocities) {
    if (!jerk_filtered_smoother_) {
      set_up_velocity_smoother(get_node_ptr(), get_time_keeper());
    }
    InitialMotion initial_motion{initial_motion_speed, initial_motion_acc};
    utils::filter_velocity(
      traj_points, initial_motion, params, jerk_filtered_smoother_, current_odometry);
  }
}

void TrajectoryVelocityOptimizer::set_up_params()
{
}

rcl_interfaces::msg::SetParametersResult TrajectoryVelocityOptimizer::on_parameter(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  // cspell:ignore jerkfiltered
  // TODO(Daniel): Add option to update params (not included in the jerkfiltered_smoother)
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_optimizer::plugin
