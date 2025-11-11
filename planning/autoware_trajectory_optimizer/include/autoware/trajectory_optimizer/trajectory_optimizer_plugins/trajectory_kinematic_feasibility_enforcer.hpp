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

// NOLINTNEXTLINE
#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_KINEMATIC_FEASIBILITY_ENFORCER_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_KINEMATIC_FEASIBILITY_ENFORCER_HPP_

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_optimizer_plugin_base.hpp"

#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>

namespace autoware::trajectory_optimizer::plugin
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using nav_msgs::msg::Odometry;

/**
 * @brief Parameters for kinematic feasibility enforcer
 */
struct TrajectoryKinematicFeasibilityParams
{
  double max_yaw_rate_rad_s{0.7};  // Maximum yaw rate [rad/s], default from MPC controller
};

/**
 * @brief Plugin that enforces Ackermann steering geometry and yaw rate constraints
 *
 * This plugin implements a forward-propagating algorithm that adjusts trajectory points
 * to ensure kinematic feasibility. Starting from the current ego pose as anchor, it
 * progressively adjusts each point's position and heading so that:
 * 1. Ackermann steering geometry constraint: Δψ_geom = s * tan(δ_max) / L
 * 2. Maximum yaw rate constraint: Δψ_rate = ψ_dot_max * Δt
 *
 * The algorithm maintains segment distances (preserving time structure) while adjusting
 * positions and headings. This ensures compatibility with downstream QP smoother which
 * requires constant time intervals.
 *
 * Based on: "Ackermann + Yaw Rate Feasibility Filtering for Trajectory Points"
 */
class TrajectoryKinematicFeasibilityEnforcer : public TrajectoryOptimizerPluginBase
{
public:
  TrajectoryKinematicFeasibilityEnforcer() = default;
  ~TrajectoryKinematicFeasibilityEnforcer() = default;

  void optimize_trajectory(
    TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
    const TrajectoryOptimizerData & data) override;

  void set_up_params() override;

  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  TrajectoryKinematicFeasibilityParams feasibility_params_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  /**
   * @brief Enforce Ackermann and yaw rate constraints on trajectory
   *
   * @param traj_points Trajectory points to adjust (modified in-place)
   * @param ego_odometry Starting anchor pose (ego vehicle pose)
   * @param dt Average time step between points [s]
   */
  void enforce_ackermann_yaw_rate_constraints(
    TrajectoryPoints & traj_points, const Odometry & ego_odometry, const double dt) const;
};

}  // namespace autoware::trajectory_optimizer::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_KINEMATIC_FEASIBILITY_ENFORCER_HPP_
