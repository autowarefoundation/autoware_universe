// Copyright 2026 TIER IV, Inc.
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

#ifndef VELOCITY_SMOOTHER_HPP_
#define VELOCITY_SMOOTHER_HPP_

#include "type_alias.hpp"

#include <autoware/velocity_smoother/smoother/jerk_filtered_smoother.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

struct VelocitySmootherParams
{
  double nearest_dist_threshold_m;
  double nearest_yaw_threshold_deg;
  double target_pull_out_speed_mps;
  double target_pull_out_acc_mps2;
  double max_speed_mps;
  double max_lateral_accel_mps2;
  double stop_dist_to_prohibit_engage;
  bool set_engage_speed;
  bool limit_speed;
  bool limit_lateral_acceleration;
  bool smooth_velocities;
};

class VelocitySmoother
{
public:
  VelocitySmoother(
    const VelocitySmootherParams & params, const rclcpp::Logger & logger,
    rclcpp::Clock::SharedPtr clock, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
    std::shared_ptr<autoware::velocity_smoother::JerkFilteredSmoother> jerk_filtered_smoother);

  void optimize(
    TrajectoryPoints & traj_points, const nav_msgs::msg::Odometry & current_odometry,
    double current_acceleration);

private:
  static void clamp_velocities(
    TrajectoryPoints & traj_points, float min_velocity, float min_acceleration);

  static void set_max_velocity(TrajectoryPoints & traj_points, float max_velocity);

  static void limit_lateral_acceleration(
    TrajectoryPoints & traj_points, double max_lateral_accel_mps2,
    const nav_msgs::msg::Odometry & current_odometry);

  void filter_velocity(
    TrajectoryPoints & traj_points, double initial_speed, double initial_acc,
    const nav_msgs::msg::Odometry & current_odometry);

  std::optional<TrajectoryPoint> calc_projected_trajectory_point_from_ego(
    const TrajectoryPoints & trajectory, const geometry_msgs::msg::Pose & ego_pose) const;

  VelocitySmootherParams params_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<autoware::velocity_smoother::JerkFilteredSmoother> jerk_filtered_smoother_;
  TrajectoryPoints prev_output_;
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // VELOCITY_SMOOTHER_HPP_
