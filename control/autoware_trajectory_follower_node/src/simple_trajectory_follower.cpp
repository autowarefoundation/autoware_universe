//  Copyright 2022 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "autoware/trajectory_follower_node/simple_trajectory_follower.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <algorithm>
#include <cmath>

namespace simple_trajectory_follower
{

using autoware::motion_utils::findNearestIndex;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils::calc_lateral_deviation;
using autoware_utils::calc_yaw_deviation;

namespace
{
// Steering result: the command plus the intermediates the node logs.
struct SteerComputation
{
  double kp;
  double kd;
  double lateral_error;
  double yaw_error;
  double steer;
};

// Acceleration result: the command plus the velocity error the node logs.
struct AccComputation
{
  double velocity_error;
  double acceleration;
};

// Linearized pure-pursuit steering command for the closest trajectory point.
SteerComputation calc_steer_cmd(
  const TrajectoryPoint & closest_traj_point, const Odometry & odometry,
  const double lateral_deviation)
{
  const auto lat_err =
    calc_lateral_deviation(closest_traj_point.pose, odometry.pose.pose.position) -
    lateral_deviation;
  const auto yaw_err = calc_yaw_deviation(closest_traj_point.pose, odometry.pose.pose);

  // linearized pure_pursuit control
  constexpr auto wheel_base = 4.0;
  constexpr auto lookahead_time = 3.0;
  constexpr auto min_lookahead = 3.0;
  const auto lookahead = min_lookahead + lookahead_time * std::abs(odometry.twist.twist.linear.x);
  const auto kp = 2.0 * wheel_base / (lookahead * lookahead);
  const auto kd = 2.0 * wheel_base / lookahead;

  constexpr auto steer_lim = 0.6;

  const auto steer = std::clamp(-kp * lat_err - kd * yaw_err, -steer_lim, steer_lim);
  return SteerComputation{kp, kd, lat_err, yaw_err, steer};
}

// Proportional acceleration command tracking the target velocity.
AccComputation calc_acc_cmd(
  const TrajectoryPoint & closest_traj_point, const Odometry & odometry,
  const bool use_external_target_vel, const double external_target_vel)
{
  const auto traj_vel = static_cast<double>(closest_traj_point.longitudinal_velocity_mps);
  const auto ego_vel = odometry.twist.twist.linear.x;
  const auto target_vel = use_external_target_vel ? external_target_vel : traj_vel;
  const auto vel_err = ego_vel - target_vel;

  // P feedback
  constexpr auto kp = 0.5;
  constexpr auto acc_lim = 2.0;

  const auto acc = std::clamp(-kp * vel_err, -acc_lim, acc_lim);
  return AccComputation{vel_err, acc};
}
}  // namespace

SimpleTrajectoryFollower::SimpleTrajectoryFollower(const SimpleTrajectoryFollowerConfig & config)
: config_(config)
{
}

SimpleTrajectoryFollowerResult SimpleTrajectoryFollower::compute_control_command(
  const Trajectory & trajectory, const Odometry & odometry, const rclcpp::Time & stamp) const
{
  const auto closest = findNearestIndex(trajectory.points, odometry.pose.pose.position);
  const auto & closest_traj_point = trajectory.points.at(closest);

  const auto steer = calc_steer_cmd(closest_traj_point, odometry, config_.lateral_deviation);
  const auto acc = calc_acc_cmd(
    closest_traj_point, odometry, config_.use_external_target_vel, config_.external_target_vel);

  SimpleTrajectoryFollowerResult result;
  result.command.stamp = result.command.lateral.stamp = result.command.longitudinal.stamp = stamp;
  result.command.lateral.steering_tire_angle = static_cast<float>(steer.steer);
  result.command.longitudinal.velocity = config_.use_external_target_vel
                                           ? static_cast<float>(config_.external_target_vel)
                                           : closest_traj_point.longitudinal_velocity_mps;
  result.command.longitudinal.acceleration = static_cast<float>(acc.acceleration);

  result.debug.steer_kp = steer.kp;
  result.debug.steer_kd = steer.kd;
  result.debug.lateral_error = steer.lateral_error;
  result.debug.yaw_error = steer.yaw_error;
  result.debug.velocity_error = acc.velocity_error;
  return result;
}

}  // namespace simple_trajectory_follower
