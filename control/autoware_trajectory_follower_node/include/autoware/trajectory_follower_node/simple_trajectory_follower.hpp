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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__SIMPLE_TRAJECTORY_FOLLOWER_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__SIMPLE_TRAJECTORY_FOLLOWER_HPP_

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace simple_trajectory_follower
{
using autoware_control_msgs::msg::Control;
using autoware_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

// Settings read once from ROS parameters by the node and passed to the constructor.
struct SimpleTrajectoryFollowerConfig
{
  bool use_external_target_vel{false};
  double external_target_vel{0.0};
  double lateral_deviation{0.0};
};

// Output of one control step.
struct SimpleTrajectoryFollowerResult
{
  Control command;  // stamp fields left unset; the node stamps it

  // Control-law intermediates not already in `command`, returned so the node can log them.
  struct Debug
  {
    double steer_kp{0.0};
    double steer_kd{0.0};
    double lateral_error{0.0};
    double yaw_error{0.0};
    double velocity_error{0.0};
  } debug;
};

// ROS-independent control logic
class SimpleTrajectoryFollower
{
public:
  explicit SimpleTrajectoryFollower(const SimpleTrajectoryFollowerConfig & config);

  // Precondition: trajectory.points must not be empty. As in the original node, an empty trajectory
  // makes motion_utils::findNearestIndex throw (std::invalid_argument); this behavior is preserved.
  SimpleTrajectoryFollowerResult compute_control_command(
    const Trajectory & trajectory, const Odometry & odometry) const;

private:
  SimpleTrajectoryFollowerConfig config_;
};

}  // namespace simple_trajectory_follower

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__SIMPLE_TRAJECTORY_FOLLOWER_HPP_
