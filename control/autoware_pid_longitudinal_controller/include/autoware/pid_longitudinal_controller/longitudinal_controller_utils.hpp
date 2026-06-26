// Copyright 2018-2021 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE__PID_LONGITUDINAL_CONTROLLER__LONGITUDINAL_CONTROLLER_UTILS_HPP_
#define AUTOWARE__PID_LONGITUDINAL_CONTROLLER__LONGITUDINAL_CONTROLLER_UTILS_HPP_

#include "autoware/trajectory/trajectory_point.hpp"

#include "geometry_msgs/msg/pose.hpp"

namespace autoware::motion::control::pid_longitudinal_controller
{
namespace longitudinal_utils
{

using TrajectoryExperimental =
  autoware::experimental::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Quaternion;

/**
 * @brief calculate distance to stopline from current vehicle position where velocity is 0
 */
double calcStopDistance(
  const Pose & current_pose, const TrajectoryExperimental & traj, const double max_dist,
  const double max_yaw);

/**
 * @brief calculate pitch angle from estimated current pose
 */
double getPitchByPose(const Quaternion & quaternion);

/**
 * @brief calculate pitch angle from trajectory on map
 * NOTE: there is currently no z information so this always returns 0.0
 * @param [in] trajectory input trajectory
 * @param [in] start_base nearest arc-length position to current vehicle position
 * @param [in] wheel_base length of wheel base
 */
double getPitchByTraj(
  const TrajectoryExperimental & trajectory, const double start_base, const double wheel_base);

/**
 * @brief calculate vehicle pose after time delay by moving the vehicle at current velocity and
 * acceleration for delayed time
 */
Pose calcPoseAfterTimeDelay(
  const Pose & current_pose, const double delay_time, const double current_vel,
  const double current_acc);

/**
 * @brief limit variable whose differential is within a certain value
 * @param [in] input_val current value
 * @param [in] prev_val previous value
 * @param [in] dt time between current and previous one
 * @param [in] lim_val limitation value for differential
 */
double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double lim_val);

/**
 * @brief limit variable whose differential is within a certain value
 * @param [in] input_val current value
 * @param [in] prev_val previous value
 * @param [in] dt time between current and previous one
 * @param [in] max_val maximum value for differential
 * @param [in] min_val minimum value for differential
 */
double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double max_val,
  const double min_val);

}  // namespace longitudinal_utils
}  // namespace autoware::motion::control::pid_longitudinal_controller

#endif  // AUTOWARE__PID_LONGITUDINAL_CONTROLLER__LONGITUDINAL_CONTROLLER_UTILS_HPP_
