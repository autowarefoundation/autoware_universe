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

#ifndef AUTOWARE__CONTROL_EVALUATOR__METRICS__DEVIATION_METRICS_HPP_
#define AUTOWARE__CONTROL_EVALUATOR__METRICS__DEVIATION_METRICS_HPP_

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

namespace control_diagnostics
{
namespace metrics
{
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

/**
 * @brief calculate lateral deviation of the given trajectory from the reference trajectory
 * @param [in] ref reference trajectory
 * @param [in] point input point
 * @return lateral deviation
 */
double calcLateralDeviation(const Trajectory & traj, const Point & point);

/**
 * @brief calculate yaw deviation of the given trajectory from the reference trajectory
 * @param [in] traj input trajectory
 * @param [in] pose input pose
 * @return yaw deviation
 */
double calcYawDeviation(const Trajectory & traj, const Pose & pose);

/**
 * @brief calculate longitudinal deviation from target_point to base_pose
 * @param [in] pose input base_pose
 * @param [in] point input target_point
 * @return longitudinal deviation from base_pose to target_point
 */
double calcLongitudinalDeviation(const Pose & base_pose, const Point & target_point);

/**
 * @brief calculate lateral deviation from target_point to base_pose
 * @param [in] pose input base_pose
 * @param [in] point input target_point
 * @return lateral deviation from base_pose to target_point
 */
double calcLateralDeviation(const Pose & base_pose, const Point & target_point);

/**
 * @brief calculate yaw deviation from base_pose to target_pose
 * @param [in] pose input base_pose
 * @param [in] pose input target_pose
 * @return yaw deviation from base_pose to target_pose
 */
double calcYawDeviation(const Pose & base_pose, const Pose & target_pose);

}  // namespace metrics
}  // namespace control_diagnostics

#endif  // AUTOWARE__CONTROL_EVALUATOR__METRICS__DEVIATION_METRICS_HPP_
