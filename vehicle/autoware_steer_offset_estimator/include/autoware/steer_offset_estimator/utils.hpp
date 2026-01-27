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

#ifndef AUTOWARE__STEER_OFFSET_ESTIMATOR__UTILS_HPP_
#define AUTOWARE__STEER_OFFSET_ESTIMATOR__UTILS_HPP_

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>

/**
 * @brief Steer offset estimator utilities namespace
 */
namespace autoware::steer_offset_estimator::utils
{
using autoware_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Vector3;

/**
 * @brief Compute relative rotation vector between two quaternions
 *
 * This function computes the rotation vector representing the angular difference
 * between two quaternions. The result is the axis-angle representation of the
 * rotation from q1 to q2.
 *
 * @param q1 First quaternion (starting orientation)
 * @param q2 Second quaternion (ending orientation)
 * @return Vector3 representing the relative rotation in axis-angle form
 */
[[nodiscard]] Vector3 compute_relative_rotation_vector(
  const tf2::Quaternion & q1, const tf2::Quaternion & q2);

/**
 * @brief Calculate twist from pose difference
 *
 * This function computes the twist (linear and angular velocities) between
 * two pose measurements. The velocities are calculated by taking the difference
 * between the poses and dividing by the time difference.
 *
 * @param pose_a First pose (earlier timestamp)
 * @param pose_b Second pose (later timestamp)
 * @return Twist containing calculated linear and angular velocities
 */
[[nodiscard]] Twist calc_twist_from_pose(const PoseStamped & pose_a, const PoseStamped & pose_b);

}  // namespace autoware::steer_offset_estimator::utils

#endif  // AUTOWARE__STEER_OFFSET_ESTIMATOR__UTILS_HPP_
