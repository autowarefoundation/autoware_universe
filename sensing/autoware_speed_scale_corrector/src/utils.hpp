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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

namespace autoware::speed_scale_corrector
{
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Vector3;

/**
 * @brief Calculate time difference between two poses
 *
 * This function computes the time difference in seconds between two pose
 * measurements based on their timestamps.
 *
 * @param pose_a First pose (earlier timestamp)
 * @param pose_b Second pose (later timestamp)
 * @return Time difference in seconds
 */
[[nodiscard]] double calc_time_diff(const PoseStamped & pose_a, const PoseStamped & pose_b);

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

}  // namespace autoware::speed_scale_corrector

#endif  // UTILS_HPP_
