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

#include "autoware/steer_offset_estimator/utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <limits>
#include <vector>

namespace autoware::steer_offset_estimator::utils
{

geometry_msgs::msg::Vector3 compute_relative_rotation_vector(
  const tf2::Quaternion & q1, const tf2::Quaternion & q2)
{
  const tf2::Quaternion diff_quaternion = q1.inverse() * q2;
  const tf2::Vector3 axis = diff_quaternion.getAxis() * diff_quaternion.getAngle();
  return geometry_msgs::msg::Vector3{}.set__x(axis.x()).set__y(axis.y()).set__z(axis.z());
}

tf2::Quaternion to_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  return tf2::Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
}

Twist calc_twist_from_pose(const PoseStamped & pose_a, const PoseStamped & pose_b)
{
  const double dt =
    (rclcpp::Time(pose_b.header.stamp) - rclcpp::Time(pose_a.header.stamp)).seconds();

  Twist twist;

  // Return zero twist if time difference is too small
  if (std::abs(dt) < std::numeric_limits<double>::epsilon()) {
    return twist;
  }

  const auto pose_a_quaternion = to_quaternion(pose_a.pose.orientation);
  const auto pose_b_quaternion = to_quaternion(pose_b.pose.orientation);

  // Calculate position difference
  Vector3 diff_xyz;
  diff_xyz.x = pose_b.pose.position.x - pose_a.pose.position.x;
  diff_xyz.y = pose_b.pose.position.y - pose_a.pose.position.y;
  diff_xyz.z = pose_b.pose.position.z - pose_a.pose.position.z;

  // Calculate orientation difference
  const Vector3 relative_rotation_vector =
    compute_relative_rotation_vector(pose_a_quaternion, pose_b_quaternion);

  // Calculate linear velocity (magnitude of position change)
  twist.linear.x =
    std::sqrt(std::pow(diff_xyz.x, 2.0) + std::pow(diff_xyz.y, 2.0) + std::pow(diff_xyz.z, 2.0)) /
    dt;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;

  // Calculate angular velocity
  twist.angular.x = relative_rotation_vector.x / dt;
  twist.angular.y = relative_rotation_vector.y / dt;
  twist.angular.z = relative_rotation_vector.z / dt;

  return twist;
}

}  // namespace autoware::steer_offset_estimator::utils
