// Copyright 2025 TIER IV.
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

#ifndef UTILS__TRANSFORM_UTILS_HPP_
#define UTILS__TRANSFORM_UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nav_msgs/msg/odometry.hpp>

#include <limits>
#include <utility>

namespace autoware::tensorrt_vad::utils
{

/**
 * @brief Extracts transformation matrices from ROS Odometry message
 *
 * Converts odometry pose (position + quaternion) into homogeneous transformation
 * matrices for coordinate frame conversions. Handles quaternion normalization
 * for numerical stability.
 *
 * @param msg ROS Odometry message containing pose information
 * @return Pair of transformation matrices:
 *         - first: base_link → map (forward transform)
 *         - second: map → base_link (inverse transform)
 */
inline std::pair<Eigen::Matrix4d, Eigen::Matrix4d> get_transform_matrix(
  const nav_msgs::msg::Odometry & msg)
{
  // Extract position from odometry
  const double x = msg.pose.pose.position.x;
  const double y = msg.pose.pose.position.y;
  const double z = msg.pose.pose.position.z;

  // Extract and normalize quaternion
  const double qx = msg.pose.pose.orientation.x;
  const double qy = msg.pose.pose.orientation.y;
  const double qz = msg.pose.pose.orientation.z;
  const double qw = msg.pose.pose.orientation.w;

  // Create Eigen quaternion and normalize for numerical stability
  Eigen::Quaterniond q(qw, qx, qy, qz);
  if (q.norm() < std::numeric_limits<double>::epsilon()) {
    // Handle degenerate case: use identity rotation
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }

  // Convert quaternion to rotation matrix
  const Eigen::Matrix3d R = q.toRotationMatrix();

  // Create translation vector
  const Eigen::Vector3d t(x, y, z);

  // Construct forward transformation: base_link → map
  Eigen::Matrix4d bl2map = Eigen::Matrix4d::Identity();
  bl2map.block<3, 3>(0, 0) = R;  // Rotation part
  bl2map.block<3, 1>(0, 3) = t;  // Translation part

  // Construct inverse transformation: map → base_link
  Eigen::Matrix4d map2bl = Eigen::Matrix4d::Identity();
  map2bl.block<3, 3>(0, 0) = R.transpose();       // Inverse rotation
  map2bl.block<3, 1>(0, 3) = -R.transpose() * t;  // Inverse translation

  return {bl2map, map2bl};
}

}  // namespace autoware::tensorrt_vad::utils

#endif  // UTILS__TRANSFORM_UTILS_HPP_
