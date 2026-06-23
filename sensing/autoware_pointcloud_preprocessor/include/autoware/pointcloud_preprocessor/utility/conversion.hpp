// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__UTILITY__CONVERSION_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__UTILITY__CONVERSION_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2/LinearMath/Transform.hpp>

#include <geometry_msgs/msg/transform.hpp>

#include <cstdint>

// ROS-message conversion helpers that depend only on message types and header-only math (no rclcpp,
// no tf2_ros). They let downstream code keep header stamps and transforms without pulling in the
// ROS runtime.
namespace autoware::pointcloud_preprocessor::utils
{

/// Convert a header stamp to an absolute time in integer nanoseconds (matches rclcpp::Time).
inline int64_t to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

/// Convert a header stamp to an absolute time in seconds (matches rclcpp::Time::seconds()).
inline double to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(to_nanoseconds(stamp)) * 1e-9;
}

/// Convert a transform message to a tf2 transform (equivalent to tf2::fromMsg).
inline tf2::Transform to_tf2_transform(const geometry_msgs::msg::Transform & transform)
{
  tf2::Transform out;
  out.setOrigin(
    tf2::Vector3(transform.translation.x, transform.translation.y, transform.translation.z));
  out.setRotation(
    tf2::Quaternion(
      transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w));
  return out;
}

/// Convert a transform message to a 4x4 homogeneous matrix (equivalent to tf2::transformToEigen).
inline Eigen::Matrix4f to_eigen_matrix(const geometry_msgs::msg::Transform & transform)
{
  const Eigen::Isometry3d isometry =
    Eigen::Translation3d(
      transform.translation.x, transform.translation.y, transform.translation.z) *
    Eigen::Quaterniond(
      transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
  return isometry.matrix().cast<float>();
}

}  // namespace autoware::pointcloud_preprocessor::utils

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__UTILITY__CONVERSION_HPP_
