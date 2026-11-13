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

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform.hpp>

// Package-internal message-to-Eigen conversion helpers that do not depend on the ROS runtime.
namespace autoware::pointcloud_preprocessor
{

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

}  // namespace autoware::pointcloud_preprocessor
