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

#ifndef COVARIANCE_UTILS_HPP_
#define COVARIANCE_UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_with_covariance.hpp>

#include <array>

namespace autoware::environment_adaptor
{

inline std::array<double, 36> rotate_covariance(
  const std::array<double, 36> & src, const Eigen::Matrix3d & R)
{
  // Rotate only the 3x3 position block (top-left) of the 6x6 covariance matrix.
  Eigen::Matrix3d src_cov;
  src_cov << src[0], src[1], src[2], src[6], src[7], src[8], src[12], src[13], src[14];

  const Eigen::Matrix3d rotated = R * src_cov * R.transpose();

  std::array<double, 36> ret = src;
  for (Eigen::Index i = 0; i < 3; ++i) {
    ret[i] = rotated(0, i);
    ret[i + 6] = rotated(1, i);
    ret[i + 12] = rotated(2, i);
  }
  return ret;
}

inline void apply_body_covariance_to_pose(
  geometry_msgs::msg::PoseWithCovariance & pose_cov, const std::array<double, 36> & body_cov)
{
  const auto & q_msg = pose_cov.pose.orientation;
  const Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  const auto map_cov = rotate_covariance(body_cov, q.normalized().toRotationMatrix());
  for (size_t i = 0; i < 36; ++i) {
    pose_cov.covariance[i] = map_cov[i];
  }
}

}  // namespace autoware::environment_adaptor

#endif  // COVARIANCE_UTILS_HPP_
