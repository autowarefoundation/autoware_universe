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

#include "autoware/diffusion_planner/utils/utils.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace autoware::diffusion_planner::utils
{
std::pair<Eigen::Matrix4f, Eigen::Matrix4f> get_transform_matrix(
  const nav_msgs::msg::Odometry & msg)
{
  // Extract position
  double x = msg.pose.pose.position.x;
  double y = msg.pose.pose.position.y;
  double z = msg.pose.pose.position.z;

  // Create Eigen quaternion and normalize it just in case
  Eigen::Quaternionf q = std::invoke([&msg]() -> Eigen::Quaternionf {
    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;
    double qw = msg.pose.pose.orientation.w;

    // Create Eigen quaternion and normalize it just in case
    Eigen::Quaternionf q(qw, qx, qy, qz);
    return (q.norm() < std::numeric_limits<float>::epsilon()) ? Eigen::Quaternionf::Identity()
                                                              : q.normalized();
  });

  // Rotation matrix (3x3)
  Eigen::Matrix3f R = q.toRotationMatrix();

  // Translation vector
  Eigen::Vector3f t(x, y, z);

  // Base_link → Map (forward)
  Eigen::Matrix4f bl2map = Eigen::Matrix4f::Identity();
  bl2map.block<3, 3>(0, 0) = R;
  bl2map.block<3, 1>(0, 3) = t;

  // Map → Base_link (inverse)
  Eigen::Matrix4f map2bl = Eigen::Matrix4f::Identity();
  map2bl.block<3, 3>(0, 0) = R.transpose();
  map2bl.block<3, 1>(0, 3) = -R.transpose() * t;

  return {bl2map, map2bl};
}

std::vector<float> create_float_data(const std::vector<int64_t> & shape, float fill)
{
  size_t total_size = 1;
  for (auto dim : shape) {
    // Check for overflow before multiplication
    if (dim > 0 && total_size > std::numeric_limits<size_t>::max() / static_cast<size_t>(dim)) {
      throw std::overflow_error("Shape dimensions would cause size_t overflow");
    }
    total_size *= static_cast<size_t>(dim);
  }
  std::vector<float> data(total_size, fill);
  return data;
}

bool check_input_map(const std::unordered_map<std::string, std::vector<float>> & input_map)
{
  for (const auto & tup : input_map) {
    if (std::any_of(tup.second.begin(), tup.second.end(), [](const auto & v) {
          return !std::isfinite(v) || std::isnan(v);
        })) {
      std::cerr << "key " << tup.first << " contains invalid values\n";
      return false;
    }
  }
  return true;
}

}  // namespace autoware::diffusion_planner::utils
