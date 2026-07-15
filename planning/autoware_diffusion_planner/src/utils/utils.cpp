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
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::utils
{

namespace
{

inline double square(double x)
{
  return x * x;
}

Eigen::Matrix3d quaternion_to_matrix(const geometry_msgs::msg::Quaternion & q_msg)
{
  const double norm =
    std::sqrt(square(q_msg.w) + square(q_msg.x) + square(q_msg.y) + square(q_msg.z));
  constexpr double kEpsilon = 1e-6;
  if (norm < kEpsilon) {
    throw std::runtime_error("Quaternion norm is too small");
  }

  return Eigen::Quaterniond(q_msg.w, q_msg.x, q_msg.y, q_msg.z).toRotationMatrix();
}
}  // namespace

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

Eigen::Matrix4d pose_to_matrix4d(const geometry_msgs::msg::Pose & pose)
{
  // Extract position
  double x = pose.position.x;
  double y = pose.position.y;
  double z = pose.position.z;

  // Rotation matrix (3x3)
  Eigen::Matrix3d R = quaternion_to_matrix(pose.orientation);

  // Translation vector
  Eigen::Vector3d t(x, y, z);

  // Create 4x4 transformation matrix
  Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
  pose_matrix.block<3, 3>(0, 0) = R;
  pose_matrix.block<3, 1>(0, 3) = t;

  return pose_matrix;
}

std::pair<float, float> rotation_matrix_to_cos_sin(const Eigen::Matrix3d & rotation_matrix)
{
  // Extract yaw angle from rotation matrix and convert to cos/sin
  // Using atan2 to get the yaw angle from the rotation matrix
  const float yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
  return {std::cos(yaw), std::sin(yaw)};
}

geometry_msgs::msg::Pose shift_x(const geometry_msgs::msg::Pose & pose, const double shift_length)
{
  // Rotation matrix (3x3)
  Eigen::Matrix3d R = quaternion_to_matrix(pose.orientation);

  // Shift along the x-axis in the local frame
  Eigen::Vector3d shift_local(shift_length, 0.0, 0.0);

  // Transform shift to the global frame
  Eigen::Vector3d shift_global = R * shift_local;

  // Create new pose
  geometry_msgs::msg::Pose shifted_pose = pose;
  shifted_pose.position.x += shift_global.x();
  shifted_pose.position.y += shift_global.y();
  shifted_pose.position.z += shift_global.z();

  return shifted_pose;
}

PolylineProjection project_pose_onto_polyline(
  const double query_x, const double query_y, const std::vector<Eigen::Matrix4d> & polyline)
{
  if (polyline.size() < 2) {
    throw std::runtime_error("project_pose_onto_polyline requires at least two poses");
  }

  const Eigen::Vector2d query(query_x, query_y);

  double best_dist_sq = std::numeric_limits<double>::max();
  Eigen::Vector2d best_foot = query;
  Eigen::Quaterniond best_orientation = Eigen::Quaterniond::Identity();
  // (closest segment index + intra-segment ratio); position of the foot along the polyline.
  double best_interpolation_index = 0.0;

  // Because the diffusion planner runs at 10Hz, we only need to consider the first few segments of
  // the polyline for snapping.
  const size_t max_index = 5;

  for (size_t i = 0; i + 1 <= max_index && i + 1 < polyline.size(); ++i) {
    // Endpoints of the i-th line segment in the xy-plane.
    const Eigen::Vector2d segment_start(polyline[i](0, 3), polyline[i](1, 3));
    const Eigen::Vector2d segment_end(polyline[i + 1](0, 3), polyline[i + 1](1, 3));

    // Direction vector of the segment (start -> end) and the vector from the start to the query.
    const Eigen::Vector2d segment_vector = segment_end - segment_start;
    const Eigen::Vector2d start_to_query = query - segment_start;

    // Squared length of the segment. Also used to guard against a degenerate (zero-length) segment.
    const double segment_length_sq = segment_vector.squaredNorm();

    // Projection ratio of the query point onto the infinite line through the segment:
    //   raw_ratio = dot(start_to_query, segment_vector) / |segment_vector|^2
    // raw_ratio == 0 -> foot at segment_start, raw_ratio == 1 -> foot at segment_end.
    // Clamping to [0, 1] keeps the foot ON the segment: if the perpendicular foot would land beyond
    // an endpoint (raw_ratio < 0 or > 1), it is pulled back to the nearest endpoint.
    constexpr double eps = 1e-9;
    double ratio = 0.0;
    if (segment_length_sq >= eps) {
      const double projection = start_to_query.dot(segment_vector);
      const double raw_ratio = projection / segment_length_sq;
      ratio = std::clamp(raw_ratio, 0.0, 1.0);
    }

    // Foot of the perpendicular (already clamped onto the segment) and its distance to the query.
    const Eigen::Vector2d foot = segment_start + ratio * segment_vector;
    const double dist_sq = (query - foot).squaredNorm();

    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best_foot = foot;
      const Eigen::Quaterniond start_orientation(polyline[i].block<3, 3>(0, 0));
      const Eigen::Quaterniond end_orientation(polyline[i + 1].block<3, 3>(0, 0));
      best_orientation = start_orientation.slerp(ratio, end_orientation);
      best_interpolation_index = static_cast<double>(i) + ratio;
    }
  }

  Eigen::Matrix4d projected = Eigen::Matrix4d::Identity();
  projected.block<3, 3>(0, 0) = best_orientation.normalized().toRotationMatrix();
  projected(0, 3) = best_foot.x();
  projected(1, 3) = best_foot.y();
  return PolylineProjection{projected, best_interpolation_index};
}

Eigen::Matrix4d inverse(const Eigen::Matrix4d & mat)
{
  return Eigen::Isometry3d(mat).inverse().matrix();
}

std::vector<float> replicate_for_batch(const std::vector<float> & single_data, const int batch_size)
{
  const size_t single_size = single_data.size();
  const size_t total_size = static_cast<size_t>(batch_size) * single_size;

  std::vector<float> batch_data;
  batch_data.reserve(total_size);

  for (int i = 0; i < batch_size; ++i) {
    batch_data.insert(batch_data.end(), single_data.begin(), single_data.end());
  }

  return batch_data;
}

}  // namespace autoware::diffusion_planner::utils
