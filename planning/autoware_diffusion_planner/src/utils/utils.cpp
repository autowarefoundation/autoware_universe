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

#include "autoware/trajectory/interpolator/cubic_spline.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/threshold.hpp"
#include "autoware/trajectory/utils/closest.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
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

namespace
{
geometry_msgs::msg::Pose matrix4d_to_pose(const Eigen::Matrix4d & matrix)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  const Eigen::Quaterniond q(matrix.block<3, 3>(0, 0));
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

// Leading vertices of the polyline up to (excluding) the first vertex that coincides with its
// predecessor. Coincident vertices would give the spline (near) zero-length bases.
std::vector<geometry_msgs::msg::Pose> leading_distinct_poses(
  const std::vector<Eigen::Matrix4d> & polyline)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(polyline.size());
  for (const auto & matrix : polyline) {
    const geometry_msgs::msg::Pose pose = matrix4d_to_pose(matrix);
    if (
      !poses.empty() &&
      autoware::experimental::trajectory::is_almost_same(poses.back().position, pose.position)) {
      break;
    }
    poses.push_back(pose);
  }
  return poses;
}

// (segment index + intra-segment ratio) of arc length s given the vertex arc lengths.
double arc_length_to_interpolation_index(const std::vector<double> & bases, const double s)
{
  const auto upper = std::upper_bound(bases.begin(), bases.end(), s);
  const int64_t segment = std::clamp<int64_t>(
    static_cast<int64_t>(std::distance(bases.begin(), upper)) - 1, 0,
    static_cast<int64_t>(bases.size()) - 2);
  const double segment_length = bases[segment + 1] - bases[segment];
  const double ratio = std::clamp((s - bases[segment]) / segment_length, 0.0, 1.0);
  return static_cast<double>(segment) + ratio;
}

std::optional<double> windowed_tangent_yaw(
  const autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Pose> & trajectory,
  const double s, const double half_window_m, const double min_length_m)
{
  // Symmetric window: shrink it when it would run past either end so the mean is not biased
  // toward one side of the snapped point (the point sits close to the start of a prediction).
  const double half = std::min({half_window_m, s, trajectory.length() - s});
  const double lo = s - half;
  const double hi = s + half;
  if (hi - lo < min_length_m) {
    return std::nullopt;
  }
  constexpr double SAMPLE_TICK_M = 0.05;
  const std::vector<double> azimuths =
    trajectory.azimuth(trajectory.base_arange({lo, hi}, SAMPLE_TICK_M));
  double sum_cos = 0.0;
  double sum_sin = 0.0;
  for (const double yaw : azimuths) {
    sum_cos += std::cos(yaw);
    sum_sin += std::sin(yaw);
  }
  return std::atan2(sum_sin, sum_cos);
}
}  // namespace

std::optional<TrajectorySnap> snap_point_to_trajectory(
  const double query_x, const double query_y, const std::vector<Eigen::Matrix4d> & polyline,
  const TrajectorySnapOptions & options)
{
  using autoware::experimental::trajectory::Trajectory;
  using autoware::experimental::trajectory::interpolator::CubicSpline;

  if (options.max_search_segment_count < 1) {
    throw std::runtime_error("snap_point_to_trajectory requires max_search_segment_count >= 1");
  }
  if (options.prefix_count < 0) {
    throw std::runtime_error("snap_point_to_trajectory requires prefix_count >= 0");
  }

  const std::vector<geometry_msgs::msg::Pose> poses = leading_distinct_poses(polyline);
  const size_t prefix = static_cast<size_t>(options.prefix_count);
  if (poses.size() < prefix + 2) {
    return std::nullopt;
  }
  const auto trajectory =
    Trajectory<geometry_msgs::msg::Pose>::Builder{}.set_xy_interpolator<CubicSpline>().build(poses);
  if (!trajectory) {
    return std::nullopt;
  }

  // Bases of the trajectory proper: vertex `prefix` is the trajectory start (s_start).
  const std::vector<double> all_bases = trajectory->get_underlying_bases();
  const std::vector<double> bases(all_bases.begin() + prefix, all_bases.end());
  const double search_start_s = bases.front();
  const double search_end_s = bases[std::min<size_t>(
    static_cast<size_t>(options.max_search_segment_count), bases.size() - 1)];
  geometry_msgs::msg::Point query;
  query.x = query_x;
  query.y = query_y;
  const std::optional<double> s = autoware::experimental::trajectory::closest_with_constraint(
    *trajectory, query, [search_start_s, search_end_s](const double & s) {
      return search_start_s <= s && s <= search_end_s;
    });
  if (!s) {
    return std::nullopt;
  }

  const geometry_msgs::msg::Pose snapped = trajectory->compute(*s);
  return TrajectorySnap{
    Eigen::Vector2d(snapped.position.x, snapped.position.y),
    arc_length_to_interpolation_index(bases, *s),
    autoware_utils_geometry::get_rpy(snapped.orientation).z,
    windowed_tangent_yaw(
      *trajectory, *s, options.yaw_fit_half_window_m, options.yaw_fit_min_length_m)};
  ;
}

BoundedPose bound_snapped_pose(
  const Eigen::Vector2d & real_position, const double real_yaw,
  const Eigen::Vector2d & snapped_position, const double snapped_yaw, const double snap_strength,
  const double max_position_error_m, const double max_yaw_error_rad)
{
  if (snap_strength < 0.0 || snap_strength > 1.0) {
    throw std::runtime_error("bound_snapped_pose requires snap_strength in [0, 1]");
  }
  if (max_position_error_m <= 0.0 || max_yaw_error_rad <= 0.0) {
    throw std::runtime_error("bound_snapped_pose requires positive error limits");
  }
  Eigen::Vector2d residual = snap_strength * (real_position - snapped_position);
  const double norm = residual.norm();
  if (norm > max_position_error_m) {
    residual *= max_position_error_m / norm;
  }

  const double yaw_residual = std::clamp(
    snap_strength * autoware_utils_math::normalize_radian(real_yaw - snapped_yaw),
    -max_yaw_error_rad, max_yaw_error_rad);

  return BoundedPose{
    real_position - residual, autoware_utils_math::normalize_radian(real_yaw - yaw_residual)};
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
