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

#ifndef AUTOWARE__DIFFUSION_PLANNER__UTILS__UTILS_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__UTILS__UTILS_HPP_

#include <Eigen/Dense>

#include "nav_msgs/msg/odometry.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::utils
{

/**
 * @brief Creates a vector of floats initialized with a specific value.
 *
 * @param shape A vector specifying the dimensions of the data (e.g., rows, columns).
 * @param fill The value to initialize the vector with. Defaults to 1.0f.
 * @return A flattened vector of floats with the specified shape and initialized values.
 */
std::vector<float> create_float_data(const std::vector<int64_t> & shape, float fill = 1.0f);

/**
 * @brief Checks if the input map contains valid data.
 *
 * @param input_map An unordered_map with string keys and vector<float> values.
 * @return True if the input map is valid, false otherwise.
 */
bool check_input_map(const std::unordered_map<std::string, std::vector<float>> & input_map);

/**
 * @brief Converts a geometry_msgs::msg::Pose to a 4x4 transformation matrix.
 *
 * @param pose The pose containing position and orientation information.
 * @return A 4x4 transformation matrix representing the pose.
 */
Eigen::Matrix4d pose_to_matrix4d(const geometry_msgs::msg::Pose & pose);

/**
 * @brief Extracts yaw angle from rotation matrix and converts to cos/sin representation.
 *
 * @param rotation_matrix 3x3 rotation matrix.
 * @return A pair containing cos(yaw) and sin(yaw).
 */
std::pair<float, float> rotation_matrix_to_cos_sin(const Eigen::Matrix3d & rotation_matrix);

/**
 * @brief Shifts the pose along the x-axis by a specified length.
 *
 * @param pose The pose to shift.
 * @param shift_length The length to shift the pose along the x-axis.
 * @return The shifted pose.
 */
geometry_msgs::msg::Pose shift_x(const geometry_msgs::msg::Pose & pose, const double shift_length);

/**
 * @brief Options for snap_point_to_trajectory.
 */
struct TrajectorySnapOptions
{
  //! Number of leading vertices of the polyline that precede the trajectory proper (e.g. the ego
  //! poses of earlier frames). They extend the spline backwards so the tangent window can stay
  //! symmetric and long close to the trajectory start, but are never selected as the closest point
  //! and do not count in interpolation_index (vertex prefix_count is index 0).
  int64_t prefix_count;
  //! Number of leading segments of the trajectory (after the prefix) the closest point is searched
  //! in. A planning cycle only advances the ego by about one segment, so a small window is enough
  //! and it keeps a far-away part of the trajectory (e.g. the return leg of a U-turn) from being
  //! selected.
  int64_t max_search_segment_count;
  //! Arc length [m] on each side of the snapped point over which the spline tangent is averaged.
  //! The window is kept symmetric around the snapped point (shrunk when it would run past either
  //! end of the trajectory) so the averaged heading is not biased toward the future path.
  double yaw_fit_half_window_m;
  //! Minimum total arc length [m] of that window for the tangent to be considered reliable.
  double yaw_fit_min_length_m;
};

/**
 * @brief Result of snapping a query point onto a trajectory.
 */
struct TrajectorySnap
{
  //! Closest point on the spline through the trajectory vertices (xy, same frame as the input).
  Eigen::Vector2d position;
  //! Position of the snapped point along the trajectory, expressed as (segment index +
  //! intra-segment ratio in [0, 1]). Multiplying by the per-segment time step yields the
  //! interpolation time of the snapped point along the trajectory.
  double interpolation_index;
  //! Heading [rad] spherically interpolated from the vertex orientations at the snapped point.
  double heading_yaw;
  //! Heading [rad] from the geometry: circular mean of the spline tangent over
  //! +-yaw_fit_half_window_m of arc length around the snapped point. std::nullopt when the
  //! available window is shorter than yaw_fit_min_length_m (no reliable heading information).
  std::optional<double> tangent_yaw;
};

/**
 * @brief Snaps a 2D query point onto a trajectory given as a sequence of poses.
 *
 * A cubic-spline trajectory (autoware::experimental::trajectory) is built through the leading
 * vertices of the polyline, stopping at the first pair of (almost) coincident consecutive vertices
 * so a stop at the end of a prediction does not degenerate the spline. The closest point on the
 * spline within the first max_search_segment_count segments is returned, together with two
 * headings: the interpolated vertex heading and the geometric tangent averaged over a window of
 * arc length (robust to noise in the individual vertex headings and to jitter of the short leading
 * segments of a predicted trajectory).
 *
 * @param query_x X coordinate of the query point.
 * @param query_y Y coordinate of the query point.
 * @param polyline Sequence of poses (4x4 transforms): options.prefix_count leading poses followed
 *        by the trajectory.
 * @param options See TrajectorySnapOptions.
 * @return The snap result, or std::nullopt when fewer than two distinct vertices remain past the
 *         prefix or the spline could not be built.
 * @throw std::runtime_error if options.max_search_segment_count is less than one or
 *        options.prefix_count is negative.
 */
std::optional<TrajectorySnap> snap_point_to_trajectory(
  double query_x, double query_y, const std::vector<Eigen::Matrix4d> & polyline,
  const TrajectorySnapOptions & options);

/**
 * @brief Pose (xy + yaw) resulting from bounding a snapped pose to the real one.
 */
struct BoundedPose
{
  Eigen::Vector2d position;
  double yaw;
};

/**
 * @brief Bounds a snapped pose by the real pose so the two can never drift apart by more than the
 *        given limits, without a discontinuity.
 *
 * The residual (real - snapped) is scaled by snap_strength: a strength of 0 returns the real pose
 * and so disables the feature, a strength of 1 returns the snapped pose, and anything in between
 * places the virtual pose on the segment between them. The scaled
 * residual is then saturated at max_position_error_m and max_yaw_error_rad, so the returned pose
 * is always within those limits of the real pose. Unlike rejecting the snap when a limit is
 * exceeded, this is continuous in the inputs: the virtual pose tracks the plan while it is close
 * to reality and slides along with reality when it is not, instead of jumping between the two.
 *
 * @param real_position Real (localized) xy position.
 * @param real_yaw Real yaw [rad].
 * @param snapped_position Snapped xy position.
 * @param snapped_yaw Snapped yaw [rad].
 * @param snap_strength How far from the real pose toward the snapped pose the virtual pose is
 *        placed, in [0, 1]. 0 is the real pose (feature off), 1 is the snapped pose.
 * @param max_position_error_m Saturation of the position residual [m] (> 0).
 * @param max_yaw_error_rad Saturation of the yaw residual [rad] (> 0).
 * @throw std::runtime_error on an out-of-range strength or a non-positive limit.
 */
BoundedPose bound_snapped_pose(
  const Eigen::Vector2d & real_position, double real_yaw, const Eigen::Vector2d & snapped_position,
  double snapped_yaw, double snap_strength, double max_position_error_m, double max_yaw_error_rad);

/**
 * @brief Computes the inverse of a 4x4 transformation matrix.
 * @note This function assumes that the matrix represents a rigid transformation and uses the
 * properties of Eigen::Isometry3d internally instead of a general 4x4 matrix inversion for better
 * numerical stability and performance.
 * @param mat The transformation matrix to invert.
 * @return A 4x4 transformation matrix representing the inverse.
 */
Eigen::Matrix4d inverse(const Eigen::Matrix4d & mat);

/**
 * @brief Replicate single sample data for batch processing.
 * @param single_data Single sample data.
 * @param batch_size The number of times to replicate the data.
 * @return Vector replicated for the specified batch size.
 */
std::vector<float> replicate_for_batch(
  const std::vector<float> & single_data, const int batch_size);

}  // namespace autoware::diffusion_planner::utils
#endif  // AUTOWARE__DIFFUSION_PLANNER__UTILS__UTILS_HPP_
