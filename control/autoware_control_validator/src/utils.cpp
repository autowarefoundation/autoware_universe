// Copyright 2023-2025 TIER IV, Inc.
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

#include "autoware/control_validator/utils.hpp"

#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/interpolation.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <optional>

namespace autoware::control_validator
{

using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;

namespace detail
{

inline TrajectoryPoints reverse_trajectory_points(const TrajectoryPoints & trajectory_points)
{
  return TrajectoryPoints(trajectory_points.crbegin(), trajectory_points.crend());
}

/**
 * @brief Remove trajectory points which are in front of the reference segment
 * and insert interpolated point at the boundary if needed.
 *
 * To avoid front removal and insertion, trajectory is given in reversed order.
 * @param[in] segment_points reference segment trajectory of length at least 2
 * @param[inout] reversed_trajectory_points trajectory in reversed order, possibly empty
 */
void clamp_trajectory_head(
  const TrajectoryPoints & segment_points, TrajectoryPoints & reversed_trajectory_points)
{
  if (segment_points.size() < 2) {
    return;
  }
  std::optional<TrajectoryPoint> opt_last_removed_point = std::nullopt;
  while (!reversed_trajectory_points.empty()) {
    const auto & point = reversed_trajectory_points.back();
    if (
      autoware::motion_utils::calcLongitudinalOffsetToSegment(
        segment_points, 0, point.pose.position) >= 0.0) {
      // reached the overlapping boundary
      break;
    }
    opt_last_removed_point.emplace(point);
    reversed_trajectory_points.pop_back();
  }
  if (reversed_trajectory_points.empty() || !opt_last_removed_point.has_value()) {
    // boundary segment does not exist and nothing to insert; either the predicted trajectory is
    // fully consumed, or the referenced segment is ahead of the predicted trajectory
    return;
  }
  // we don't need the whole segments of the predicted trajectory for interpolation
  TrajectoryPoints boundary_segment_points = {
    opt_last_removed_point.value(), reversed_trajectory_points.back()};
  auto interpolated_point = autoware::motion_utils::calcInterpolatedPoint(
    autoware::motion_utils::convertToTrajectory(boundary_segment_points),
    segment_points.front().pose);
  reversed_trajectory_points.emplace_back(interpolated_point);
}

/**
 * @brief Align the predicted trajectory with the reference trajectory by trimming the parts which
 * are out of the reference trajectory range.
 * @param reference_trajectory_points reference trajectory
 * @param predicted_trajectory_points predicted trajectory
 * @return aligned predicted trajectory
 */
TrajectoryPoints align_trajectory_with_reference_trajectory(
  const TrajectoryPoints & reference_trajectory_points,
  const TrajectoryPoints & predicted_trajectory_points)
{
  if (reference_trajectory_points.size() < 2 || predicted_trajectory_points.empty()) {
    return TrajectoryPoints();
  }

  // To reduce memory consumption, limit the maximum size of the predicted trajectory to calculate.
  // In C++20, copying can be prevented by using e.g. `std::span`.
  constexpr size_t MAX_TRAJECTORY_SIZE = 5000;
  if (predicted_trajectory_points.size() > MAX_TRAJECTORY_SIZE) {
    TrajectoryPoints safe_predicted_trajectory_points(
      predicted_trajectory_points.cbegin(),
      predicted_trajectory_points.cbegin() + MAX_TRAJECTORY_SIZE);
    return align_trajectory_with_reference_trajectory(
      reference_trajectory_points, safe_predicted_trajectory_points);
  }

  // We don't need to consider the entire trajectory; only the first and last segments are needed.
  TrajectoryPoints first_segment_points = {
    reference_trajectory_points[0], reference_trajectory_points[1]};
  TrajectoryPoints reversed_last_segment_points = {
    reference_trajectory_points[reference_trajectory_points.size() - 1],
    reference_trajectory_points[reference_trajectory_points.size() - 2]};

  // If no overlapping between reference_ and predicted_trajectory, return empty trajectory.
  //
  // predicted_trajectory:   p1------------------pN
  // reference_trajectory:                           r1------------------rN
  //     OR
  // predicted_trajectory:                           p1------------------pN
  // reference_trajectory:   r1------------------rN
  const bool is_p_n_before_r1 =
    autoware::motion_utils::calcLongitudinalOffsetToSegment(
      first_segment_points, 0, predicted_trajectory_points.back().pose.position) < 0.0;
  const bool is_p1_behind_r_n =
    autoware::motion_utils::calcLongitudinalOffsetToSegment(
      reversed_last_segment_points, 0, predicted_trajectory_points.front().pose.position) < 0.0;
  const bool is_no_overlapping = (is_p_n_before_r1 || is_p1_behind_r_n);

  if (is_no_overlapping) {
    return TrajectoryPoints();
  }

  // If first point of predicted_trajectory is in front of reference_trajectory, remove points which
  // are in front of reference_trajectory and insert pNew along the predicted_trajectory.
  //
  // predicted_trajectory:           p1-----p2-----p3----//------pN
  // reference_trajectory:                     r1--------//------rN
  // ↓
  // predicted_trajectory:                   pNew--p3----//------pN
  // reference_trajectory:                     r1--------//------rN
  auto reversed_aligned_trajectory_points = reverse_trajectory_points(predicted_trajectory_points);
  clamp_trajectory_head(first_segment_points, reversed_aligned_trajectory_points);

  // If last point of predicted_trajectory is behind reference_trajectory, remove points which
  // are behind reference_trajectory and insert pNew along the predicted_trajectory
  //
  // predicted_trajectory:           p1-----//------pN-2-----pN-1-----pN
  // reference_trajectory:           r1-----//-----rN-1--rN
  // ↓
  // predicted_trajectory:           p1-----//------pN-2-pNew
  // reference_trajectory:           r1-----//-----rN-1--rN

  auto aligned_trajectory_points = reverse_trajectory_points(reversed_aligned_trajectory_points);
  clamp_trajectory_head(reversed_last_segment_points, aligned_trajectory_points);

  return aligned_trajectory_points;
}

}  // namespace detail

void shift_pose(Pose & pose, double longitudinal)
{
  const auto yaw = tf2::getYaw(pose.orientation);
  pose.position.x += std::cos(yaw) * longitudinal;
  pose.position.y += std::sin(yaw) * longitudinal;
}

double calc_max_lateral_distance(
  const Trajectory & reference_trajectory, const Trajectory & predicted_trajectory)
{
  const auto aligned_predicted_trajectory_points =
    detail::align_trajectory_with_reference_trajectory(
      reference_trajectory.points, predicted_trajectory.points);
  double max_dist = 0;
  for (const auto & point : aligned_predicted_trajectory_points) {
    const auto p0 = autoware_utils::get_point(point);
    // find nearest segment
    const size_t nearest_segment_idx =
      autoware::motion_utils::findNearestSegmentIndex(reference_trajectory.points, p0);
    const double temp_dist = std::abs(
      autoware::motion_utils::calcLateralOffset(
        reference_trajectory.points, p0, nearest_segment_idx));
    if (temp_dist > max_dist) {
      max_dist = temp_dist;
    }
  }
  return max_dist;
}

}  // namespace autoware::control_validator
