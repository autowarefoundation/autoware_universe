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

#include "autoware/trajectory_ranker/utils.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <vector>

namespace autoware::trajectory_ranker::utils
{

namespace
{
inline double to_seconds(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}

std::optional<size_t> find_segment_index(
  const std::vector<double> & times, double t, size_t start_index)
{
  if (times.size() < 2) return std::nullopt;
  const size_t start = std::min(start_index, times.size() - 1);

  if (t < times.front()) return std::nullopt;
  if (t >= times.back()) return std::nullopt;

  auto it = std::upper_bound(times.begin() + start, times.end(), t);

  if (it == times.begin() || it == times.end()) return std::nullopt;
  return static_cast<size_t>(std::distance(times.begin(), it)) - 1;
}
}  // namespace

TrajectoryPoint calc_interpolated_point(
  const TrajectoryPoint & curr_pt, const TrajectoryPoint & next_pt, const double ratio,
  const bool use_zero_order_hold_for_twist)
{
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  TrajectoryPoint interpolated_point{};

  // pose interpolation
  interpolated_point.pose =
    autoware_utils_geometry::calc_interpolated_pose(curr_pt, next_pt, ratio);

  // twist interpolation
  if (use_zero_order_hold_for_twist) {
    interpolated_point.longitudinal_velocity_mps = curr_pt.longitudinal_velocity_mps;
    interpolated_point.lateral_velocity_mps = curr_pt.lateral_velocity_mps;
    interpolated_point.acceleration_mps2 = curr_pt.acceleration_mps2;
  } else {
    interpolated_point.longitudinal_velocity_mps = autoware::interpolation::lerp(
      curr_pt.longitudinal_velocity_mps, next_pt.longitudinal_velocity_mps, clamped_ratio);
    interpolated_point.lateral_velocity_mps = autoware::interpolation::lerp(
      curr_pt.lateral_velocity_mps, next_pt.lateral_velocity_mps, clamped_ratio);
    interpolated_point.acceleration_mps2 = autoware::interpolation::lerp(
      curr_pt.acceleration_mps2, next_pt.acceleration_mps2, clamped_ratio);
  }

  // heading rate interpolation
  interpolated_point.heading_rate_rps = autoware::interpolation::lerp(
    curr_pt.heading_rate_rps, next_pt.heading_rate_rps, clamped_ratio);

  // wheel interpolation
  interpolated_point.front_wheel_angle_rad = autoware::interpolation::lerp(
    curr_pt.front_wheel_angle_rad, next_pt.front_wheel_angle_rad, clamped_ratio);
  interpolated_point.rear_wheel_angle_rad = autoware::interpolation::lerp(
    curr_pt.rear_wheel_angle_rad, next_pt.rear_wheel_angle_rad, clamped_ratio);

  // time interpolation
  const double t1 = to_seconds(curr_pt.time_from_start);
  const double t2 = to_seconds(next_pt.time_from_start);
  const double t = (t2 == t1) ? t1 : autoware::interpolation::lerp(t1, t2, clamped_ratio);
  interpolated_point.time_from_start = rclcpp::Duration::from_seconds(t);

  return interpolated_point;
}

TrajectoryPoints sampling(
  const TrajectoryPoints & points, const Pose & p_ego, const size_t sample_num,
  const double resolution)
{
  TrajectoryPoints output;
  if (points.empty() || sample_num == 0) return output;

  const auto ego_idx_opt = autoware::motion_utils::findNearestIndex(points, p_ego, 10.0, M_PI_2);

  if (!ego_idx_opt.has_value()) return output;
  const size_t ego_idx = std::min(ego_idx_opt.value(), points.size() - 1);

  // time vector
  std::vector<double> times;
  times.reserve(points.size());
  for (const auto & pt : points) times.push_back(to_seconds(pt.time_from_start));

  const double t0 = times[ego_idx];
  output.reserve(sample_num);

  for (size_t i = 0; i < sample_num; i++) {
    const double elapsed_time = t0 + static_cast<double>(i) * resolution;
    const auto seg_opt = find_segment_index(times, elapsed_time, ego_idx);
    if (!seg_opt.has_value()) {
      output.push_back(points.back());
      continue;
    }
    const size_t index = seg_opt.value();
    const double t1 = times[index];
    const double t2 = times[index + 1];

    if (t2 == t1) {
      output.push_back(points.at(index));
      continue;
    }

    const double ratio = (elapsed_time - t1) / (t2 - t1);
    output.push_back(calc_interpolated_point(points.at(index), points.at(index + 1), ratio, false));
  }
  return output;
}
}  // namespace autoware::trajectory_ranker::utils
