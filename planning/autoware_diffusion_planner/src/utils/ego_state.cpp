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

#include "autoware/diffusion_planner/utils/ego_state.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <utility>

namespace autoware::diffusion_planner::utils
{

namespace
{
using nav_msgs::msg::Odometry;

// Linearly interpolate pose and twist between two samples; ratio 0 returns `earlier`, 1 `later`.
Odometry interpolate_ego_state(const Odometry & earlier, const Odometry & later, const double ratio)
{
  Odometry interpolated = earlier;
  interpolated.pose.pose = autoware_utils_geometry::calc_interpolated_pose(
    earlier.pose.pose, later.pose.pose, ratio, false);
  const auto lerp = [ratio](const double a, const double b) { return a + (b - a) * ratio; };
  auto & lin = interpolated.twist.twist.linear;
  auto & ang = interpolated.twist.twist.angular;
  lin.x = lerp(earlier.twist.twist.linear.x, later.twist.twist.linear.x);
  lin.y = lerp(earlier.twist.twist.linear.y, later.twist.twist.linear.y);
  lin.z = lerp(earlier.twist.twist.linear.z, later.twist.twist.linear.z);
  ang.x = lerp(earlier.twist.twist.angular.x, later.twist.twist.angular.x);
  ang.y = lerp(earlier.twist.twist.angular.y, later.twist.twist.angular.y);
  ang.z = lerp(earlier.twist.twist.angular.z, later.twist.twist.angular.z);
  return interpolated;
}
}  // namespace

std::pair<Odometry, double> select_ego_state(
  const std::deque<Odometry> & ego_history, const rclcpp::Time & frame_time,
  const bool use_time_interpolation)
{
  // Nearest sample: used when interpolation is off, and as the fallback outside the buffer range.
  const Odometry * nearest = &ego_history.front();
  double min_time_diff_s = std::numeric_limits<double>::max();
  for (const auto & candidate : ego_history) {
    const double time_diff_s =
      std::abs((rclcpp::Time(candidate.header.stamp) - frame_time).seconds());
    if (time_diff_s < min_time_diff_s) {
      min_time_diff_s = time_diff_s;
      nearest = &candidate;
    }
  }

  if (!use_time_interpolation) {
    return {*nearest, min_time_diff_s};
  }

  const double frame_sec = frame_time.seconds();
  for (size_t i = 0; i + 1 < ego_history.size(); ++i) {
    const double t0 = rclcpp::Time(ego_history[i].header.stamp).seconds();
    const double t1 = rclcpp::Time(ego_history[i + 1].header.stamp).seconds();
    if (frame_sec >= t0 && frame_sec <= t1) {
      const double ratio = (t1 > t0) ? (frame_sec - t0) / (t1 - t0) : 0.0;
      return {interpolate_ego_state(ego_history[i], ego_history[i + 1], ratio), 0.0};
    }
  }
  // frame_time is outside the buffered range: fall back to the nearest sample.
  return {*nearest, min_time_diff_s};
}

}  // namespace autoware::diffusion_planner::utils
