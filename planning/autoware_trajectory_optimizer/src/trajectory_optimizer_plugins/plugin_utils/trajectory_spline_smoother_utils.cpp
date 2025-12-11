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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_spline_smoother_utils.hpp"

#include "autoware/trajectory/interpolator/akima_spline.hpp"
#include "autoware/trajectory/interpolator/interpolator.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/trajectory_point.hpp"
#include "autoware/trajectory_optimizer/utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::trajectory_optimizer::plugin::trajectory_spline_smoother_utils
{

using autoware::experimental::trajectory::interpolator::AkimaSpline;
using InterpolationTrajectory =
  autoware::experimental::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;

void apply_spline(
  TrajectoryPoints & traj_points, const double interpolation_resolution_m,
  const double max_distance_discrepancy_m, const bool preserve_original_orientation)
{
  constexpr size_t minimum_points_for_akima_spline = 5;
  if (traj_points.size() < minimum_points_for_akima_spline) {
    auto clock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("trajectory_spline_smoother"), *clock, 5000,
      "Not enough points in trajectory for spline interpolation");
    return;
  }
  const TrajectoryPoints original_traj_points = traj_points;
  auto trajectory_interpolation_util =
    InterpolationTrajectory::Builder{}
      .set_xy_interpolator<AkimaSpline>()  // Set interpolator for x-y plane
      .build(traj_points);
  if (!trajectory_interpolation_util) {
    auto clock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("trajectory_spline_smoother"), *clock, 5000,
      "Failed to build interpolation trajectory");
    return;
  }
  trajectory_interpolation_util->align_orientation_with_trajectory_direction();
  TrajectoryPoints output_points{traj_points.front()};
  constexpr double min_interpolation_step = 1e-2;
  const auto ds = std::max(interpolation_resolution_m, min_interpolation_step);
  output_points.reserve(static_cast<size_t>(trajectory_interpolation_util->length() / ds));

  for (auto s = ds; s <= trajectory_interpolation_util->length(); s += ds) {
    auto p = trajectory_interpolation_util->compute(s);
    if (!autoware::trajectory_optimizer::utils::validate_point(p)) {
      continue;
    }
    output_points.push_back(p);
  }

  if (output_points.size() < 2) {
    auto clock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("trajectory_spline_smoother"), *clock, 5000,
      "Not enough points in trajectory after akima spline interpolation");
    return;
  }
  const auto & last_interpolated_point = output_points.back();
  auto & original_trajectory_last_point = traj_points.back();

  if (!autoware::trajectory_optimizer::utils::validate_point(original_trajectory_last_point)) {
    auto clock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("trajectory_spline_smoother"), *clock, 5000,
      "Last point in original trajectory is invalid. Removing last point");
    traj_points = output_points;
    return;
  }

  auto d = autoware_utils_geometry::calc_distance2d(
    last_interpolated_point.pose.position, original_trajectory_last_point.pose.position);
  if (d > min_interpolation_step) {
    output_points.push_back(original_trajectory_last_point);
  }

  if (preserve_original_orientation) {
    autoware::trajectory_optimizer::utils::copy_trajectory_orientation(
      original_traj_points, output_points, max_distance_discrepancy_m, M_PI);
  }

  traj_points = output_points;
}

}  // namespace autoware::trajectory_optimizer::plugin::trajectory_spline_smoother_utils
