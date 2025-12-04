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

#include "autoware/trajectory_optimizer/utils.hpp"

#include "autoware/trajectory/interpolator/akima_spline.hpp"
#include "autoware/trajectory/interpolator/interpolator.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/trajectory_point.hpp"
#include "autoware/trajectory_optimizer/trajectory_optimizer_structs.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_planning_msgs/msg/detail/trajectory_point__struct.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_optimizer::utils
{
using autoware::experimental::trajectory::interpolator::AkimaSpline;
using InterpolationTrajectory =
  autoware::experimental::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;

rclcpp::Logger get_logger()
{
  return rclcpp::get_logger("trajectory_optimizer");
}

void log_error_throttle(const std::string & message)
{
  auto clock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
  RCLCPP_ERROR_THROTTLE(get_logger(), *clock, 5000, "%s", message.c_str());
}

void log_warn_throttle(const std::string & message)
{
  auto clock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
  RCLCPP_WARN_THROTTLE(get_logger(), *clock, 5000, "%s", message.c_str());
}

void smooth_trajectory_with_elastic_band(
  TrajectoryPoints & traj_points, const Odometry & current_odometry,
  const std::shared_ptr<EBPathSmoother> & eb_path_smoother_ptr)
{
  if (!eb_path_smoother_ptr) {
    log_error_throttle("Elastic band path smoother is not initialized");
    return;
  }
  constexpr size_t minimum_points_for_elastic_band = 3;
  if (traj_points.empty() || traj_points.size() < minimum_points_for_elastic_band) {
    return;
  }
  traj_points = eb_path_smoother_ptr->smoothTrajectory(traj_points, current_odometry.pose.pose);
  eb_path_smoother_ptr->resetPreviousData();
}

double compute_dt(const TrajectoryPoint & current, const TrajectoryPoint & next)
{
  constexpr double min_dt_threshold = 1e-9;

  const double curr_time = static_cast<double>(current.time_from_start.sec) +
                           static_cast<double>(current.time_from_start.nanosec) * 1e-9;
  const double next_time = static_cast<double>(next.time_from_start.sec) +
                           static_cast<double>(next.time_from_start.nanosec) * 1e-9;

  return std::max(next_time - curr_time, min_dt_threshold);
}

void recalculate_longitudinal_acceleration(
  TrajectoryPoints & trajectory, const bool use_constant_dt, const double constant_dt)
{
  if (trajectory.size() < 2) {
    return;
  }

  auto get_dt = [&](const size_t i) -> double {
    constexpr double min_dt_threshold = 1e-9;
    if (use_constant_dt) {
      return std::max(constant_dt, min_dt_threshold);
    }
    return compute_dt(trajectory[i], trajectory[i + 1]);
  };

  const size_t size = trajectory.size();
  for (size_t i = 0; i + 1 < size; ++i) {
    const double dt = get_dt(i);
    const double dv = static_cast<double>(trajectory[i + 1].longitudinal_velocity_mps) -
                      static_cast<double>(trajectory[i].longitudinal_velocity_mps);
    trajectory[i].acceleration_mps2 = static_cast<float>(dv / dt);
  }
  trajectory.back().acceleration_mps2 = 0.0f;
}

bool validate_point(const TrajectoryPoint & point)
{
  auto is_valid = [](auto value) { return std::isfinite(value) && !std::isnan(value); };

  return is_valid(point.longitudinal_velocity_mps) && is_valid(point.acceleration_mps2) &&
         is_valid(point.pose.position.x) && is_valid(point.pose.position.y) &&
         is_valid(point.pose.position.z) && is_valid(point.pose.orientation.x) &&
         is_valid(point.pose.orientation.y) && is_valid(point.pose.orientation.z) &&
         is_valid(point.pose.orientation.w);
}

void copy_trajectory_orientation(
  const TrajectoryPoints & input_trajectory, TrajectoryPoints & output_trajectory,
  const double max_distance_m, const double max_yaw_rad)
{
  for (auto & out_point : output_trajectory) {
    const auto nearest_index_opt = autoware::motion_utils::findNearestIndex(
      input_trajectory, out_point.pose, max_distance_m, max_yaw_rad);
    if (!nearest_index_opt.has_value()) {
      continue;
    }
    const auto nearest_index = nearest_index_opt.value();
    out_point.pose.orientation = input_trajectory.at(nearest_index).pose.orientation;
  }
}

void apply_spline(
  TrajectoryPoints & traj_points, const double interpolation_resolution_m,
  const double max_distance_discrepancy_m, const bool preserve_original_orientation)
{
  constexpr size_t minimum_points_for_akima_spline = 5;
  if (traj_points.size() < minimum_points_for_akima_spline) {
    log_warn_throttle("Not enough points in trajectory for spline interpolation");
    return;
  }
  const TrajectoryPoints original_traj_points = traj_points;
  auto trajectory_interpolation_util =
    InterpolationTrajectory::Builder{}
      .set_xy_interpolator<AkimaSpline>()  // Set interpolator for x-y plane
      .build(traj_points);
  if (!trajectory_interpolation_util) {
    log_warn_throttle("Failed to build interpolation trajectory");
    return;
  }
  trajectory_interpolation_util->align_orientation_with_trajectory_direction();
  TrajectoryPoints output_points{traj_points.front()};
  constexpr double min_interpolation_step = 1e-2;
  const auto ds = std::max(interpolation_resolution_m, min_interpolation_step);
  output_points.reserve(static_cast<size_t>(trajectory_interpolation_util->length() / ds));

  for (auto s = ds; s <= trajectory_interpolation_util->length(); s += ds) {
    auto p = trajectory_interpolation_util->compute(s);
    if (!validate_point(p)) {
      continue;
    }
    output_points.push_back(p);
  }

  if (output_points.size() < 2) {
    log_warn_throttle("Not enough points in trajectory after akima spline interpolation");
    return;
  }
  auto last_interpolated_point = output_points.back();
  auto & original_trajectory_last_point = traj_points.back();

  if (!validate_point(original_trajectory_last_point)) {
    log_warn_throttle("Last point in original trajectory is invalid. Removing last point");
    traj_points = output_points;
    return;
  }

  auto d = autoware_utils::calc_distance2d(
    last_interpolated_point.pose.position, original_trajectory_last_point.pose.position);
  if (d > min_interpolation_step) {
    output_points.push_back(original_trajectory_last_point);
  }

  if (preserve_original_orientation) {
    copy_trajectory_orientation(
      original_traj_points, output_points, max_distance_discrepancy_m, M_PI);
  }

  traj_points = output_points;
}

}  // namespace autoware::trajectory_optimizer::utils
