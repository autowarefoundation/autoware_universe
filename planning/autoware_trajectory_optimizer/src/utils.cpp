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

void clamp_velocities(
  TrajectoryPoints & input_trajectory_array, float min_velocity, float min_acceleration)
{
  std::for_each(
    input_trajectory_array.begin(), input_trajectory_array.end(),
    [min_velocity, min_acceleration](TrajectoryPoint & point) {
      point.longitudinal_velocity_mps = std::max(point.longitudinal_velocity_mps, min_velocity);
      point.acceleration_mps2 = std::max(point.acceleration_mps2, min_acceleration);
    });
}

void set_max_velocity(TrajectoryPoints & input_trajectory_array, const float max_velocity)
{
  if (input_trajectory_array.empty()) {
    return;
  }

  // Handle single-point trajectory
  if (input_trajectory_array.size() == 1) {
    input_trajectory_array[0].longitudinal_velocity_mps =
      std::min(input_trajectory_array[0].longitudinal_velocity_mps, max_velocity);
    input_trajectory_array[0].acceleration_mps2 = 0.0f;
    return;
  }

  std::vector<std::pair<size_t, size_t>> modified_segment_indices;
  std::vector<double> original_dts;
  original_dts.reserve(input_trajectory_array.size() - 1);

  // Lambda to compute dt from velocity change and acceleration
  auto compute_dt_using_velocity_and_acc =
    [](const TrajectoryPoint & from, const TrajectoryPoint & to) -> double {
    const auto dv = static_cast<double>(to.longitudinal_velocity_mps) -
                    static_cast<double>(from.longitudinal_velocity_mps);
    const auto acc = static_cast<double>(from.acceleration_mps2);
    constexpr double epsilon_acceleration = 1e-6;
    const auto denominator_acc =
      std::abs(acc) < epsilon_acceleration ? epsilon_acceleration : std::abs(acc);
    return std::abs(dv) / denominator_acc;
  };

  // Store original dt values computed from velocity and acceleration
  for (size_t i = 0; i < input_trajectory_array.size() - 1; ++i) {
    original_dts.push_back(
      compute_dt_using_velocity_and_acc(input_trajectory_array[i], input_trajectory_array[i + 1]));
  }

  // Identify segments where velocity exceeds max_velocity
  size_t segment_start = 0;
  bool in_segment = false;

  for (size_t i = 0; i < input_trajectory_array.size(); ++i) {
    const bool exceeds_max = input_trajectory_array[i].longitudinal_velocity_mps > max_velocity;

    if (exceeds_max && !in_segment) {
      // Start of new segment
      segment_start = i;
      in_segment = true;
    } else if (!exceeds_max && in_segment) {
      // End of segment
      modified_segment_indices.emplace_back(segment_start, i - 1);
      in_segment = false;
    }
  }

  // Handle case where segment extends to end of trajectory
  if (in_segment) {
    modified_segment_indices.emplace_back(segment_start, input_trajectory_array.size() - 1);
  }

  // Cap velocities and set accelerations in offending segments
  for (const auto & [start, end] : modified_segment_indices) {
    // Cap velocities for all points in segment
    for (size_t i = start; i <= end; ++i) {
      input_trajectory_array[i].longitudinal_velocity_mps = max_velocity;
    }

    // Set acceleration to 0 for all intermediate points (constant velocity)
    // Points from start to end-1 have no velocity change to next point
    for (size_t i = start; i < end; ++i) {
      input_trajectory_array[i].acceleration_mps2 = 0.0f;
    }
  }

  // Recalculate accelerations at segment boundaries (transitions)
  for (const auto & [start, end] : modified_segment_indices) {
    // Update acceleration for point right before segment start (transition INTO segment)
    // Skip if segment starts at index 0 (no point before it)
    if (start > 0) {
      const size_t idx_before = start - 1;
      const double dt = original_dts[idx_before];
      const auto v_before =
        static_cast<double>(input_trajectory_array[idx_before].longitudinal_velocity_mps);
      const auto v_start =
        static_cast<double>(input_trajectory_array[start].longitudinal_velocity_mps);
      const double new_acc = (v_start - v_before) / dt;
      input_trajectory_array[idx_before].acceleration_mps2 = static_cast<float>(new_acc);
    }

    // Update acceleration for last point of segment (transition OUT OF segment)
    // Skip if segment ends at last trajectory point (should remain 0.0)
    if (end < input_trajectory_array.size() - 1) {
      const double dt = original_dts[end];
      const auto v_end = static_cast<double>(input_trajectory_array[end].longitudinal_velocity_mps);
      const auto v_after =
        static_cast<double>(input_trajectory_array[end + 1].longitudinal_velocity_mps);
      const double new_acc = (v_after - v_end) / dt;
      input_trajectory_array[end].acceleration_mps2 = static_cast<float>(new_acc);
    }
  }

  // Ensure last point always has zero acceleration
  input_trajectory_array.back().acceleration_mps2 = 0.0f;
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

void limit_lateral_acceleration(
  TrajectoryPoints & input_trajectory_array, const double max_lateral_accel_mps2,
  const Odometry & current_odometry)
{
  if (input_trajectory_array.empty()) {
    return;
  }

  auto get_delta_time = [](const auto & next, const auto & current) -> double {
    return next->time_from_start.sec + next->time_from_start.nanosec * 1e-9 -
           (current->time_from_start.sec + current->time_from_start.nanosec * 1e-9);
  };

  const auto & current_position = current_odometry.pose.pose.position;
  motion_utils::calculate_time_from_start(input_trajectory_array, current_position);

  const auto closest_index =
    motion_utils::findNearestIndex(input_trajectory_array, current_position);
  const auto start_itr = std::next(
    input_trajectory_array.begin(),
    static_cast<std::vector<TrajectoryPoint>::difference_type>(closest_index));

  for (auto itr = start_itr; itr < std::prev(input_trajectory_array.end()); ++itr) {
    const auto current_pose = itr->pose;
    const auto next_pose = std::next(itr)->pose;
    const auto delta_time = get_delta_time(std::next(itr), itr);

    tf2::Quaternion q_current;
    tf2::Quaternion q_next;
    tf2::convert(current_pose.orientation, q_current);
    tf2::convert(next_pose.orientation, q_next);
    double delta_theta = q_current.angleShortestPath(q_next);
    // Handle wrap-around
    if (delta_theta > M_PI) {
      delta_theta -= 2.0 * M_PI;
    } else if (delta_theta < -M_PI) {
      delta_theta += 2.0 * M_PI;
    }

    const double yaw_rate = std::max(std::abs(delta_theta / delta_time), 1.0E-5);
    const double current_speed = std::abs(itr->longitudinal_velocity_mps);
    // Compute lateral acceleration
    const double lateral_acceleration = std::abs(current_speed * yaw_rate);
    if (lateral_acceleration < max_lateral_accel_mps2) continue;

    itr->longitudinal_velocity_mps = static_cast<float>(max_lateral_accel_mps2 / yaw_rate);
  }

  motion_utils::calculate_time_from_start(
    input_trajectory_array, current_odometry.pose.pose.position);

  // recalculate acceleration after velocity change
  recalculate_longitudinal_acceleration(input_trajectory_array);
}

void filter_velocity(
  TrajectoryPoints & input_trajectory, const InitialMotion & initial_motion,
  const double nearest_dist_threshold_m, const double nearest_yaw_threshold_rad,
  const std::shared_ptr<JerkFilteredSmoother> & smoother, const Odometry & current_odometry)
{
  if (!smoother) {
    log_error_throttle("JerkFilteredSmoother is not initialized");
    return;
  }

  if (input_trajectory.size() < 2) {
    return;
  }

  const auto & initial_motion_speed = initial_motion.speed_mps;
  const auto & initial_motion_acc = initial_motion.acc_mps2;

  constexpr bool enable_smooth_limit = true;
  constexpr bool use_resampling = true;

  input_trajectory = smoother->applyLateralAccelerationFilter(
    input_trajectory, initial_motion_speed, initial_motion_acc, enable_smooth_limit,
    use_resampling);

  // Steering angle rate limit (Note: set use_resample = false since it is resampled above)
  input_trajectory = smoother->applySteeringRateLimit(input_trajectory, false);
  // Resample trajectory with ego-velocity based interval distance

  input_trajectory = smoother->resampleTrajectory(
    input_trajectory, initial_motion_speed, current_odometry.pose.pose, nearest_dist_threshold_m,
    nearest_yaw_threshold_rad);

  const size_t traj_closest = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    input_trajectory, current_odometry.pose.pose, nearest_dist_threshold_m,
    nearest_yaw_threshold_rad);

  // // Clip trajectory from closest point
  TrajectoryPoints clipped;
  clipped.insert(
    clipped.end(),
    input_trajectory.begin() + static_cast<TrajectoryPoints::difference_type>(traj_closest),
    input_trajectory.end());
  input_trajectory = clipped;

  std::vector<TrajectoryPoints> debug_trajectories;
  if (!smoother->apply(
        initial_motion_speed, initial_motion_acc, input_trajectory, input_trajectory,
        debug_trajectories, false)) {
    log_warn_throttle("Fail to solve optimization.");
  }
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
