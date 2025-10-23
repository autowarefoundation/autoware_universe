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
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::utils
{
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
}

void remove_invalid_points(TrajectoryPoints & input_trajectory)
{
  // remove points with nan or inf values
  input_trajectory.erase(
    std::remove_if(
      input_trajectory.begin(), input_trajectory.end(),
      [](const TrajectoryPoint & point) { return !validate_point(point); }),
    input_trajectory.end());

  utils::remove_close_proximity_points(input_trajectory, 1E-2);

  if (input_trajectory.size() < 2) {
    log_error_throttle(
      "Not enough points in trajectory after removing close proximity points and invalid points");
    return;
  }
}

void remove_close_proximity_points(TrajectoryPoints & input_trajectory_array, const double min_dist)
{
  if (std::size(input_trajectory_array) < 2) {
    return;
  }

  input_trajectory_array.erase(
    std::remove_if(
      std::next(input_trajectory_array.begin()),  // Start from second element
      input_trajectory_array.end(),
      [&](const TrajectoryPoint & point) {
        const auto prev_it = std::prev(&point);
        const auto dist = autoware_utils::calc_distance2d(point, *prev_it);
        return dist < min_dist;
      }),
    input_trajectory_array.end());
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
  std::for_each(
    input_trajectory_array.begin(), input_trajectory_array.end(),
    [max_velocity](TrajectoryPoint & point) {
      point.longitudinal_velocity_mps = std::min(point.longitudinal_velocity_mps, max_velocity);
    });

  // recalculate acceleration after velocity change
  const int64_t size = input_trajectory_array.size();
  for (int64_t i = 0; i + 1 < size; ++i) {
    const float curr_time_from_start =
      static_cast<float>(input_trajectory_array[i].time_from_start.sec) +
      static_cast<float>(input_trajectory_array[i].time_from_start.nanosec) * 1e-9f;
    const float next_time_from_start =
      static_cast<float>(input_trajectory_array[i + 1].time_from_start.sec) +
      static_cast<float>(input_trajectory_array[i + 1].time_from_start.nanosec) * 1e-9f;
    const float dt = next_time_from_start - curr_time_from_start;
    const float dv = input_trajectory_array[i + 1].longitudinal_velocity_mps -
                     input_trajectory_array[i].longitudinal_velocity_mps;
    input_trajectory_array[i].acceleration_mps2 = dv / (dt + 1e-5f);
  }
  input_trajectory_array.back().acceleration_mps2 = 0.0f;
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

    itr->longitudinal_velocity_mps = max_lateral_accel_mps2 / yaw_rate;
  }

  motion_utils::calculate_time_from_start(
    input_trajectory_array, current_odometry.pose.pose.position);
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

void fix_trajectory_orientation(
  const TrajectoryPoints & input_trajectory, TrajectoryPoints & output_trajectory,
  const double yaw_threshold_rad)
{
  for (auto & point : output_trajectory) {
    const auto nearest_index_opt =
      autoware::motion_utils::findNearestIndex(input_trajectory, point.pose);

    if (!nearest_index_opt.has_value()) {
      continue;
    }

    const size_t nearest_idx = nearest_index_opt.value();

    // Get yaw from both orientations
    const double input_yaw = tf2::getYaw(input_trajectory[nearest_idx].pose.orientation);
    const double output_yaw = tf2::getYaw(point.pose.orientation);

    // Calculate yaw difference (normalized to [-pi, pi])
    const double yaw_diff = autoware_utils_math::normalize_radian(output_yaw - input_yaw);

    // If difference exceeds threshold, use original orientation
    if (std::abs(yaw_diff) > yaw_threshold_rad) {
      point.pose.orientation = input_trajectory[nearest_idx].pose.orientation;
    }
  }
}

void apply_spline(
  TrajectoryPoints & traj_points, const double interpolation_resolution_m,
  const double max_yaw_discrepancy_deg, const double max_distance_discrepancy_m,
  const bool copy_original_orientation)
{
  constexpr size_t min_points_for_akima_spline = 5;
  constexpr double min_interpolation_resolution_m = 0.1;
  const auto traj_length = autoware::motion_utils::calcArcLength(traj_points);

  if (
    interpolation_resolution_m < min_interpolation_resolution_m ||
    traj_points.size() < min_points_for_akima_spline || traj_length < interpolation_resolution_m) {
    return;
  }

  constexpr bool use_lerp_for_z = false;
  constexpr bool use_zero_order_hold_for_twist = true;
  constexpr bool resample_input_trajectory_stop_point = false;
  constexpr bool dont_use_akima_spline_for_xy =
    true;  // Note: autoware::motion_utils::resampleTrajectory has an error where the use akima
           // spline input is inverted, so setting the use_akima_spline_for_xy to true actually
           // applies a simple lerp
  autoware_planning_msgs::msg::Trajectory temp_traj;
  temp_traj.points = traj_points;
  // first resample to a lower resolution to avoid ill-conditioned spline
  temp_traj = autoware::motion_utils::resampleTrajectory(
    temp_traj, 2.0 * interpolation_resolution_m, dont_use_akima_spline_for_xy, use_lerp_for_z,
    use_zero_order_hold_for_twist, resample_input_trajectory_stop_point);
  // then resample to the desired resolution using akima spline
  temp_traj = autoware::motion_utils::resampleTrajectory(
    temp_traj, interpolation_resolution_m, !dont_use_akima_spline_for_xy, use_lerp_for_z,
    use_zero_order_hold_for_twist, resample_input_trajectory_stop_point);

  // check where the original trajectory ends in the new trajectory or where there is a significant
  // change in yaw
  const double max_yaw_discrepancy_rad = autoware_utils_math::deg2rad(max_yaw_discrepancy_deg);
  const auto last_original_point = traj_points.back();
  const auto nearest_index_opt = autoware::motion_utils::findNearestIndex(
    temp_traj.points, last_original_point.pose, max_distance_discrepancy_m,
    max_yaw_discrepancy_rad);
  if (!nearest_index_opt.has_value() || nearest_index_opt.value() == 0) {
    log_warn_throttle("Could not find a suitable point to crop the trajectory");
    return;
  }
  // crop the trajectory up to the nearest index
  temp_traj.points = TrajectoryPoints(
    temp_traj.points.begin(), std::next(temp_traj.points.begin(), nearest_index_opt.value()));
  // ensure the last point is the same as the original trajectory last point
  temp_traj.points.push_back(last_original_point);
  // re-sample again using lerp to ensure the resolution is maintained after cropping
  temp_traj = autoware::motion_utils::resampleTrajectory(
    temp_traj, interpolation_resolution_m, dont_use_akima_spline_for_xy, use_lerp_for_z,
    use_zero_order_hold_for_twist, resample_input_trajectory_stop_point);
  if (copy_original_orientation) {
    // Copy orientation from original trajectory
    for (auto & out_point : temp_traj.points) {
      const auto nearest_index_yaw_opt = autoware::motion_utils::findNearestIndex(
        traj_points, out_point.pose, max_distance_discrepancy_m, M_PI_2);
      if (!nearest_index_yaw_opt.has_value()) {
        continue;
      }
      const auto nearest_index_yaw = nearest_index_yaw_opt.value();
      out_point.pose.orientation = traj_points.at(nearest_index_yaw).pose.orientation;
    }
  }
  traj_points = temp_traj.points;
}

void add_ego_state_to_trajectory(
  TrajectoryPoints & traj_points, const Odometry & current_odometry,
  const double nearest_dist_threshold_m, const double nearest_yaw_threshold_rad,
  const double backward_trajectory_extension_m)
{
  TrajectoryPoint ego_state;
  ego_state.pose = current_odometry.pose.pose;
  ego_state.longitudinal_velocity_mps = static_cast<float>(current_odometry.twist.twist.linear.x);

  if (traj_points.empty()) {
    traj_points.push_back(ego_state);
    return;
  }
  const auto & last_point = traj_points.back();
  const auto yaw_diff = std::abs(
    autoware_utils_math::normalize_degree(
      ego_state.pose.orientation.z - last_point.pose.orientation.z));
  const auto distance = autoware_utils::calc_distance2d(last_point, ego_state);
  constexpr double epsilon{1e-2};
  const bool is_change_small = distance < epsilon && yaw_diff < epsilon;
  if (is_change_small) {
    return;
  }

  const bool is_change_large =
    distance > nearest_dist_threshold_m || yaw_diff > nearest_yaw_threshold_rad;
  if (is_change_large) {
    traj_points = {ego_state};
    return;
  }

  traj_points.push_back(ego_state);

  size_t clip_idx = 0;
  double accumulated_length = 0.0;
  for (size_t i = traj_points.size() - 1; i > 0; i--) {
    accumulated_length += autoware_utils::calc_distance2d(traj_points.at(i - 1), traj_points.at(i));
    if (accumulated_length > backward_trajectory_extension_m) {
      clip_idx = i;
      break;
    }
  }
  traj_points.erase(traj_points.begin(), traj_points.begin() + static_cast<int>(clip_idx));
}

void expand_trajectory_with_ego_history(
  TrajectoryPoints & traj_points, const TrajectoryPoints & ego_history_points,
  const Odometry & current_odometry)
{
  if (ego_history_points.empty()) {
    return;
  }

  if (traj_points.empty()) {
    traj_points.insert(traj_points.begin(), ego_history_points.begin(), ego_history_points.end());
    return;
  }

  const auto first_ego_history_point = ego_history_points.front();
  const auto first_ego_trajectory_point = traj_points.front();
  const auto first_ego_trajectory_point_arc_length = autoware::motion_utils::calcSignedArcLength(
    ego_history_points, first_ego_history_point.pose.position,
    first_ego_trajectory_point.pose.position);

  const auto ego_position = current_odometry.pose.pose.position;
  const auto distance_ego_to_first_trajectory_point =
    autoware_utils::calc_distance2d(first_ego_trajectory_point, ego_position);

  std::for_each(ego_history_points.rbegin(), ego_history_points.rend(), [&](const auto & point) {
    const auto point_arc_length = autoware::motion_utils::calcSignedArcLength(
      ego_history_points, first_ego_history_point.pose.position, point.pose.position);

    const bool is_ahead_of_first_point = point_arc_length > first_ego_trajectory_point_arc_length;
    const bool is_closer_than_first_trajectory_point =
      autoware_utils::calc_distance2d(point, ego_position) < distance_ego_to_first_trajectory_point;
    const bool is_point_already_in_trajectory =
      std::any_of(traj_points.begin(), traj_points.end(), [&](const TrajectoryPoint & traj_point) {
        return autoware_utils::calc_distance2d(traj_point, point) < 1e-1;
      });
    if (
      is_ahead_of_first_point || is_point_already_in_trajectory ||
      is_closer_than_first_trajectory_point) {
      return;
    }

    traj_points.insert(traj_points.begin(), point);
    traj_points.front().longitudinal_velocity_mps =
      first_ego_trajectory_point.longitudinal_velocity_mps;
  });
}

}  // namespace autoware::trajectory_optimizer::utils
