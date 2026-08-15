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

#include "velocity_smoother.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_math/unit_conversion.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

VelocitySmoother::VelocitySmoother(
  const VelocitySmootherParams & params, const rclcpp::Logger & logger,
  rclcpp::Clock::SharedPtr clock, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  std::shared_ptr<autoware::velocity_smoother::JerkFilteredSmoother> jerk_filtered_smoother)
: params_(params),
  logger_(logger),
  clock_(std::move(clock)),
  jerk_filtered_smoother_(std::move(jerk_filtered_smoother))
{
  if (jerk_filtered_smoother_) {
    jerk_filtered_smoother_->setWheelBase(vehicle_info.wheel_base_m);
  }
}

void VelocitySmoother::optimize(
  TrajectoryPoints & traj_points, const nav_msgs::msg::Odometry & current_odometry,
  double current_acceleration)
{
  const double current_speed = current_odometry.twist.twist.linear.x;
  const double & target_pull_out_speed_mps = params_.target_pull_out_speed_mps;
  const double & target_pull_out_acc_mps2 = params_.target_pull_out_acc_mps2;
  const double & max_speed_mps = params_.max_speed_mps;

  // 1. Limit lateral acceleration (disabled by default)
  if (params_.limit_lateral_acceleration) {
    limit_lateral_acceleration(traj_points, params_.max_lateral_accel_mps2, current_odometry);
  }

  // 2. Calculate initial motion
  auto initial_motion_speed = current_speed;
  auto initial_motion_acc = current_acceleration;

  if (current_speed < target_pull_out_speed_mps) {
    // Vehicle is slow (starting/pull-out): use pull-out parameters
    initial_motion_speed = target_pull_out_speed_mps;
    initial_motion_acc = target_pull_out_acc_mps2;
  } else if (!prev_output_.empty()) {
    // Vehicle is moving: use prev_output for temporal consistency when tracking is good
    const auto projected =
      calc_projected_trajectory_point_from_ego(prev_output_, current_odometry.pose.pose);
    if (projected) {
      const double desired_vel = std::fabs(projected->longitudinal_velocity_mps);
      const double desired_acc = projected->acceleration_mps2;
      constexpr double replan_vel_deviation = 3.0;  // [m/s]
      if (std::fabs(current_speed - desired_vel) <= replan_vel_deviation) {
        initial_motion_speed = desired_vel;
        initial_motion_acc = desired_acc;
      }
    }
  }

  // 3. Engage speed: clamp minimum velocity during pull-out
  if (params_.set_engage_speed && (current_speed < target_pull_out_speed_mps)) {
    // Check distance to stop point to avoid engaging when obstacle is ahead
    const auto closest_idx =
      autoware::motion_utils::findNearestIndex(traj_points, current_odometry.pose.pose.position);
    const auto zero_vel_idx =
      autoware::motion_utils::searchZeroVelocityIndex(traj_points, closest_idx, traj_points.size());

    bool should_engage = true;
    if (zero_vel_idx) {
      const double stop_dist =
        autoware::motion_utils::calcSignedArcLength(traj_points, closest_idx, *zero_vel_idx);
      if (stop_dist <= params_.stop_dist_to_prohibit_engage) {
        should_engage = false;
      }
    }

    if (should_engage) {
      clamp_velocities(
        traj_points, static_cast<float>(initial_motion_speed),
        static_cast<float>(initial_motion_acc));
    }
  }

  // 4. Set max velocity
  if (params_.limit_speed) {
    set_max_velocity(traj_points, static_cast<float>(max_speed_mps));
  }

  // 5. Filter velocity using JerkFilteredSmoother
  if (params_.smooth_velocities) {
    filter_velocity(traj_points, initial_motion_speed, initial_motion_acc, current_odometry);

    // Save output for temporal consistency in next cycle
    if (!traj_points.empty()) {
      prev_output_ = traj_points;
    }
  }
}

std::optional<TrajectoryPoint> VelocitySmoother::calc_projected_trajectory_point_from_ego(
  const TrajectoryPoints & trajectory, const geometry_msgs::msg::Pose & ego_pose) const
{
  if (trajectory.size() < 2) {
    return std::nullopt;
  }

  const auto seg_idx = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    trajectory, ego_pose, params_.nearest_dist_threshold_m,
    autoware_utils_math::deg2rad(params_.nearest_yaw_threshold_deg));

  const auto & p0 = trajectory.at(seg_idx);
  const auto & p1 = trajectory.at(seg_idx + 1);

  const double dx = p1.pose.position.x - p0.pose.position.x;
  const double dy = p1.pose.position.y - p0.pose.position.y;
  const double seg_len_sq = dx * dx + dy * dy;

  if (seg_len_sq < 1e-10) {
    return p0;
  }

  const double ex = ego_pose.position.x - p0.pose.position.x;
  const double ey = ego_pose.position.y - p0.pose.position.y;
  const double ratio = std::clamp((ex * dx + ey * dy) / seg_len_sq, 0.0, 1.0);

  TrajectoryPoint result = p0;
  result.longitudinal_velocity_mps = static_cast<float>(
    p0.longitudinal_velocity_mps +
    ratio * (p1.longitudinal_velocity_mps - p0.longitudinal_velocity_mps));
  result.acceleration_mps2 = static_cast<float>(
    p0.acceleration_mps2 + ratio * (p1.acceleration_mps2 - p0.acceleration_mps2));

  return result;
}

void VelocitySmoother::clamp_velocities(
  TrajectoryPoints & traj_points, const float min_velocity, const float min_acceleration)
{
  std::for_each(
    traj_points.begin(), traj_points.end(),
    [min_velocity, min_acceleration](TrajectoryPoint & point) {
      point.longitudinal_velocity_mps = std::max(point.longitudinal_velocity_mps, min_velocity);
      point.acceleration_mps2 = std::max(point.acceleration_mps2, min_acceleration);
    });
}

void VelocitySmoother::set_max_velocity(TrajectoryPoints & traj_points, const float max_velocity)
{
  if (traj_points.empty()) {
    return;
  }

  if (traj_points.size() == 1) {
    traj_points[0].longitudinal_velocity_mps =
      std::min(traj_points[0].longitudinal_velocity_mps, max_velocity);
    traj_points[0].acceleration_mps2 = 0.0f;
    return;
  }

  std::vector<std::pair<size_t, size_t>> modified_segment_indices;
  std::vector<double> original_dts;
  original_dts.reserve(traj_points.size() - 1);

  auto compute_dt_using_velocity_and_acc =
    [](const TrajectoryPoint & from, const TrajectoryPoint & to) -> double {
    const auto dv = static_cast<double>(to.longitudinal_velocity_mps) -
                    static_cast<double>(from.longitudinal_velocity_mps);
    const auto acc = static_cast<double>(from.acceleration_mps2);
    constexpr double epsilon_acceleration = 1e-6;
    const auto denominator_acc = std::max(std::abs(acc), epsilon_acceleration);
    return std::abs(dv) / denominator_acc;
  };

  for (size_t i = 0; i < traj_points.size() - 1; ++i) {
    original_dts.push_back(compute_dt_using_velocity_and_acc(traj_points[i], traj_points[i + 1]));
  }

  size_t segment_start = 0;
  bool in_segment = false;

  for (size_t i = 0; i < traj_points.size(); ++i) {
    const bool exceeds_max = traj_points[i].longitudinal_velocity_mps > max_velocity;
    if (exceeds_max && !in_segment) {
      segment_start = i;
      in_segment = true;
    } else if (!exceeds_max && in_segment) {
      modified_segment_indices.emplace_back(segment_start, i - 1);
      in_segment = false;
    }
  }
  if (in_segment) {
    modified_segment_indices.emplace_back(segment_start, traj_points.size() - 1);
  }

  for (const auto & [start, end] : modified_segment_indices) {
    for (size_t i = start; i <= end; ++i) {
      traj_points[i].longitudinal_velocity_mps = max_velocity;
    }
    for (size_t i = start; i < end; ++i) {
      traj_points[i].acceleration_mps2 = 0.0f;
    }
  }

  for (const auto & [start, end] : modified_segment_indices) {
    if (start > 0) {
      const size_t idx_before = start - 1;
      const double dt = original_dts[idx_before];
      const auto v_before = static_cast<double>(traj_points[idx_before].longitudinal_velocity_mps);
      const auto v_start = static_cast<double>(traj_points[start].longitudinal_velocity_mps);
      const double new_acc = (v_start - v_before) / dt;
      traj_points[idx_before].acceleration_mps2 = static_cast<float>(new_acc);
    }
    if (end < traj_points.size() - 1) {
      const double dt = original_dts[end];
      const auto v_end = static_cast<double>(traj_points[end].longitudinal_velocity_mps);
      const auto v_after = static_cast<double>(traj_points[end + 1].longitudinal_velocity_mps);
      const double new_acc = (v_after - v_end) / dt;
      traj_points[end].acceleration_mps2 = static_cast<float>(new_acc);
    }
  }

  traj_points.back().acceleration_mps2 = 0.0f;
}

void VelocitySmoother::limit_lateral_acceleration(
  TrajectoryPoints & traj_points, const double max_lateral_accel_mps2,
  const nav_msgs::msg::Odometry & current_odometry)
{
  if (traj_points.empty()) {
    return;
  }

  auto get_delta_time = [](const auto & next, const auto & current) -> double {
    return next->time_from_start.sec + next->time_from_start.nanosec * 1e-9 -
           (current->time_from_start.sec + current->time_from_start.nanosec * 1e-9);
  };

  const auto & current_position = current_odometry.pose.pose.position;
  motion_utils::calculate_time_from_start(traj_points, current_position);

  const auto closest_index = motion_utils::findNearestIndex(traj_points, current_position);
  const auto start_itr =
    std::next(traj_points.begin(), static_cast<TrajectoryPoints::difference_type>(closest_index));

  for (auto itr = start_itr; itr < std::prev(traj_points.end()); ++itr) {
    const auto current_pose = itr->pose;
    const auto next_pose = std::next(itr)->pose;
    const auto delta_time = get_delta_time(std::next(itr), itr);

    tf2::Quaternion q_current;
    tf2::Quaternion q_next;
    tf2::convert(current_pose.orientation, q_current);
    tf2::convert(next_pose.orientation, q_next);
    double delta_theta = q_current.angleShortestPath(q_next);
    if (delta_theta > M_PI) {
      delta_theta -= 2.0 * M_PI;
    } else if (delta_theta < -M_PI) {
      delta_theta += 2.0 * M_PI;
    }

    constexpr double epsilon_yaw_rate = 1.0e-5;
    const double yaw_rate = std::max(std::abs(delta_theta / delta_time), epsilon_yaw_rate);
    const double current_speed = std::abs(itr->longitudinal_velocity_mps);
    const double lateral_acceleration = current_speed * yaw_rate;
    if (lateral_acceleration < max_lateral_accel_mps2) continue;

    itr->longitudinal_velocity_mps = static_cast<float>(max_lateral_accel_mps2 / yaw_rate);
  }

  motion_utils::calculate_time_from_start(traj_points, current_odometry.pose.pose.position);

  // Recalculate longitudinal acceleration after velocity change
  if (traj_points.size() >= 2) {
    for (size_t i = 0; i + 1 < traj_points.size(); ++i) {
      constexpr double min_dt = 1e-9;
      const double curr_time = static_cast<double>(traj_points[i].time_from_start.sec) +
                               static_cast<double>(traj_points[i].time_from_start.nanosec) * 1e-9;
      const double next_time =
        static_cast<double>(traj_points[i + 1].time_from_start.sec) +
        static_cast<double>(traj_points[i + 1].time_from_start.nanosec) * 1e-9;
      const double dt = std::max(next_time - curr_time, min_dt);
      const double dv = static_cast<double>(traj_points[i + 1].longitudinal_velocity_mps) -
                        static_cast<double>(traj_points[i].longitudinal_velocity_mps);
      traj_points[i].acceleration_mps2 = static_cast<float>(dv / dt);
    }
    traj_points.back().acceleration_mps2 = 0.0f;
  }
}

void VelocitySmoother::filter_velocity(
  TrajectoryPoints & traj_points, const double initial_speed, const double initial_acc,
  const nav_msgs::msg::Odometry & current_odometry)
{
  if (!jerk_filtered_smoother_) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 5000, "JerkFilteredSmoother is not initialized");
    return;
  }

  if (traj_points.size() < 2) {
    return;
  }

  const double nearest_yaw_threshold_rad =
    autoware_utils_math::deg2rad(params_.nearest_yaw_threshold_deg);

  constexpr bool enable_smooth_limit = true;
  constexpr bool use_resampling = true;

  // Lateral acceleration filter
  traj_points = jerk_filtered_smoother_->applyLateralAccelerationFilter(
    traj_points, initial_speed, initial_acc, enable_smooth_limit, use_resampling);

  // Steering rate limit (use_resample = false since already resampled above)
  traj_points = jerk_filtered_smoother_->applySteeringRateLimit(traj_points, false);

  // Resample trajectory with initial_motion_speed based interval distance
  traj_points = jerk_filtered_smoother_->resampleTrajectory(
    traj_points, initial_speed, current_odometry.pose.pose, params_.nearest_dist_threshold_m,
    nearest_yaw_threshold_rad);

  // Clip trajectory from closest point
  const size_t traj_closest = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    traj_points, current_odometry.pose.pose, params_.nearest_dist_threshold_m,
    nearest_yaw_threshold_rad);

  TrajectoryPoints clipped(
    traj_points.begin() + static_cast<TrajectoryPoints::difference_type>(traj_closest),
    traj_points.end());
  traj_points = clipped;

  // Set terminal velocity to 0 before QP
  if (!traj_points.empty()) {
    traj_points.back().longitudinal_velocity_mps = 0.0;
  }

  // Save stop point pose before QP (velocities will be modified by QP)
  const auto stop_idx = autoware::motion_utils::searchZeroVelocityIndex(traj_points);
  bool has_stop_pose = false;
  geometry_msgs::msg::Pose stop_pose;
  if (stop_idx) {
    stop_pose = traj_points.at(*stop_idx).pose;
    has_stop_pose = true;
  }

  // Apply JerkFilteredSmoother (forward/backward jerk filtering + QP)
  std::vector<TrajectoryPoints> debug_trajectories;
  if (!jerk_filtered_smoother_->apply(
        initial_speed, initial_acc, traj_points, traj_points, debug_trajectories, false)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "Fail to solve optimization.");
  }

  // Overwrite stop point - set 0 velocity from stop point onward
  if (has_stop_pose) {
    const auto nearest_output_idx = autoware::motion_utils::findNearestIndex(
      traj_points, stop_pose, params_.nearest_dist_threshold_m, nearest_yaw_threshold_rad);
    if (nearest_output_idx) {
      for (size_t i = *nearest_output_idx; i < traj_points.size(); ++i) {
        traj_points[i].longitudinal_velocity_mps = 0.0;
      }
    }
  }

  // Ensure terminal velocity is 0 after QP
  if (!traj_points.empty()) {
    traj_points.back().longitudinal_velocity_mps = 0.0;
  }
}

}  // namespace autoware::minimum_rule_based_planner
