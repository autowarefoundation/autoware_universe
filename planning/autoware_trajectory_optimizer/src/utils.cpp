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

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/logging.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
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

void debug_log_trajectory_tail_orientations(
  const TrajectoryPoints & traj, const std::string & stage_name, size_t num_tail_points)
{
  auto logger = rclcpp::get_logger("trajectory_pipeline_debug");

  if (traj.empty()) {
    RCLCPP_INFO(logger, "[%s] Empty trajectory", stage_name.c_str());
    return;
  }

  const size_t traj_size = traj.size();
  const size_t start_idx = (traj_size > num_tail_points) ? (traj_size - num_tail_points) : 0;

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);
  oss << "[" << stage_name << "] traj_size=" << traj_size << ", tail orientations:\n";

  for (size_t i = start_idx; i < traj_size; ++i) {
    const auto & pt = traj[i];
    const double yaw = tf2::getYaw(pt.pose.orientation);

    oss << "  idx=" << i << ": yaw=" << yaw << " rad (" << (yaw * 180.0 / M_PI) << " deg)";
    oss << ", vel=" << pt.longitudinal_velocity_mps << " m/s";

    // Compute delta_theta to next point if not the last point
    if (i + 1 < traj_size) {
      const auto & next_pt = traj[i + 1];
      const double next_yaw = tf2::getYaw(next_pt.pose.orientation);
      double delta_theta = next_yaw - yaw;

      // Normalize to [-pi, pi]
      while (delta_theta > M_PI) delta_theta -= 2.0 * M_PI;
      while (delta_theta < -M_PI) delta_theta += 2.0 * M_PI;

      oss << ", delta_theta_to_next=" << delta_theta << " rad (" << (delta_theta * 180.0 / M_PI)
          << " deg)";
    }
    oss << "\n";
  }

  RCLCPP_INFO(logger, "%s", oss.str().c_str());
}

}  // namespace autoware::trajectory_optimizer::utils
