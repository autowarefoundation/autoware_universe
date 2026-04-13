// Copyright 2018-2021 The Autoware Foundation
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

#include "autoware/mpc_lateral_controller/mpc_utils.hpp"

#include "autoware/interpolation/linear_interpolation.hpp"
#include "autoware/interpolation/spline_interpolation.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/math/normalization.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{
namespace
{
double calcLongitudinalOffset(
  const geometry_msgs::msg::Point & p_front, const geometry_msgs::msg::Point & p_back,
  const geometry_msgs::msg::Point & p_target)
{
  const Eigen::Vector3d segment_vec{p_back.x - p_front.x, p_back.y - p_front.y, 0};
  const Eigen::Vector3d target_vec{p_target.x - p_front.x, p_target.y - p_front.y, 0};

  return segment_vec.dot(target_vec) / segment_vec.norm();
}

bool isTemporalShortSegment(
  const double ds, const double dt, const double vx, const bool use_short_segment_protection)
{
  if (!use_short_segment_protection) {
    return ds < 1.0e-3;
  }

  constexpr double min_distance_threshold = 2.0e-2;
  constexpr double stop_like_velocity_threshold = 1.0e-1;
  constexpr double min_velocity_floor = 1.0e-1;
  constexpr double expected_distance_ratio = 0.5;
  constexpr double min_time_step = 1.0e-3;

  if (ds < min_distance_threshold && std::fabs(vx) < stop_like_velocity_threshold) {
    return true;
  }

  const double bounded_dt = std::max(dt, min_time_step);
  const double expected_distance = std::max(std::fabs(vx), min_velocity_floor) * bounded_dt;
  return ds < expected_distance_ratio * expected_distance;
}
}  // namespace

namespace MPCUtils
{
using autoware_utils::calc_distance2d;
using autoware_utils::create_quaternion_from_yaw;
using autoware_utils::normalize_radian;

double calcDistance2d(const MPCTrajectory & trajectory, const size_t idx1, const size_t idx2)
{
  const double dx = trajectory.x.at(idx1) - trajectory.x.at(idx2);
  const double dy = trajectory.y.at(idx1) - trajectory.y.at(idx2);
  return std::hypot(dx, dy);
}

double calcDistance3d(const MPCTrajectory & trajectory, const size_t idx1, const size_t idx2)
{
  const double dx = trajectory.x.at(idx1) - trajectory.x.at(idx2);
  const double dy = trajectory.y.at(idx1) - trajectory.y.at(idx2);
  const double dz = trajectory.z.at(idx1) - trajectory.z.at(idx2);
  return std::hypot(dx, dy, dz);
}

void convertEulerAngleToMonotonic(std::vector<double> & angle_vector)
{
  for (uint i = 1; i < angle_vector.size(); ++i) {
    const double da = angle_vector.at(i) - angle_vector.at(i - 1);
    angle_vector.at(i) = angle_vector.at(i - 1) + normalize_radian(da);
  }
}

double calcLateralError(const Pose & ego_pose, const Pose & ref_pose)
{
  const double err_x = ego_pose.position.x - ref_pose.position.x;
  const double err_y = ego_pose.position.y - ref_pose.position.y;
  const double ref_yaw = tf2::getYaw(ref_pose.orientation);
  const double lat_err = -std::sin(ref_yaw) * err_x + std::cos(ref_yaw) * err_y;
  return lat_err;
}

void calcMPCTrajectoryArcLength(const MPCTrajectory & trajectory, std::vector<double> & arc_length)
{
  double dist = 0.0;
  arc_length.clear();
  arc_length.push_back(dist);
  for (uint i = 1; i < trajectory.size(); ++i) {
    dist += calcDistance2d(trajectory, i, i - 1);
    arc_length.push_back(dist);
  }
}

double calcMPCTrajectoryArcLength(const MPCTrajectory & trajectory)
{
  double length = 0.0;
  for (size_t i = 1; i < trajectory.size(); ++i) {
    length += calcDistance2d(trajectory, i, i - 1);
  }
  return length;
}

std::pair<bool, MPCTrajectory> resampleMPCTrajectoryByDistance(
  const MPCTrajectory & input, const double resample_interval_dist, const size_t nearest_seg_idx,
  const double ego_offset_to_segment)
{
  MPCTrajectory output;

  if (input.empty()) {
    return {true, output};
  }
  std::vector<double> input_arclength;
  calcMPCTrajectoryArcLength(input, input_arclength);

  if (input_arclength.empty()) {
    return {false, output};
  }

  std::vector<double> output_arclength;
  // To accurately sample the ego point, resample separately in the forward direction and the
  // backward direction from the current position.
  for (double s = std::clamp(
         input_arclength.at(nearest_seg_idx) + ego_offset_to_segment, 0.0,
         input_arclength.back() - 1e-6);
       0 <= s; s -= resample_interval_dist) {
    output_arclength.push_back(s);
  }
  std::reverse(output_arclength.begin(), output_arclength.end());
  for (double s = std::max(input_arclength.at(nearest_seg_idx) + ego_offset_to_segment, 0.0) +
                  resample_interval_dist;
       s < input_arclength.back(); s += resample_interval_dist) {
    output_arclength.push_back(s);
  }

  std::vector<double> input_yaw = input.yaw;
  convertEulerAngleToMonotonic(input_yaw);

  const auto lerp_arc_length = [&](const auto & input_value) {
    return autoware::interpolation::lerp(input_arclength, input_value, output_arclength);
  };
  const auto spline_arc_length = [&](const auto & input_value) {
    return autoware::interpolation::spline(input_arclength, input_value, output_arclength);
  };

  output.x = spline_arc_length(input.x);
  output.y = spline_arc_length(input.y);
  output.z = spline_arc_length(input.z);
  output.yaw = spline_arc_length(input_yaw);
  output.vx = lerp_arc_length(input.vx);  // must be linear
  output.k = spline_arc_length(input.k);
  output.smooth_k = spline_arc_length(input.smooth_k);
  output.relative_time = lerp_arc_length(input.relative_time);  // must be linear

  return {true, output};
}

bool linearInterpMPCTrajectory(
  const std::vector<double> & in_index, const MPCTrajectory & in_traj,
  const std::vector<double> & out_index, MPCTrajectory & out_traj)
{
  if (in_traj.empty()) {
    out_traj = in_traj;
    return true;
  }

  std::vector<double> in_traj_yaw = in_traj.yaw;
  convertEulerAngleToMonotonic(in_traj_yaw);

  const auto lerp_arc_length = [&](const auto & input_value) {
    return autoware::interpolation::lerp(in_index, input_value, out_index);
  };

  try {
    out_traj.x = lerp_arc_length(in_traj.x);
    out_traj.y = lerp_arc_length(in_traj.y);
    out_traj.z = lerp_arc_length(in_traj.z);
    out_traj.yaw = lerp_arc_length(in_traj_yaw);
    out_traj.vx = lerp_arc_length(in_traj.vx);
    out_traj.k = lerp_arc_length(in_traj.k);
    out_traj.smooth_k = lerp_arc_length(in_traj.smooth_k);
    out_traj.relative_time = lerp_arc_length(in_traj.relative_time);
  } catch (const std::exception & e) {
    std::cerr << "linearInterpMPCTrajectory error!: " << e.what() << std::endl;
  }

  if (out_traj.empty()) {
    std::cerr << "[mpc util] linear interpolation error" << std::endl;
    return false;
  }

  return true;
}

void calcTrajectoryYawFromXY(
  MPCTrajectory & traj, const bool is_forward_shift, const bool use_input_yaw_for_short_segment)
{
  if (traj.yaw.size() < 3) {  // at least 3 points are required to calculate yaw
    return;
  }
  if (traj.yaw.size() != traj.vx.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("mpc_utils"), "trajectory size has no consistency.");
    return;
  }

  const auto input_yaw = traj.yaw;

  // interpolate yaw
  for (int i = 1; i < static_cast<int>(traj.yaw.size()) - 1; ++i) {
    const double dx = traj.x.at(i + 1) - traj.x.at(i - 1);
    const double dy = traj.y.at(i + 1) - traj.y.at(i - 1);
    const auto curr_idx = static_cast<size_t>(i);
    const double prev_dist = calcDistance2d(traj, curr_idx, curr_idx - 1);
    const double next_dist = calcDistance2d(traj, curr_idx + 1, curr_idx);
    const double prev_dt = traj.relative_time.at(curr_idx) - traj.relative_time.at(curr_idx - 1);
    const double next_dt = traj.relative_time.at(curr_idx + 1) - traj.relative_time.at(curr_idx);
    const double prev_vx = 0.5 * (traj.vx.at(curr_idx - 1) + traj.vx.at(curr_idx));
    const double next_vx = 0.5 * (traj.vx.at(curr_idx) + traj.vx.at(curr_idx + 1));
    if (
      std::hypot(dx, dy) < 1.0e-3 ||
      isTemporalShortSegment(prev_dist, prev_dt, prev_vx, use_input_yaw_for_short_segment) ||
      isTemporalShortSegment(next_dist, next_dt, next_vx, use_input_yaw_for_short_segment)) {
      traj.yaw.at(i) = use_input_yaw_for_short_segment ? input_yaw.at(i) : traj.yaw.at(i - 1);
      continue;
    }
    traj.yaw.at(i) = is_forward_shift ? std::atan2(dy, dx) : std::atan2(dy, dx) + M_PI;
  }
  if (traj.yaw.size() > 1) {
    const double dx0 = traj.x.at(1) - traj.x.at(0);
    const double dy0 = traj.y.at(1) - traj.y.at(0);
    const double ds0 = calcDistance2d(traj, 1, 0);
    const double dt0 = traj.relative_time.at(1) - traj.relative_time.at(0);
    const double vx0 = 0.5 * (traj.vx.at(0) + traj.vx.at(1));
    if (
      std::hypot(dx0, dy0) >= 1.0e-3 &&
      !isTemporalShortSegment(ds0, dt0, vx0, use_input_yaw_for_short_segment)) {
      traj.yaw.at(0) = is_forward_shift ? std::atan2(dy0, dx0) : std::atan2(dy0, dx0) + M_PI;
    } else {
      traj.yaw.at(0) = use_input_yaw_for_short_segment ? input_yaw.at(0) : traj.yaw.at(1);
    }

    const size_t last = traj.yaw.size() - 1;
    const double dxn = traj.x.at(last) - traj.x.at(last - 1);
    const double dyn = traj.y.at(last) - traj.y.at(last - 1);
    const double dsn = calcDistance2d(traj, last, last - 1);
    const double dtn = traj.relative_time.at(last) - traj.relative_time.at(last - 1);
    const double vxn = 0.5 * (traj.vx.at(last - 1) + traj.vx.at(last));
    if (
      std::hypot(dxn, dyn) >= 1.0e-3 &&
      !isTemporalShortSegment(dsn, dtn, vxn, use_input_yaw_for_short_segment)) {
      traj.yaw.back() = is_forward_shift ? std::atan2(dyn, dxn) : std::atan2(dyn, dxn) + M_PI;
    } else {
      traj.yaw.back() =
        use_input_yaw_for_short_segment ? input_yaw.at(last) : traj.yaw.at(last - 1);
    }
  }
}

void calcTrajectoryCurvature(
  const int curvature_smoothing_num_traj, const int curvature_smoothing_num_ref_steer,
  MPCTrajectory & traj, const bool use_short_segment_protection)
{
  traj.k =
    calcTrajectoryCurvature(curvature_smoothing_num_traj, traj, use_short_segment_protection);
  traj.smooth_k =
    calcTrajectoryCurvature(curvature_smoothing_num_ref_steer, traj, use_short_segment_protection);
}

void calcTrajectoryCurvatureBySpatialResample(
  const int curvature_smoothing_num_traj, const int curvature_smoothing_num_ref_steer,
  const double resample_interval_dist, MPCTrajectory & traj)
{
  if (traj.size() < 3) {
    return;
  }

  // 1. Compute arc length of original temporal trajectory
  std::vector<double> orig_arclength;
  calcMPCTrajectoryArcLength(traj, orig_arclength);
  const double total_length = orig_arclength.back();
  if (total_length < 1e-6) {
    return;
  }

  // 2. Remove spatially duplicate points (stopped vehicle).
  //    Spline requires strictly increasing arc length.
  constexpr double dedup_eps = 1e-3;
  std::vector<double> unique_arclength;
  std::vector<double> unique_x;
  std::vector<double> unique_y;
  unique_arclength.push_back(orig_arclength.front());
  unique_x.push_back(traj.x.front());
  unique_y.push_back(traj.y.front());
  for (size_t i = 1; i < orig_arclength.size(); ++i) {
    if (orig_arclength[i] - unique_arclength.back() > dedup_eps) {
      unique_arclength.push_back(orig_arclength[i]);
      unique_x.push_back(traj.x[i]);
      unique_y.push_back(traj.y[i]);
    }
  }
  if (unique_arclength.size() < 3) {
    return;
  }

  // 3. Generate equally-spaced arc length points
  std::vector<double> resampled_arclength;
  for (double s = 0.0; s < total_length; s += resample_interval_dist) {
    resampled_arclength.push_back(s);
  }
  if (resampled_arclength.back() < total_length - 1e-6) {
    resampled_arclength.push_back(total_length);
  }
  if (resampled_arclength.size() < 3) {
    return;
  }

  // 4. Spatially resample x, y using spline interpolation on deduplicated points
  MPCTrajectory spatial_traj;
  spatial_traj.x = autoware::interpolation::spline(unique_arclength, unique_x, resampled_arclength);
  spatial_traj.y = autoware::interpolation::spline(unique_arclength, unique_y, resampled_arclength);
  const auto n = resampled_arclength.size();
  spatial_traj.z.resize(n, 0.0);
  spatial_traj.yaw.resize(n, 0.0);
  spatial_traj.vx.resize(n, 0.0);
  spatial_traj.k.resize(n, 0.0);
  spatial_traj.smooth_k.resize(n, 0.0);
  spatial_traj.relative_time.resize(n, 0.0);

  // 5. Calculate curvature on the spatially uniform trajectory
  const auto k_spatial = calcTrajectoryCurvature(curvature_smoothing_num_traj, spatial_traj, false);
  const auto smooth_k_spatial =
    calcTrajectoryCurvature(curvature_smoothing_num_ref_steer, spatial_traj, false);

  // 6. Map curvature back to original temporal trajectory.
  //    Use lerp on unique (strictly increasing) arc lengths.
  //    Duplicate-arclength points (stopped) get curvature = 0.
  const auto k_at_unique =
    autoware::interpolation::lerp(resampled_arclength, k_spatial, unique_arclength);
  const auto smooth_k_at_unique =
    autoware::interpolation::lerp(resampled_arclength, smooth_k_spatial, unique_arclength);

  traj.k.assign(traj.size(), 0.0);
  traj.smooth_k.assign(traj.size(), 0.0);
  size_t unique_idx = 0;
  for (size_t i = 0; i < traj.size(); ++i) {
    // Find matching unique point
    while (unique_idx + 1 < unique_arclength.size() &&
           unique_arclength[unique_idx + 1] <= orig_arclength[i] + dedup_eps) {
      ++unique_idx;
    }
    // Only assign curvature if this point has a unique spatial position
    if (i == 0 || orig_arclength[i] - orig_arclength[i - 1] > dedup_eps) {
      traj.k[i] = k_at_unique[unique_idx];
      traj.smooth_k[i] = smooth_k_at_unique[unique_idx];
    }
    // else: remains 0.0 (stopped / duplicate point)
  }
}

std::vector<double> calcTrajectoryCurvature(
  const int curvature_smoothing_num, const MPCTrajectory & traj,
  const bool use_short_segment_protection)
{
  std::vector<double> curvature_vec(traj.x.size());
  if (traj.x.size() < 3) {
    return curvature_vec;
  }

  /* calculate curvature by circle fitting from three points */
  geometry_msgs::msg::Point p1, p2, p3;
  const int max_smoothing_num =
    static_cast<int>(std::floor(0.5 * (static_cast<double>(traj.x.size() - 1))));
  const size_t L = static_cast<size_t>(std::min(curvature_smoothing_num, max_smoothing_num));
  for (size_t i = L; i < traj.x.size() - L; ++i) {
    const size_t curr_idx = i;
    const size_t prev_idx = curr_idx - L;
    const size_t next_idx = curr_idx + L;
    const double dist_prev = calcDistance2d(traj, curr_idx, prev_idx);
    const double dist_next = calcDistance2d(traj, next_idx, curr_idx);
    const double dist_span = calcDistance2d(traj, next_idx, prev_idx);
    const double dt_prev = traj.relative_time.at(curr_idx) - traj.relative_time.at(prev_idx);
    const double dt_next = traj.relative_time.at(next_idx) - traj.relative_time.at(curr_idx);
    const double dt_span = traj.relative_time.at(next_idx) - traj.relative_time.at(prev_idx);
    const double vx_prev = 0.5 * (traj.vx.at(prev_idx) + traj.vx.at(curr_idx));
    const double vx_next = 0.5 * (traj.vx.at(curr_idx) + traj.vx.at(next_idx));
    const double vx_span = 0.5 * (traj.vx.at(prev_idx) + traj.vx.at(next_idx));
    if (
      isTemporalShortSegment(dist_prev, dt_prev, vx_prev, use_short_segment_protection) ||
      isTemporalShortSegment(dist_next, dt_next, vx_next, use_short_segment_protection) ||
      isTemporalShortSegment(dist_span, dt_span, vx_span, use_short_segment_protection)) {
      curvature_vec.at(curr_idx) = curr_idx > 0 ? curvature_vec.at(curr_idx - 1) : 0.0;
      continue;
    }
    p1.x = traj.x.at(prev_idx);
    p2.x = traj.x.at(curr_idx);
    p3.x = traj.x.at(next_idx);
    p1.y = traj.y.at(prev_idx);
    p2.y = traj.y.at(curr_idx);
    p3.y = traj.y.at(next_idx);
    try {
      curvature_vec.at(curr_idx) = autoware_utils::calc_curvature(p1, p2, p3);
    } catch (...) {
      std::cerr << "[MPC] 2 points are too close to calculate curvature." << std::endl;
      curvature_vec.at(curr_idx) = curr_idx > 0 ? curvature_vec.at(curr_idx - 1) : 0.0;
    }
  }

  /* first and last curvature is copied from next value */
  for (size_t i = 0; i < std::min(L, traj.x.size()); ++i) {
    curvature_vec.at(i) = curvature_vec.at(std::min(L, traj.x.size() - 1));
    curvature_vec.at(traj.x.size() - i - 1) =
      curvature_vec.at(std::max(traj.x.size() - L - 1, size_t(0)));
  }
  return curvature_vec;
}

MPCTrajectory convertToMPCTrajectory(const Trajectory & input, const bool use_temporal_trajectory)
{
  MPCTrajectory output;
  for (const TrajectoryPoint & p : input.points) {
    const double x = p.pose.position.x;
    const double y = p.pose.position.y;
    const double z = p.pose.position.z;
    const double yaw = tf2::getYaw(p.pose.orientation);
    const double vx = p.longitudinal_velocity_mps;
    const double k = 0.0;

    // Time handling: temporal (use timestamps) vs spatial (calculate from distance/velocity)
    const double t = use_temporal_trajectory
                       ? rclcpp::Duration(p.time_from_start).seconds()
                       : 0.0;  // Will be recalculated by calcMPCTrajectoryTime()
    output.push_back(x, y, z, yaw, vx, k, k, t);
  }

  if (!use_temporal_trajectory) {
    calcMPCTrajectoryTime(output);
  }

  return output;
}

Trajectory convertToAutowareTrajectory(const MPCTrajectory & input, const double wheelbase)
{
  Trajectory output;
  TrajectoryPoint p;
  for (size_t i = 0; i < input.size(); ++i) {
    p.pose.position.x = input.x.at(i);
    p.pose.position.y = input.y.at(i);
    p.pose.position.z = input.z.at(i);
    p.pose.orientation = autoware_utils::create_quaternion_from_yaw(input.yaw.at(i));
    p.longitudinal_velocity_mps =
      static_cast<decltype(p.longitudinal_velocity_mps)>(input.vx.at(i));
    p.time_from_start =
      rclcpp::Duration::from_seconds(input.relative_time.at(i) - input.relative_time.front());
    if (wheelbase != 0.0) {
      p.front_wheel_angle_rad = static_cast<float>(std::atan(input.smooth_k.at(i) * wheelbase));
    }
    output.points.push_back(p);
    if (output.points.size() == output.points.max_size()) {
      break;
    }
  }
  return output;
}

bool calcMPCTrajectoryTime(MPCTrajectory & traj)
{
  constexpr auto min_dt = 1.0e-4;  // must be positive value to avoid duplication in time
  double t = 0.0;
  traj.relative_time.clear();
  traj.relative_time.push_back(t);
  for (size_t i = 0; i < traj.x.size() - 1; ++i) {
    const double dist = calcDistance3d(traj, i, i + 1);
    const double v = std::max(std::fabs(traj.vx.at(i)), 0.1);
    t += std::max(dist / v, min_dt);
    traj.relative_time.push_back(t);
  }
  return true;
}

void dynamicSmoothingVelocity(
  const size_t start_seg_idx, const double start_vel, const double acc_lim, const double tau,
  MPCTrajectory & traj, const bool use_temporal_trajectory)
{
  double curr_v = start_vel;
  // set current velocity in both start and end point of the segment
  traj.vx.at(start_seg_idx) = start_vel;
  if (1 < traj.vx.size()) {
    traj.vx.at(start_seg_idx + 1) = start_vel;
  }

  for (size_t i = start_seg_idx + 2; i < traj.size(); ++i) {
    const double ds = calcDistance2d(traj, i, i - 1);
    const double dt = [&]() {
      if (use_temporal_trajectory) {
        const double time_dt = traj.relative_time.at(i) - traj.relative_time.at(i - 1);
        constexpr double min_time_dt = 1.0e-4;
        return std::max(time_dt, min_time_dt);
      }
      return ds / std::max(std::fabs(curr_v), std::numeric_limits<double>::epsilon());
    }();
    const double a = tau / std::max(tau + dt, std::numeric_limits<double>::epsilon());
    const double updated_v = a * curr_v + (1.0 - a) * traj.vx.at(i);
    const double dv = std::max(-acc_lim * dt, std::min(acc_lim * dt, updated_v - curr_v));
    curr_v = curr_v + dv;
    traj.vx.at(i) = curr_v;
  }

  if (!use_temporal_trajectory) {
    // Spatial mode: recalculate time after velocity changes
    calcMPCTrajectoryTime(traj);
  }
  // Temporal mode keeps timestamps and updates velocity using time delta.
}

bool calcNearestPoseInterp(
  const MPCTrajectory & traj, const Pose & self_pose, Pose * nearest_pose, size_t * nearest_index,
  double * nearest_time, const double max_dist, const double max_yaw, const bool use_time_window,
  const double min_time_window_sec, const double max_time_window_sec)
{
  if (traj.empty() || !nearest_pose || !nearest_index || !nearest_time) {
    return false;
  }

  const auto autoware_traj = convertToAutowareTrajectory(traj);
  if (autoware_traj.points.empty()) {
    const auto logger = rclcpp::get_logger("mpc_util");
    auto clock = rclcpp::Clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(logger, clock, 5000, "[calcNearestPoseInterp] input trajectory is empty");
    return false;
  }

  *nearest_index = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    autoware_traj.points, self_pose, max_dist, max_yaw);
  if (use_time_window) {
    const double min_time_sec = std::min(min_time_window_sec, max_time_window_sec);
    const double max_time_sec = std::max(min_time_window_sec, max_time_window_sec);
    std::vector<size_t> candidates;
    candidates.reserve(traj.size());
    for (size_t i = 0; i < traj.size(); ++i) {
      const double t = traj.relative_time.at(i);
      if (min_time_sec <= t && t <= max_time_sec) {
        candidates.push_back(i);
      }
    }
    if (!candidates.empty()) {
      const double self_yaw = tf2::getYaw(self_pose.orientation);
      size_t best_relaxed_idx = candidates.front();
      double best_relaxed_dist2 = std::numeric_limits<double>::max();
      size_t best_strict_idx = candidates.front();
      double best_strict_dist2 = std::numeric_limits<double>::max();
      bool has_strict_candidate = false;
      for (const auto idx : candidates) {
        const double dx = traj.x.at(idx) - self_pose.position.x;
        const double dy = traj.y.at(idx) - self_pose.position.y;
        const double dist2 = dx * dx + dy * dy;
        if (dist2 < best_relaxed_dist2) {
          best_relaxed_dist2 = dist2;
          best_relaxed_idx = idx;
        }
        const double yaw_error = std::fabs(normalize_radian(self_yaw - traj.yaw.at(idx)));
        if (std::sqrt(dist2) <= max_dist && yaw_error <= max_yaw && dist2 < best_strict_dist2) {
          best_strict_dist2 = dist2;
          best_strict_idx = idx;
          has_strict_candidate = true;
        }
      }
      *nearest_index = has_strict_candidate ? best_strict_idx : best_relaxed_idx;
    }
  }
  const size_t traj_size = traj.size();

  if (traj.size() == 1) {
    nearest_pose->position.x = traj.x.at(*nearest_index);
    nearest_pose->position.y = traj.y.at(*nearest_index);
    nearest_pose->orientation = create_quaternion_from_yaw(traj.yaw.at(*nearest_index));
    *nearest_time = traj.relative_time.at(*nearest_index);
    return true;
  }

  /* get second nearest index = next to nearest_index */
  const auto [prev, next] = [&]() -> std::pair<size_t, size_t> {
    if (*nearest_index == 0) {
      return std::make_pair(0, 1);
    }
    if (*nearest_index == traj_size - 1) {
      return std::make_pair(traj_size - 2, traj_size - 1);
    }

    geometry_msgs::msg::Point nearest_traj_point;
    nearest_traj_point.x = traj.x.at(*nearest_index);
    nearest_traj_point.y = traj.y.at(*nearest_index);
    geometry_msgs::msg::Point next_nearest_traj_point;
    next_nearest_traj_point.x = traj.x.at(*nearest_index + 1);
    next_nearest_traj_point.y = traj.y.at(*nearest_index + 1);

    const double signed_length =
      calcLongitudinalOffset(nearest_traj_point, next_nearest_traj_point, self_pose.position);
    if (signed_length <= 0) {
      return std::make_pair(*nearest_index - 1, *nearest_index);
    }
    return std::make_pair(*nearest_index, *nearest_index + 1);
  }();

  geometry_msgs::msg::Point next_traj_point;
  next_traj_point.x = traj.x.at(next);
  next_traj_point.y = traj.y.at(next);
  geometry_msgs::msg::Point prev_traj_point;
  prev_traj_point.x = traj.x.at(prev);
  prev_traj_point.y = traj.y.at(prev);
  const double traj_seg_length = autoware_utils::calc_distance2d(prev_traj_point, next_traj_point);
  /* if distance between two points are too close */
  if (traj_seg_length < 1.0E-5) {
    nearest_pose->position.x = traj.x.at(*nearest_index);
    nearest_pose->position.y = traj.y.at(*nearest_index);
    nearest_pose->orientation = create_quaternion_from_yaw(traj.yaw.at(*nearest_index));
    *nearest_time = traj.relative_time.at(*nearest_index);
    return true;
  }

  /* linear interpolation */
  const double ratio = std::clamp(
    calcLongitudinalOffset(prev_traj_point, next_traj_point, self_pose.position) / traj_seg_length,
    0.0, 1.0);
  nearest_pose->position.x = (1 - ratio) * traj.x.at(prev) + ratio * traj.x.at(next);
  nearest_pose->position.y = (1 - ratio) * traj.y.at(prev) + ratio * traj.y.at(next);
  const double tmp_yaw_err = normalize_radian(traj.yaw.at(prev) - traj.yaw.at(next));
  const double nearest_yaw = normalize_radian(traj.yaw.at(next) + (1 - ratio) * tmp_yaw_err);
  nearest_pose->orientation = create_quaternion_from_yaw(nearest_yaw);
  *nearest_time = (1 - ratio) * traj.relative_time.at(prev) + ratio * traj.relative_time.at(next);
  return true;
}

double calcStopDistance(const Trajectory & current_trajectory, const int origin)
{
  constexpr float zero_velocity = std::numeric_limits<float>::epsilon();
  const float origin_velocity =
    current_trajectory.points.at(static_cast<size_t>(origin)).longitudinal_velocity_mps;
  double stop_dist = 0.0;

  // search forward
  if (std::fabs(origin_velocity) > zero_velocity) {
    for (int i = origin + 1; i < static_cast<int>(current_trajectory.points.size()) - 1; ++i) {
      const auto & p0 = current_trajectory.points.at(i);
      const auto & p1 = current_trajectory.points.at(i - 1);
      stop_dist += calc_distance2d(p0, p1);
      if (std::fabs(p0.longitudinal_velocity_mps) < zero_velocity) {
        break;
      }
    }
    return stop_dist;
  }

  // search backward
  for (int i = origin - 1; 0 < i; --i) {
    const auto & p0 = current_trajectory.points.at(i);
    const auto & p1 = current_trajectory.points.at(i + 1);
    if (std::fabs(p0.longitudinal_velocity_mps) > zero_velocity) {
      break;
    }
    stop_dist -= calc_distance2d(p0, p1);
  }
  return stop_dist;
}

void extendTrajectoryInYawDirection(
  const double yaw, const double interval, const bool is_forward_shift, MPCTrajectory & traj)
{
  if (traj.empty()) return;

  // set terminal yaw
  traj.yaw.back() = yaw;

  // get terminal pose
  const auto autoware_traj = MPCUtils::convertToAutowareTrajectory(traj);
  auto extended_pose = autoware_traj.points.back().pose;

  constexpr double extend_dist = 10.0;
  const double extend_vel = traj.vx.back();
  const double x_offset = is_forward_shift ? interval : -interval;
  constexpr double min_vel_threshold = 0.1;
  const double dt =
    (std::fabs(extend_vel) < min_vel_threshold) ? 1.0e-4 : interval / std::fabs(extend_vel);
  const size_t num_extended_point = static_cast<size_t>(extend_dist / interval);
  for (size_t i = 0; i < num_extended_point; ++i) {
    extended_pose = autoware_utils::calc_offset_pose(extended_pose, x_offset, 0.0, 0.0);
    traj.push_back(
      extended_pose.position.x, extended_pose.position.y, extended_pose.position.z, traj.yaw.back(),
      extend_vel, traj.k.back(), traj.smooth_k.back(), traj.relative_time.back() + dt);
  }
}

MPCTrajectory clipTrajectoryByLength(const MPCTrajectory & trajectory, const double length)
{
  MPCTrajectory clipped_trajectory;
  clipped_trajectory.push_back(trajectory.at(0));

  double current_length = 0.0;
  for (size_t i = 1; i < trajectory.size(); ++i) {
    current_length += calcDistance3d(trajectory, i, i - 1);
    if (current_length > length) {
      break;
    }
    clipped_trajectory.push_back(trajectory.at(i));
  }

  return clipped_trajectory;
}

}  // namespace MPCUtils
}  // namespace autoware::motion::control::mpc_lateral_controller
