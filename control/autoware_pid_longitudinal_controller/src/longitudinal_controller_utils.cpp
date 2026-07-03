// Copyright 2018-2021 Tier IV, Inc. All rights reserved.
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

#include "autoware/pid_longitudinal_controller/longitudinal_controller_utils.hpp"

#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/math/normalization.hpp"

#include <experimental/optional>  // NOLINT
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{
namespace longitudinal_utils
{
bool isValidTrajectory(const Trajectory & traj, const bool use_temporal_trajectory)
{
  // Check if trajectory is empty first
  if (traj.points.empty()) {
    return false;
  }

  double prev_t = -std::numeric_limits<double>::infinity();
  for (const auto & p : traj.points) {
    if (
      !isfinite(p.pose.position.x) || !isfinite(p.pose.position.y) ||
      !isfinite(p.pose.position.z) || !isfinite(p.pose.orientation.w) ||
      !isfinite(p.pose.orientation.x) || !isfinite(p.pose.orientation.y) ||
      !isfinite(p.pose.orientation.z) || !isfinite(p.longitudinal_velocity_mps) ||
      !isfinite(p.lateral_velocity_mps) || !isfinite(p.acceleration_mps2) ||
      !isfinite(p.heading_rate_rps)) {
      return false;
    }

    if (use_temporal_trajectory) {
      const double t = rclcpp::Duration(p.time_from_start).seconds();
      if (!std::isfinite(t) || t <= prev_t) {
        return false;
      }
      prev_t = t;
    }
  }

  return true;
}

double calcStopDistance(
  const Pose & current_pose, const Trajectory & traj, const double max_dist, const double max_yaw)
{
  const auto stop_idx_opt = autoware::motion_utils::searchZeroVelocityIndex(traj.points);

  const size_t end_idx = stop_idx_opt ? *stop_idx_opt : traj.points.size() - 1;
  const size_t seg_idx = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    traj.points, current_pose, max_dist, max_yaw);
  const double signed_length_on_traj = autoware::motion_utils::calcSignedArcLength(
    traj.points, current_pose.position, seg_idx, traj.points.at(end_idx).pose.position,
    std::min(end_idx, traj.points.size() - 2));

  if (std::isnan(signed_length_on_traj)) {
    return 0.0;
  }
  return signed_length_on_traj;
}

double getPitchByPose(const Quaternion & quaternion_msg)
{
  double roll, pitch, yaw;
  tf2::Quaternion quaternion;
  tf2::fromMsg(quaternion_msg, quaternion);
  tf2::Matrix3x3{quaternion}.getRPY(roll, pitch, yaw);

  return pitch;
}

double getPitchByTraj(
  const Trajectory & trajectory, const size_t start_idx, const double wheel_base)
{
  // cannot calculate pitch
  if (trajectory.points.size() <= 1) {
    return 0.0;
  }

  const auto [prev_idx, next_idx] = [&]() {
    for (size_t i = start_idx + 1; i < trajectory.points.size(); ++i) {
      const double dist =
        autoware_utils::calc_distance3d(trajectory.points.at(start_idx), trajectory.points.at(i));
      if (dist > wheel_base) {
        // calculate pitch from trajectory between rear wheel (nearest) and front center (i)
        return std::make_pair(start_idx, i);
      }
    }
    // NOTE: The ego pose is close to the goal.
    return std::make_pair(
      std::min(start_idx, trajectory.points.size() - 2), trajectory.points.size() - 1);
  }();

  return autoware_utils::calc_elevation_angle(
    trajectory.points.at(prev_idx).pose.position, trajectory.points.at(next_idx).pose.position);
}

Pose calcPoseAfterTimeDelay(
  const Pose & current_pose, const double delay_time, const double current_vel,
  const double current_acc)
{
  if (delay_time <= 0.0) {
    return current_pose;
  }

  // check time to stop
  const double time_to_stop = -current_vel / current_acc;

  const double delay_time_calculation =
    time_to_stop > 0.0 && time_to_stop < delay_time ? time_to_stop : delay_time;
  // simple linear prediction
  const double yaw = tf2::getYaw(current_pose.orientation);
  const double running_distance = delay_time_calculation * current_vel + 0.5 * current_acc *
                                                                           delay_time_calculation *
                                                                           delay_time_calculation;
  const double dx = running_distance * std::cos(yaw);
  const double dy = running_distance * std::sin(yaw);

  auto pred_pose = current_pose;
  pred_pose.position.x += dx;
  pred_pose.position.y += dy;
  return pred_pose;
}

double lerp(const double v_from, const double v_to, const double ratio)
{
  return v_from + (v_to - v_from) * ratio;
}

double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double max_val,
  const double min_val)
{
  const double diff_raw = (input_val - prev_val) / dt;
  const double diff = std::min(std::max(diff_raw, min_val), max_val);
  const double filtered_val = prev_val + diff * dt;
  return filtered_val;
}

double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double lim_val)
{
  const double max_val = std::fabs(lim_val);
  const double min_val = -max_val;
  return applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val);
}

geometry_msgs::msg::Pose findTrajectoryPoseAfterDistance(
  const size_t src_idx, const double distance,
  const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  double remain_dist = distance;
  geometry_msgs::msg::Pose p = trajectory.points.back().pose;
  for (size_t i = src_idx; i < trajectory.points.size() - 1; ++i) {
    const double dist = autoware_utils::calc_distance3d(
      trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose);
    if (remain_dist < dist) {
      if (remain_dist <= 0.0) {
        return trajectory.points.at(i).pose;
      }
      double ratio = remain_dist / dist;
      const auto p0 = trajectory.points.at(i).pose;
      const auto p1 = trajectory.points.at(i + 1).pose;
      p = trajectory.points.at(i).pose;
      p.position.x = autoware::interpolation::lerp(p0.position.x, p1.position.x, ratio);
      p.position.y = autoware::interpolation::lerp(p0.position.y, p1.position.y, ratio);
      p.position.z = autoware::interpolation::lerp(p0.position.z, p1.position.z, ratio);
      p.orientation =
        autoware::interpolation::lerpOrientation(p0.orientation, p1.orientation, ratio);
      break;
    }
    remain_dist -= dist;
  }
  return p;
}

std::pair<TrajectoryPoint, size_t> lerpTrajectoryPointByTime(
  const std::vector<TrajectoryPoint> & points, const double target_time)
{
  if (points.empty()) {
    return std::make_pair(TrajectoryPoint{}, 0);
  }
  if (points.size() == 1) {
    return std::make_pair(points.front(), 0);
  }

  const double front_time = rclcpp::Duration(points.front().time_from_start).seconds();
  if (target_time <= front_time) {
    return std::make_pair(points.front(), 0);
  }

  const double back_time = rclcpp::Duration(points.back().time_from_start).seconds();
  if (target_time >= back_time) {
    return std::make_pair(points.back(), points.size() - 1);
  }

  for (size_t i = 0; i < points.size() - 1; ++i) {
    const double t0 = rclcpp::Duration(points.at(i).time_from_start).seconds();
    const double t1 = rclcpp::Duration(points.at(i + 1).time_from_start).seconds();
    if (target_time > t1) {
      continue;
    }
    const double ratio = std::clamp(
      (target_time - t0) / std::max(t1 - t0, std::numeric_limits<double>::epsilon()), 0.0, 1.0);
    auto interpolated = points.at(i);
    const auto & p0 = points.at(i);
    const auto & p1 = points.at(i + 1);
    interpolated.pose.position.x =
      autoware::interpolation::lerp(p0.pose.position.x, p1.pose.position.x, ratio);
    interpolated.pose.position.y =
      autoware::interpolation::lerp(p0.pose.position.y, p1.pose.position.y, ratio);
    interpolated.pose.position.z =
      autoware::interpolation::lerp(p0.pose.position.z, p1.pose.position.z, ratio);
    interpolated.pose.orientation =
      autoware::interpolation::lerpOrientation(p0.pose.orientation, p1.pose.orientation, ratio);
    interpolated.longitudinal_velocity_mps = autoware::interpolation::lerp(
      p0.longitudinal_velocity_mps, p1.longitudinal_velocity_mps, ratio);
    interpolated.lateral_velocity_mps =
      autoware::interpolation::lerp(p0.lateral_velocity_mps, p1.lateral_velocity_mps, ratio);
    interpolated.acceleration_mps2 =
      autoware::interpolation::lerp(p0.acceleration_mps2, p1.acceleration_mps2, ratio);
    interpolated.heading_rate_rps =
      autoware::interpolation::lerp(p0.heading_rate_rps, p1.heading_rate_rps, ratio);
    interpolated.time_from_start = rclcpp::Duration::from_seconds(target_time);
    return std::make_pair(interpolated, i);
  }

  return std::make_pair(points.back(), points.size() - 1);
}

std::optional<double> estimateTrajectoryTimeFromPose(
  const std::vector<TrajectoryPoint> & points, const Pose & pose, const double max_dist,
  const double max_yaw, const double min_time_window_sec, const double max_time_window_sec)
{
  if (points.empty()) {
    return std::nullopt;
  }

  const double min_time_sec = std::min(min_time_window_sec, max_time_window_sec);
  const double max_time_sec = std::max(min_time_window_sec, max_time_window_sec);
  std::vector<size_t> candidates;
  candidates.reserve(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    const double t = rclcpp::Duration(points.at(i).time_from_start).seconds();
    if (min_time_sec <= t && t <= max_time_sec) {
      candidates.push_back(i);
    }
  }
  if (candidates.empty()) {
    return std::nullopt;
  }

  const double self_yaw = tf2::getYaw(pose.orientation);
  size_t best_relaxed_idx = candidates.front();
  double best_relaxed_dist2 = std::numeric_limits<double>::max();
  size_t best_strict_idx = candidates.front();
  double best_strict_dist2 = std::numeric_limits<double>::max();
  bool has_strict_candidate = false;
  for (const auto idx : candidates) {
    const double dx = points.at(idx).pose.position.x - pose.position.x;
    const double dy = points.at(idx).pose.position.y - pose.position.y;
    const double dist2 = dx * dx + dy * dy;
    if (dist2 < best_relaxed_dist2) {
      best_relaxed_dist2 = dist2;
      best_relaxed_idx = idx;
    }

    const double yaw_error = std::fabs(
      autoware_utils::normalize_radian(self_yaw - tf2::getYaw(points.at(idx).pose.orientation)));
    if (std::sqrt(dist2) <= max_dist && yaw_error <= max_yaw && dist2 < best_strict_dist2) {
      best_strict_dist2 = dist2;
      best_strict_idx = idx;
      has_strict_candidate = true;
    }
  }

  const size_t nearest_index = has_strict_candidate ? best_strict_idx : best_relaxed_idx;
  if (points.size() == 1) {
    return rclcpp::Duration(points.front().time_from_start).seconds();
  }

  const auto [prev, next] = [&]() -> std::pair<size_t, size_t> {
    if (nearest_index == 0) {
      return std::make_pair(0, 1);
    }
    if (nearest_index == points.size() - 1) {
      return std::make_pair(points.size() - 2, points.size() - 1);
    }

    const double signed_length =
      autoware::motion_utils::calcLongitudinalOffsetToSegment(points, nearest_index, pose.position);
    if (signed_length <= 0.0) {
      return std::make_pair(nearest_index - 1, nearest_index);
    }
    return std::make_pair(nearest_index, nearest_index + 1);
  }();

  const double seg_length = autoware::motion_utils::calcSignedArcLength(points, prev, next);
  if (std::fabs(seg_length) < 1.0e-5) {
    return rclcpp::Duration(points.at(nearest_index).time_from_start).seconds();
  }

  const double len_to_interpolated =
    autoware::motion_utils::calcLongitudinalOffsetToSegment(points, prev, pose.position);
  const double ratio = std::clamp(len_to_interpolated / seg_length, 0.0, 1.0);
  const double prev_time = rclcpp::Duration(points.at(prev).time_from_start).seconds();
  const double next_time = rclcpp::Duration(points.at(next).time_from_start).seconds();
  return autoware::interpolation::lerp(prev_time, next_time, ratio);
}

double estimateLocalTrajectoryTimeStep(
  const std::vector<TrajectoryPoint> & points, const double target_time)
{
  if (points.size() < 2) {
    return 0.0;
  }

  const double front_time = rclcpp::Duration(points.front().time_from_start).seconds();
  if (target_time <= front_time) {
    return rclcpp::Duration(points.at(1).time_from_start).seconds() - front_time;
  }

  const double back_time = rclcpp::Duration(points.back().time_from_start).seconds();
  if (target_time >= back_time) {
    return back_time - rclcpp::Duration(points.at(points.size() - 2).time_from_start).seconds();
  }

  for (size_t i = 0; i < points.size() - 1; ++i) {
    const double t0 = rclcpp::Duration(points.at(i).time_from_start).seconds();
    const double t1 = rclcpp::Duration(points.at(i + 1).time_from_start).seconds();
    if (target_time <= t1) {
      return t1 - t0;
    }
  }

  return back_time - rclcpp::Duration(points.at(points.size() - 2).time_from_start).seconds();
}
}  // namespace longitudinal_utils
}  // namespace autoware::motion::control::pid_longitudinal_controller
