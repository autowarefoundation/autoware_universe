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

#include "autoware/diffusion_planner/utils/planning_factor_utils.hpp"

#include <rclcpp/duration.hpp>

#include <vector>

namespace autoware::diffusion_planner
{

using autoware_planning_msgs::msg::TrajectoryPoint;

std::optional<DetectedStopFactor> detect_stop_factor(
  const std::vector<TrajectoryPoint> & points, const PlanningFactorDetectionConfig & config)
{
  std::optional<size_t> stop_idx;
  rclcpp::Duration stop_start_time(0, 0);
  bool is_valid_stop = true;

  for (size_t i = 0; i < points.size(); ++i) {
    if (!stop_idx && points[i].longitudinal_velocity_mps <= config.stop_velocity_threshold) {
      stop_idx = i;
      stop_start_time = rclcpp::Duration(points[i].time_from_start);
    }
    if (
      stop_idx &&
      (rclcpp::Duration(points[i].time_from_start) - stop_start_time).seconds() <
        config.stop_keep_duration_threshold &&
      points[i].longitudinal_velocity_mps > config.stop_velocity_threshold) {
      is_valid_stop = false;
    }
  }

  if (!stop_idx || !is_valid_stop) {
    return std::nullopt;
  }

  DetectedStopFactor stop;
  stop.ego_pose = points[0].pose;
  stop.stop_pose = points[*stop_idx].pose;
  return stop;
}

std::optional<DetectedSlowdownFactor> detect_slowdown_factor(
  const std::vector<TrajectoryPoint> & points, const PlanningFactorDetectionConfig & config)
{
  std::optional<size_t> slowdown_start_idx;
  std::optional<size_t> slowdown_end_idx;

  for (size_t i = 0; i < points.size(); ++i) {
    if (points[i].acceleration_mps2 < config.slowdown_accel_threshold) {
      if (!slowdown_start_idx) slowdown_start_idx = i;
    } else if (slowdown_start_idx && !slowdown_end_idx) {
      slowdown_end_idx = i;
    }
  }

  if (slowdown_start_idx && !slowdown_end_idx) {
    slowdown_end_idx = points.size() - 1;
  }

  if (!slowdown_start_idx) {
    return std::nullopt;
  }

  DetectedSlowdownFactor slowdown;
  slowdown.ego_pose = points[0].pose;
  slowdown.start_pose = points[*slowdown_start_idx].pose;
  slowdown.end_pose = points[*slowdown_end_idx].pose;
  slowdown.start_velocity = points[*slowdown_start_idx].longitudinal_velocity_mps;
  slowdown.end_velocity = points[*slowdown_end_idx].longitudinal_velocity_mps;
  return slowdown;
}

PlanningFactorDetectionResult detect_planning_factors(
  const std::vector<TrajectoryPoint> & points, const PlanningFactorDetectionConfig & config)
{
  PlanningFactorDetectionResult result;

  if (points.empty()) {
    return result;
  }

  result.stop = detect_stop_factor(points, config);
  result.slowdown = detect_slowdown_factor(points, config);
  return result;
}

}  // namespace autoware::diffusion_planner
