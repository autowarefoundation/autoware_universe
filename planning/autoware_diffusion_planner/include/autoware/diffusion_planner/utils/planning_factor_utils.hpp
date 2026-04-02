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

#ifndef AUTOWARE__DIFFUSION_PLANNER__UTILS__PLANNING_FACTOR_UTILS_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__UTILS__PLANNING_FACTOR_UTILS_HPP_

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <optional>
#include <vector>

namespace autoware::diffusion_planner
{

struct PlanningFactorDetectionConfig
{
  double stop_velocity_threshold{0.1};
  double stop_keep_duration_threshold{1.0};
  double slowdown_accel_threshold{-0.3};
};

struct DetectedStopFactor
{
  geometry_msgs::msg::Pose ego_pose;
  geometry_msgs::msg::Pose stop_pose;
};

struct DetectedSlowdownFactor
{
  geometry_msgs::msg::Pose ego_pose;
  geometry_msgs::msg::Pose start_pose;
  geometry_msgs::msg::Pose end_pose;
  double start_velocity;
  double end_velocity;
};

struct PlanningFactorDetectionResult
{
  std::optional<DetectedStopFactor> stop;
  std::optional<DetectedSlowdownFactor> slowdown;
};

PlanningFactorDetectionResult detect_planning_factors(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const PlanningFactorDetectionConfig & config);

}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__UTILS__PLANNING_FACTOR_UTILS_HPP_
