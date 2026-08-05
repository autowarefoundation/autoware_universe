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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_STRUCTS_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_STRUCTS_HPP_
#include "autoware/trajectory_processor/semantic_speed_tracker.hpp"

#include <autoware_trajectory_processor/trajectory_optimizer_param.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace autoware::trajectory_optimizer
{
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

using TrajectoryOptimizerParams = trajectory_optimizer_node_params::Params;
using SemanticSpeedTracker = autoware::trajectory_processor::SemanticSpeedTracker;

struct InitialMotion
{
  double speed_mps{0.0};
  double acc_mps2{0.0};
};

// Runtime data struct - contains vehicle state updated each cycle from topics
// and per-trajectory semantic tracking state shared across plugins.
// A fresh instance is created for each candidate trajectory so semantic_speed_tracker
// is automatically reset between trajectories.
struct TrajectoryOptimizerData
{
  Odometry current_odometry;
  AccelWithCovarianceStamped current_acceleration;
  SemanticSpeedTracker semantic_speed_tracker;
};

}  // namespace autoware::trajectory_optimizer
#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_STRUCTS_HPP_
