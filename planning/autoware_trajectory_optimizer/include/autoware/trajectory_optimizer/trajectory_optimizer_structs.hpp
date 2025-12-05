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

#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_STRUCTS_HPP_
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_STRUCTS_HPP_
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace autoware::trajectory_optimizer
{
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

struct InitialMotion
{
  double speed_mps{0.0};
  double acc_mps2{0.0};
};

// Runtime data struct - contains vehicle state updated each cycle from topics
// This is NOT configuration, it's runtime state passed to plugins
struct TrajectoryOptimizerData
{
  Odometry current_odometry;
  AccelWithCovarianceStamped current_acceleration;
};

// Main node parameters struct - contains only plugin activation flags
// Plugin-specific parameters are managed by each plugin independently
struct TrajectoryOptimizerParams
{
  bool use_akima_spline_interpolation{false};
  bool use_eb_smoother{false};
  bool use_qp_smoother{false};
  bool fix_invalid_points{false};
  bool optimize_velocity{false};
  bool extend_trajectory_backward{false};
  bool use_kinematic_feasibility_enforcer{false};
  bool use_mpt_optimizer{false};
};
}  // namespace autoware::trajectory_optimizer
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_STRUCTS_HPP_
