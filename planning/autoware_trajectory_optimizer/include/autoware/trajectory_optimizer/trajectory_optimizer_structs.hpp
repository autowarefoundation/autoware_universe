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
struct TrajectoryOptimizerParams
{
  double nearest_dist_threshold_m{0.0};
  double nearest_yaw_threshold_rad{0.0};
  double target_pull_out_speed_mps{0.0};
  double target_pull_out_acc_mps2{0.0};
  double max_speed_mps{0.0};
  double max_lateral_accel_mps2{0.0};
  double spline_interpolation_resolution_m{0.0};
  double spline_interpolation_max_yaw_discrepancy_deg{0.0};
  double spline_interpolation_max_distance_discrepancy_m{0.0};
  double backward_trajectory_extension_m{0.0};
  bool use_akima_spline_interpolation{false};
  bool smooth_velocities{false};
  bool smooth_trajectories{false};
  bool limit_speed{false};
  bool limit_lateral_acceleration{false};
  bool set_engage_speed{false};
  bool fix_invalid_points{false};
  bool extend_trajectory_backward{false};
  bool spline_copy_original_orientation{false};
  // QP Smoother parameters
  bool use_qp_smoother{false};
  double qp_weight_smoothness{10.0};
  double qp_weight_fidelity{100.0};
  double qp_time_step_s{0.1};
  double qp_osqp_eps_abs{1e-4};
  double qp_osqp_eps_rel{1e-4};
  int qp_osqp_max_iter{4000};
  bool qp_osqp_verbose{false};
  Odometry current_odometry;
  AccelWithCovarianceStamped current_acceleration;
};
}  // namespace autoware::trajectory_optimizer
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_STRUCTS_HPP_
