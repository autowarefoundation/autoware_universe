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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/continuous_jerk_smoother.hpp"

#include <Eigen/Core>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/qp_interface/proxqp_interface.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

ContinuousJerkSmoother::ContinuousJerkSmoother(const ContinuousJerkSmootherParams & params)
: params_(params)
{
  // Initialize QP solver with ProxQP
  qp_interface_ =
    std::make_shared<autoware::qp_interface::ProxQPInterface>(false, 20000, 1.0e-8, 1.0e-6, false);
}

void ContinuousJerkSmoother::set_params(const ContinuousJerkSmootherParams & params)
{
  params_ = params;
}

ContinuousJerkSmootherParams ContinuousJerkSmoother::get_params() const
{
  return params_;
}

std::vector<double> ContinuousJerkSmoother::calc_trajectory_interval_distance(
  const TrajectoryPoints & trajectory) const
{
  std::vector<double> intervals;
  intervals.reserve(trajectory.size() - 1);
  for (size_t i = 1; i < trajectory.size(); ++i) {
    const double dist =
      autoware_utils_geometry::calc_distance2d(trajectory.at(i).pose, trajectory.at(i - 1).pose);
    intervals.push_back(dist);
  }
  return intervals;
}

bool ContinuousJerkSmoother::apply(
  const double v0, const double a0, const TrajectoryPoints & input, TrajectoryPoints & output,
  const std::vector<double> & max_velocity_per_point)
{
  output = input;

  if (input.empty()) {
    RCLCPP_WARN(logger_, "Input TrajectoryPoints to the continuous jerk smoother is empty.");
    return false;
  }

  if (input.size() == 1) {
    // No need to do optimization
    output.front().longitudinal_velocity_mps = v0;
    output.front().acceleration_mps2 = a0;
    return true;
  }

  const double a_max = params_.max_accel;
  const double a_min = params_.min_decel;
  const double j_max = params_.max_jerk;
  const double j_min = params_.min_jerk;
  const double over_j_weight = params_.over_j_weight;
  const double over_v_weight = params_.over_v_weight;
  const double over_a_weight = params_.over_a_weight;
  // const double a_stop_decel = 0.0;
  const double smooth_weight = params_.jerk_weight;
  const double velocity_tracking_weight = params_.velocity_tracking_weight;
  const double accel_tracking_weight = params_.accel_tracking_weight;

  // Search for stop point (zero velocity) starting from index 1
  // to avoid getting 0 as a stop point at the beginning
  const auto zero_vel_id = autoware::motion_utils::searchZeroVelocityIndex(input, 1, input.size());

  // Determine optimization horizon N
  const size_t total_size = input.size();
  size_t N = total_size;
  if (zero_vel_id) {
    N = *zero_vel_id + 1;  // Optimize only up to stop point
  }

  if (N < 2) {
    RCLCPP_WARN(logger_, "Trajectory too small for optimization (N=%zu)", N);
    return false;
  }

  // Store reference velocities from input trajectory (for tracking objective)
  std::vector<double> v_ref_arr(N, 0.0);
  for (size_t i = 0; i < N; ++i) {
    v_ref_arr.at(i) = input.at(i).longitudinal_velocity_mps;
  }

  output = input;

  const std::vector<double> interval_dist_arr = calc_trajectory_interval_distance(input);

  // v_max_arr: Per-point velocity upper bounds (e.g., from lateral acceleration limits)
  std::vector<double> v_max_arr(N, 0.0);
  if (max_velocity_per_point.empty()) {
    // Fallback: use a large value if max_velocity_per_point is not provided
    // (should not happen in practice as it's always initialized in trajectory_velocity_optimizer)
    std::fill(v_max_arr.begin(), v_max_arr.end(), 1000.0);
  } else {
    for (size_t i = 0; i < N; ++i) {
      v_max_arr.at(i) = max_velocity_per_point.at(i);
    }
  }

  /*
   * QP Formulation:
   * x = [
   *      b[0], b[1], ..., b[N-1],               : 0 ~ N-1
   *      a[0], a[1], .... a[N-1],               : N ~ 2N-1
   *      delta[0], ..., delta[N-1],             : 2N ~ 3N-1
   *      sigma[0], sigma[1], ...., sigma[N-1],  : 3N ~ 4N-1
   *      gamma[0], gamma[1], ..., gamma[N-1]    : 4N ~ 5N-2
   *     ]
   *
   * b[i]  : velocity^2
   * delta : 0 < b[i]-delta[i] < max_velocity^2 (velocity upper bound)
   * sigma : a_min < a[i] - sigma[i] < a_max
   * gamma : jerk_min < pseudo_jerk[i] * ref_vel[i] - gamma[i] < jerk_max
   */
  const uint32_t IDX_B0 = 0;
  const uint32_t IDX_A0 = N;
  const uint32_t IDX_DELTA0 = 2 * N;
  const uint32_t IDX_SIGMA0 = 3 * N;
  const uint32_t IDX_GAMMA0 = 4 * N;

  const uint32_t l_variables = 5 * N;    // gamma has N-1 elements
  const uint32_t l_constraints = 4 * N;  // N + N + (N-1) + (N-1) + 2 = 4N - 2

  // Allocate matrices
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(l_constraints, l_variables);
  std::vector<double> lower_bound(l_constraints, 0.0);
  std::vector<double> upper_bound(l_constraints, 0.0);
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(l_variables, l_variables);
  std::vector<double> q(l_variables, 0.0);

  /**************************************************************/
  /**************** design objective function *******************/
  /**************************************************************/

  // Jerk minimization: d(ai)/ds * v_ref -> minimize weight * ((a1 - a0) / ds * v_ref)^2 * ds
  for (size_t i = 0; i < N - 1; ++i) {
    const double ref_vel = 0.5 * (v_ref_arr.at(i) + v_ref_arr.at(i + 1));
    const double interval_dist = std::max(interval_dist_arr.at(i), 0.0001);
    const double w_x_ds_inv = (1.0 / interval_dist) * std::max(ref_vel, 0.1);
    P(IDX_A0 + i, IDX_A0 + i) += smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
    P(IDX_A0 + i, IDX_A0 + i + 1) -= smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
    P(IDX_A0 + i + 1, IDX_A0 + i) -= smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
    P(IDX_A0 + i + 1, IDX_A0 + i + 1) += smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
  }

  // Reference velocity tracking: minimize weight * (b - v_ref^2)^2
  // Expands to: weight * b^2 - 2*weight*v_ref^2*b + const
  // P term: weight on b^2
  // q term: -2*weight*v_ref^2 on b
  // Notice 2 is not needed because the QP solver assumes 1/2 factor in objective P term
  for (size_t i = 0; i < N; ++i) {
    const double ref_vel = v_ref_arr.at(i);
    // const double ref_vel = 1.0;
    P(IDX_B0 + i, IDX_B0 + i) += velocity_tracking_weight;
    q.at(IDX_B0 + i) += -velocity_tracking_weight * ref_vel * ref_vel;
  }

  // Slack variable costs
  for (size_t i = 0; i < N; ++i) {
    // Slack variable costs
    P(IDX_DELTA0 + i, IDX_DELTA0 + i) += over_v_weight;  // over velocity cost
    P(IDX_SIGMA0 + i, IDX_SIGMA0 + i) += over_a_weight;  // over acceleration cost
    P(IDX_GAMMA0 + i, IDX_GAMMA0 + i) += over_j_weight;  // over jerk cost
  }

  // Reference acceleration tracking: minimize weight * (a - a_ref)^2
  // Expands to: weight * a^2 - 2*weight*a_ref*a + const
  // P term: weight on a^2
  // q term: -2*weight*a_ref on a
  for (size_t i = 0; i < N; ++i) {
    const double a_ref = input.at(i).acceleration_mps2;
    P(IDX_A0 + i, IDX_A0 + i) += accel_tracking_weight;
    q.at(IDX_A0 + i) += -accel_tracking_weight * a_ref;
  }

  /**************************************************************/
  /**************** design constraint matrix ********************/
  /**************************************************************/

  size_t constr_idx = 0;

  // Soft Constraint Velocity Limit: 0 < b - delta < v_max^2
  // v_max is from max_velocity_per_point (includes lateral acceleration limits)
  for (size_t i = 0; i < N; ++i, ++constr_idx) {
    A(constr_idx, IDX_B0 + i) = 1.0;       // b_i
    A(constr_idx, IDX_DELTA0 + i) = -1.0;  // -delta_i
    upper_bound[constr_idx] = v_max_arr.at(i) * v_max_arr.at(i);
    lower_bound[constr_idx] = 0.0;
  }

  // Soft Constraint Acceleration Limit: a_min < a - sigma < a_max
  for (size_t i = 0; i < N; ++i, ++constr_idx) {
    A(constr_idx, IDX_A0 + i) = 1.0;       // a_i
    A(constr_idx, IDX_SIGMA0 + i) = -1.0;  // -sigma_i
    upper_bound[constr_idx] = a_max;
    lower_bound[constr_idx] = a_min;
  }

  // Soft Constraint Jerk Limit: jerk_min < pseudo_jerk[i] * ref_vel[i] - gamma[i] < jerk_max
  // -> jerk_min * ds < (a[i+1] - a[i]) * ref_vel[i] - gamma[i] * ds < jerk_max * ds
  for (size_t i = 0; i < N - 1; ++i, ++constr_idx) {
    const double ref_vel = std::max(0.5 * (v_ref_arr.at(i) + v_ref_arr.at(i + 1)), 0.1);
    const double ds = interval_dist_arr.at(i);
    A(constr_idx, IDX_A0 + i) = -ref_vel;     // -a[i] * ref_vel
    A(constr_idx, IDX_A0 + i + 1) = ref_vel;  //  a[i+1] * ref_vel
    A(constr_idx, IDX_GAMMA0 + i) = -ds;      // -gamma[i] * ds
    upper_bound[constr_idx] = j_max * ds;     //  jerk_max * ds
    lower_bound[constr_idx] = j_min * ds;     //  jerk_min * ds
  }

  // Dynamics constraint: b' = 2a ... (b(i+1) - b(i)) / ds = 2a(i)
  for (size_t i = 0; i < N - 1; ++i, ++constr_idx) {
    A(constr_idx, IDX_B0 + i) = -1.0;                            // b(i)
    A(constr_idx, IDX_B0 + i + 1) = 1.0;                         // b(i+1)
    A(constr_idx, IDX_A0 + i) = -2.0 * interval_dist_arr.at(i);  // a(i) * ds
    upper_bound[constr_idx] = 0.0;
    lower_bound[constr_idx] = 0.0;
  }

  // Initial condition constraints
  {
    A(constr_idx, IDX_B0) = 1.0;  // b0
    upper_bound[constr_idx] = v0 * v0;
    lower_bound[constr_idx] = v0 * v0;
    ++constr_idx;

    A(constr_idx, IDX_A0) = 1.0;  // a0
    upper_bound[constr_idx] = a0;
    lower_bound[constr_idx] = a0;
    // ++constr_idx;  // Not needed as this is the last constraint
  }

  // Execute optimization
  const auto optval = qp_interface_->optimize(P, A, q, lower_bound, upper_bound);

  if (!qp_interface_->isSolved()) {
    RCLCPP_WARN(logger_, "Optimization failed: %s", qp_interface_->getStatus().c_str());
    return false;
  }

  const auto has_nan =
    std::any_of(optval.begin(), optval.end(), [](const auto v) { return std::isnan(v); });
  if (has_nan) {
    RCLCPP_WARN(logger_, "Optimization failed: result contains NaN values");
    return false;
  }

  // Extract velocity & acceleration from solution for optimized points
  for (size_t i = 0; i < N; ++i) {
    double b = optval.at(IDX_B0 + i);
    output.at(i).longitudinal_velocity_mps = std::sqrt(std::max(b, 0.0));
    output.at(i).acceleration_mps2 = optval.at(IDX_A0 + i);
  }

  // Set remaining points after stop point to zero velocity
  for (size_t i = N; i < total_size; ++i) {
    output.at(i).longitudinal_velocity_mps = 0.0;
    output.at(i).acceleration_mps2 = 0.0;
  }

  return true;
}

}  // namespace autoware::trajectory_optimizer::plugin
