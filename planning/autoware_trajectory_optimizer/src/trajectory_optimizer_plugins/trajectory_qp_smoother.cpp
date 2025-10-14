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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_qp_smoother.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

TrajectoryQPSmoother::TrajectoryQPSmoother(
  const std::string name, rclcpp::Node * node_ptr,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
  const TrajectoryOptimizerParams & params)
: TrajectoryOptimizerPluginBase(name, node_ptr, time_keeper, params)
{
  set_up_params();
}

void TrajectoryQPSmoother::set_up_params()
{
  auto node_ptr = get_node_ptr();
  using autoware_utils_rclcpp::get_or_declare_parameter;

  qp_params_.weight_smoothness =
    get_or_declare_parameter<double>(*node_ptr, "qp_weight_smoothness");
  qp_params_.weight_fidelity = get_or_declare_parameter<double>(*node_ptr, "qp_weight_fidelity");
  qp_params_.time_step_s = get_or_declare_parameter<double>(*node_ptr, "qp_time_step_s");
  qp_params_.osqp_eps_abs = get_or_declare_parameter<double>(*node_ptr, "qp_osqp_eps_abs");
  qp_params_.osqp_eps_rel = get_or_declare_parameter<double>(*node_ptr, "qp_osqp_eps_rel");
  qp_params_.osqp_max_iter = get_or_declare_parameter<int>(*node_ptr, "qp_osqp_max_iter");
  qp_params_.osqp_verbose = get_or_declare_parameter<bool>(*node_ptr, "qp_osqp_verbose");
}

rcl_interfaces::msg::SetParametersResult TrajectoryQPSmoother::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param<double>(parameters, "qp_weight_smoothness", qp_params_.weight_smoothness);
  update_param<double>(parameters, "qp_weight_fidelity", qp_params_.weight_fidelity);
  update_param<double>(parameters, "qp_time_step_s", qp_params_.time_step_s);
  update_param<double>(parameters, "qp_osqp_eps_abs", qp_params_.osqp_eps_abs);
  update_param<double>(parameters, "qp_osqp_eps_rel", qp_params_.osqp_eps_rel);
  update_param<int>(parameters, "qp_osqp_max_iter", qp_params_.osqp_max_iter);
  update_param<bool>(parameters, "qp_osqp_verbose", qp_params_.osqp_verbose);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void TrajectoryQPSmoother::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *get_time_keeper());

  // Check if QP smoother is enabled
  if (!params.use_qp_smoother) {
    return;
  }

  // Validate input: minimum point count
  if (traj_points.size() < 3) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "QP Smoother: Trajectory too short (< 3 points), skipping optimization");
    return;
  }

  // Check minimum path length (skip optimization for very short paths)
  const double path_length = autoware::motion_utils::calcArcLength(traj_points);
  constexpr double min_path_length_m = 5.0;  // Minimum 5 meters for meaningful smoothing

  if (path_length < min_path_length_m) {
    RCLCPP_DEBUG_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "QP Smoother: Path too short (%.2f m < %.2f m), skipping optimization", path_length,
      min_path_length_m);
    return;
  }

  // Solve QP problem
  TrajectoryPoints smoothed_trajectory;
  if (solve_qp_problem(traj_points, smoothed_trajectory)) {
    traj_points = smoothed_trajectory;
    RCLCPP_INFO(
      get_node_ptr()->get_logger(), "QP Smoother: Successfully smoothed trajectory with %zu points",
      traj_points.size());
  } else {
    RCLCPP_ERROR_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 1000,
      "QP Smoother: Optimization FAILED, using original trajectory. Check previous error "
      "messages!");
  }
}

bool TrajectoryQPSmoother::solve_qp_problem(
  const TrajectoryPoints & input_trajectory, TrajectoryPoints & output_trajectory)
{
  const int N = static_cast<int>(input_trajectory.size());
  const int num_variables = 2 * N;  // [x, y] for each point (path-only optimization)

  // Prepare OSQP matrices
  Eigen::MatrixXd H(num_variables, num_variables);
  Eigen::MatrixXd A;
  std::vector<double> f_vec(num_variables);
  std::vector<double> l_vec;
  std::vector<double> u_vec;

  prepare_osqp_matrices(input_trajectory, H, A, f_vec, l_vec, u_vec);

  // Create OSQP solver with settings
  autoware::osqp_interface::OSQPInterface osqp_solver(qp_params_.osqp_eps_abs, true);

  // Configure solver settings
  osqp_solver.updateEpsRel(qp_params_.osqp_eps_rel);
  osqp_solver.updateMaxIter(qp_params_.osqp_max_iter);
  osqp_solver.updateVerbose(qp_params_.osqp_verbose);

  // Solve the QP problem
  auto result = osqp_solver.optimize(H, A, f_vec, l_vec, u_vec);

  // Check solution status
  if (result.solution_status != 1) {
    RCLCPP_ERROR(
      get_node_ptr()->get_logger(),
      "QP Smoother: Optimization FAILED! Status: %d (%s), Iterations: %d, N=%d points",
      result.solution_status, osqp_solver.getStatusMessage().c_str(), result.iteration_status, N);
    return false;
  }

  // Check for NaN values
  const auto has_nan = std::any_of(
    result.primal_solution.begin(), result.primal_solution.end(),
    [](const auto v) { return std::isnan(v); });
  if (has_nan) {
    RCLCPP_WARN(get_node_ptr()->get_logger(), "QP Smoother: Solution contains NaN values");
    return false;
  }

  // Post-process to create output trajectory
  Eigen::VectorXd solution =
    Eigen::Map<Eigen::VectorXd>(result.primal_solution.data(), result.primal_solution.size());
  post_process_trajectory(solution, input_trajectory, output_trajectory);

  // Calculate path deviation metrics
  double max_deviation = 0.0;
  double total_deviation = 0.0;
  for (size_t i = 0; i < output_trajectory.size(); ++i) {
    const double dx = output_trajectory[i].pose.position.x - input_trajectory[i].pose.position.x;
    const double dy = output_trajectory[i].pose.position.y - input_trajectory[i].pose.position.y;
    const double deviation = std::sqrt(dx * dx + dy * dy);
    max_deviation = std::max(max_deviation, deviation);
    total_deviation += deviation;
  }
  const double avg_deviation = total_deviation / static_cast<double>(N);

  // Calculate jerk metrics for smoothness evaluation
  double max_jerk = 0.0;
  double total_jerk = 0.0;
  const double dt = qp_params_.time_step_s;
  for (size_t i = 0; i < output_trajectory.size() - 1; ++i) {
    if (i < output_trajectory.size() - 1) {
      const double jerk =
        (output_trajectory[i + 1].acceleration_mps2 - output_trajectory[i].acceleration_mps2) / dt;
      max_jerk = std::max(max_jerk, std::abs(jerk));
      total_jerk += std::abs(jerk);
    }
  }
  const double avg_jerk = total_jerk / static_cast<double>(N - 1);

  // Diagnostic logging with velocity, acceleration, and jerk metrics
  RCLCPP_INFO(
    get_node_ptr()->get_logger(),
    "QP Smoother: N=%d, dt=%.3f, iters=%d, obj=%.2e, "
    "v=[%.2f, %.2f] m/s, a=[%.2f, %.2f] m/s², "
    "jerk=[avg=%.2f, max=%.2f] m/s³, path_dev=[avg=%.3f, max=%.3f]m",
    N, dt, result.iteration_status, osqp_solver.getObjVal(),
    output_trajectory.front().longitudinal_velocity_mps,
    output_trajectory.back().longitudinal_velocity_mps, output_trajectory.front().acceleration_mps2,
    output_trajectory.back().acceleration_mps2, avg_jerk, max_jerk, avg_deviation, max_deviation);

  return true;
}

void TrajectoryQPSmoother::prepare_osqp_matrices(
  const TrajectoryPoints & input_trajectory, Eigen::MatrixXd & H, Eigen::MatrixXd & A,
  std::vector<double> & f_vec, std::vector<double> & l_vec, std::vector<double> & u_vec)
{
  const int N = static_cast<int>(input_trajectory.size());
  const int num_variables = 2 * N;

  H = Eigen::MatrixXd::Zero(num_variables, num_variables);
  std::fill(f_vec.begin(), f_vec.end(), 0.0);

  // Use fixed time step for temporal scaling
  const double dt = qp_params_.time_step_s;
  const double dt_sq = dt * dt;

  // Scale smoothness weight by 1/dt² for velocity-aware path smoothing
  const double weight_smoothness_scaled = qp_params_.weight_smoothness / dt_sq;

  // Minimize path curvature: Σ ||(p_{i+1} - 2*p_i + p_{i-1})||²
  for (int i = 1; i < N - 1; ++i) {
    const int x_im1 = 2 * (i - 1);
    const int y_im1 = 2 * (i - 1) + 1;
    const int x_i = 2 * i;
    const int y_i = 2 * i + 1;
    const int x_ip1 = 2 * (i + 1);
    const int y_ip1 = 2 * (i + 1) + 1;

    // x-direction curvature
    H(x_im1, x_im1) += weight_smoothness_scaled;
    H(x_i, x_i) += 4.0 * weight_smoothness_scaled;
    H(x_ip1, x_ip1) += weight_smoothness_scaled;
    H(x_im1, x_i) += -2.0 * weight_smoothness_scaled;
    H(x_i, x_im1) += -2.0 * weight_smoothness_scaled;
    H(x_i, x_ip1) += -2.0 * weight_smoothness_scaled;
    H(x_ip1, x_i) += -2.0 * weight_smoothness_scaled;
    H(x_im1, x_ip1) += weight_smoothness_scaled;
    H(x_ip1, x_im1) += weight_smoothness_scaled;

    // y-direction curvature
    H(y_im1, y_im1) += weight_smoothness_scaled;
    H(y_i, y_i) += 4.0 * weight_smoothness_scaled;
    H(y_ip1, y_ip1) += weight_smoothness_scaled;
    H(y_im1, y_i) += -2.0 * weight_smoothness_scaled;
    H(y_i, y_im1) += -2.0 * weight_smoothness_scaled;
    H(y_i, y_ip1) += -2.0 * weight_smoothness_scaled;
    H(y_ip1, y_i) += -2.0 * weight_smoothness_scaled;
    H(y_im1, y_ip1) += weight_smoothness_scaled;
    H(y_ip1, y_im1) += weight_smoothness_scaled;
  }

  for (int i = 0; i < N; ++i) {
    const int x_i = 2 * i;
    const int y_i = 2 * i + 1;
    const double x_orig = input_trajectory[i].pose.position.x;
    const double y_orig = input_trajectory[i].pose.position.y;

    H(x_i, x_i) += qp_params_.weight_fidelity;
    H(y_i, y_i) += qp_params_.weight_fidelity;
    f_vec[x_i] = -qp_params_.weight_fidelity * x_orig;
    f_vec[y_i] = -qp_params_.weight_fidelity * y_orig;
  }

  const int num_constraints = 2;
  A = Eigen::MatrixXd::Zero(num_constraints, num_variables);
  l_vec.resize(num_constraints);
  u_vec.resize(num_constraints);

  A(0, 0) = 1.0;
  l_vec[0] = input_trajectory[0].pose.position.x;
  u_vec[0] = input_trajectory[0].pose.position.x;

  A(1, 1) = 1.0;
  l_vec[1] = input_trajectory[0].pose.position.y;
  u_vec[1] = input_trajectory[0].pose.position.y;
}

void TrajectoryQPSmoother::post_process_trajectory(
  const Eigen::VectorXd & solution, const TrajectoryPoints & input_trajectory,
  TrajectoryPoints & output_trajectory) const
{
  const size_t N = input_trajectory.size();
  output_trajectory.resize(N);

  // Use fixed time step for velocity/acceleration derivation
  const double dt = qp_params_.time_step_s;

  // First pass: Update positions and orientations
  for (size_t i = 0; i < N; ++i) {
    // Copy input trajectory data
    output_trajectory[i] = input_trajectory[i];

    // Update with smoothed positions
    const double x = solution[2 * i];
    const double y = solution[2 * i + 1];

    output_trajectory[i].pose.position.x = x;
    output_trajectory[i].pose.position.y = y;
    output_trajectory[i].pose.position.z = input_trajectory[i].pose.position.z;

    // Recalculate orientation from smoothed path
    if (i < N - 1) {
      // Use motion_utils function to calculate azimuth angle
      geometry_msgs::msg::Point p_from;
      geometry_msgs::msg::Point p_to;
      p_from.x = x;
      p_from.y = y;
      p_to.x = solution[2 * (i + 1)];
      p_to.y = solution[2 * (i + 1) + 1];
      const double yaw = autoware_utils_geometry::calc_azimuth_angle(p_from, p_to);
      output_trajectory[i].pose.orientation =
        autoware_utils_geometry::create_quaternion_from_yaw(yaw);
    } else {
      if (i > 0) {
        output_trajectory[i].pose.orientation = output_trajectory[i - 1].pose.orientation;
      }
    }
  }

  // Second pass: Recalculate velocities from smoothed positions
  // velocity[i] = ||position[i+1] - position[i]|| / dt
  // Note: Start from index 1 to use previous position
  double prev_x = output_trajectory[0].pose.position.x;
  double prev_y = output_trajectory[0].pose.position.y;
  output_trajectory[0].longitudinal_velocity_mps = input_trajectory[0].longitudinal_velocity_mps;

  for (size_t i = 1; i < N; ++i) {
    const double curr_x = output_trajectory[i].pose.position.x;
    const double curr_y = output_trajectory[i].pose.position.y;
    const double distance = std::hypot(curr_x - prev_x, curr_y - prev_y);
    output_trajectory[i].longitudinal_velocity_mps = static_cast<float>(distance / dt);
    prev_x = curr_x;
    prev_y = curr_y;
  }

  // Third pass: Smooth velocities with moving average
  // This helps reduce noise from numerical differentiation
  constexpr size_t velocity_smoothing_window = 3;  // Small window for light smoothing
  if (N >= velocity_smoothing_window) {
    std::vector<float> smoothed_velocities(N);
    for (size_t i = 0; i + velocity_smoothing_window <= N; ++i) {
      double sum_velocity = 0.0;
      for (size_t w = 0; w < velocity_smoothing_window; ++w) {
        sum_velocity += output_trajectory[i + w].longitudinal_velocity_mps;
      }
      smoothed_velocities[i] =
        static_cast<float>(sum_velocity / static_cast<double>(velocity_smoothing_window));
    }

    // Apply smoothed velocities
    for (size_t i = 0; i + velocity_smoothing_window <= N; ++i) {
      output_trajectory[i].longitudinal_velocity_mps = smoothed_velocities[i];
    }

    // Keep last smoothed velocity for remaining points
    const float last_smoothed_vel = smoothed_velocities[N - velocity_smoothing_window];
    for (size_t i = N - velocity_smoothing_window + 1; i < N; ++i) {
      output_trajectory[i].longitudinal_velocity_mps = last_smoothed_vel;
    }
  }

  // Fourth pass: Recalculate accelerations from velocities
  // acceleration[i] = (velocity[i+1] - velocity[i]) / dt
  for (size_t i = 0; i < N - 1; ++i) {
    const double v0 = output_trajectory[i].longitudinal_velocity_mps;
    const double v1 = output_trajectory[i + 1].longitudinal_velocity_mps;
    output_trajectory[i].acceleration_mps2 = static_cast<float>((v1 - v0) / dt);
  }
  // Last point: set acceleration to zero
  output_trajectory[N - 1].acceleration_mps2 = 0.0f;
}

}  // namespace autoware::trajectory_optimizer::plugin
