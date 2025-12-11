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

#include "autoware/trajectory_optimizer/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

void TrajectoryQPSmoother::set_up_params()
{
  auto node_ptr = get_node_ptr();
  using autoware_utils_rclcpp::get_or_declare_parameter;

  qp_params_.weight_smoothness =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_qp_smoother.weight_smoothness");
  qp_params_.weight_fidelity =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_qp_smoother.weight_fidelity");
  qp_params_.time_step_s =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_qp_smoother.time_step_s");
  qp_params_.osqp_eps_abs =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_qp_smoother.osqp_eps_abs");
  qp_params_.osqp_eps_rel =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_qp_smoother.osqp_eps_rel");
  qp_params_.osqp_max_iter =
    get_or_declare_parameter<int>(*node_ptr, "trajectory_qp_smoother.osqp_max_iter");
  qp_params_.osqp_verbose =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_qp_smoother.osqp_verbose");
  qp_params_.preserve_input_trajectory_orientation = get_or_declare_parameter<bool>(
    *node_ptr, "trajectory_qp_smoother.preserve_input_trajectory_orientation");
  qp_params_.max_distance_for_orientation_m = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_qp_smoother.max_distance_for_orientation_m");

  // Velocity-based fidelity parameters
  qp_params_.use_velocity_based_fidelity =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_qp_smoother.use_velocity_based_fidelity");
  qp_params_.velocity_threshold_mps =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_qp_smoother.velocity_threshold_mps");
  qp_params_.sigmoid_sharpness =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_qp_smoother.sigmoid_sharpness");
  qp_params_.min_fidelity_weight =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_qp_smoother.min_fidelity_weight");
  qp_params_.max_fidelity_weight =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_qp_smoother.max_fidelity_weight");

  // Point constraint parameters
  qp_params_.num_constrained_points_start =
    get_or_declare_parameter<int>(*node_ptr, "trajectory_qp_smoother.num_constrained_points_start");
  qp_params_.num_constrained_points_end =
    get_or_declare_parameter<int>(*node_ptr, "trajectory_qp_smoother.num_constrained_points_end");

  // Log configuration at startup
  RCLCPP_DEBUG(
    node_ptr->get_logger(),
    "QP Smoother: velocity-based fidelity = %s, constrained points = [start: %d, end: %d]",
    qp_params_.use_velocity_based_fidelity ? "ENABLED" : "DISABLED",
    qp_params_.num_constrained_points_start, qp_params_.num_constrained_points_end);

  if (qp_params_.use_velocity_based_fidelity) {
    RCLCPP_DEBUG(
      node_ptr->get_logger(),
      "QP Smoother: v_threshold=%.2f m/s, sigmoid_k=%.1f, w_range=[%.2f, %.2f]",
      qp_params_.velocity_threshold_mps, qp_params_.sigmoid_sharpness,
      qp_params_.min_fidelity_weight, qp_params_.max_fidelity_weight);
  }
}

rcl_interfaces::msg::SetParametersResult TrajectoryQPSmoother::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param<double>(
    parameters, "trajectory_qp_smoother.weight_smoothness", qp_params_.weight_smoothness);
  update_param<double>(
    parameters, "trajectory_qp_smoother.weight_fidelity", qp_params_.weight_fidelity);
  update_param<double>(parameters, "trajectory_qp_smoother.time_step_s", qp_params_.time_step_s);
  update_param<double>(parameters, "trajectory_qp_smoother.osqp_eps_abs", qp_params_.osqp_eps_abs);
  update_param<double>(parameters, "trajectory_qp_smoother.osqp_eps_rel", qp_params_.osqp_eps_rel);
  update_param<int>(parameters, "trajectory_qp_smoother.osqp_max_iter", qp_params_.osqp_max_iter);
  update_param<bool>(parameters, "trajectory_qp_smoother.osqp_verbose", qp_params_.osqp_verbose);
  update_param<bool>(
    parameters, "trajectory_qp_smoother.preserve_input_trajectory_orientation",
    qp_params_.preserve_input_trajectory_orientation);
  update_param<double>(
    parameters, "trajectory_qp_smoother.max_distance_for_orientation_m",
    qp_params_.max_distance_for_orientation_m);

  // Velocity-based fidelity parameter updates
  update_param<bool>(
    parameters, "trajectory_qp_smoother.use_velocity_based_fidelity",
    qp_params_.use_velocity_based_fidelity);
  update_param<double>(
    parameters, "trajectory_qp_smoother.velocity_threshold_mps", qp_params_.velocity_threshold_mps);
  update_param<double>(
    parameters, "trajectory_qp_smoother.sigmoid_sharpness", qp_params_.sigmoid_sharpness);
  update_param<double>(
    parameters, "trajectory_qp_smoother.min_fidelity_weight", qp_params_.min_fidelity_weight);
  update_param<double>(
    parameters, "trajectory_qp_smoother.max_fidelity_weight", qp_params_.max_fidelity_weight);

  // Point constraint parameter updates
  update_param<int>(
    parameters, "trajectory_qp_smoother.num_constrained_points_start",
    qp_params_.num_constrained_points_start);
  update_param<int>(
    parameters, "trajectory_qp_smoother.num_constrained_points_end",
    qp_params_.num_constrained_points_end);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void TrajectoryQPSmoother::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
  [[maybe_unused]] const TrajectoryOptimizerData & data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *get_time_keeper());

  if (!params.use_qp_smoother) {
    return;
  }

  // Minimum points needed: base requirement (5) or total constrained points + 1 free point
  constexpr size_t base_min_points = 5;
  const size_t total_constrained_points =
    qp_params_.num_constrained_points_start + qp_params_.num_constrained_points_end;
  const size_t min_points_for_optimization =
    std::max(base_min_points, total_constrained_points + 1);

  if (traj_points.size() < min_points_for_optimization) {
    RCLCPP_DEBUG_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "QP Smoother: Trajectory too short (%zu points < %zu required), skipping optimization",
      traj_points.size(), min_points_for_optimization);
    return;
  }

  // Check minimum path length (skip optimization for very short paths)
  const double path_length = autoware::motion_utils::calcArcLength(traj_points);
  constexpr double min_path_length_m = 0.5;  // Minimum 0.5 meters for meaningful smoothing

  if (path_length < min_path_length_m) {
    RCLCPP_DEBUG_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "QP Smoother: Path too short (%.2f m < %.2f m), skipping optimization", path_length,
      min_path_length_m);
    return;
  }

  // Store original trajectory for orientation correction
  const TrajectoryPoints original_trajectory = traj_points;

  // Solve QP problem
  TrajectoryPoints smoothed_trajectory;
  if (!solve_qp_problem(traj_points, smoothed_trajectory)) {
    RCLCPP_ERROR_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 1000,
      "QP Smoother: Optimization FAILED, using original trajectory. Check previous error "
      "messages!");
    return;
  }
  // Copy orientations from original trajectory
  if (qp_params_.preserve_input_trajectory_orientation) {
    utils::copy_trajectory_orientation(
      original_trajectory, smoothed_trajectory, qp_params_.max_distance_for_orientation_m, M_PI);
  }

  traj_points = smoothed_trajectory;
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

  // Log weight statistics if velocity-based fidelity is enabled
  if (qp_params_.use_velocity_based_fidelity) {
    const std::vector<double> weights = compute_velocity_based_weights(input_trajectory);
    const double min_weight = *std::min_element(weights.begin(), weights.end());
    const double max_weight = *std::max_element(weights.begin(), weights.end());
    const double avg_weight =
      std::accumulate(weights.begin(), weights.end(), 0.0) / static_cast<double>(weights.size());

    RCLCPP_DEBUG_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "QP Smoother: Fidelity weights: min=%.3f, max=%.3f, avg=%.3f (N=%d points)", min_weight,
      max_weight, avg_weight, N);
  }

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
    const double jerk =
      (output_trajectory[i + 1].acceleration_mps2 - output_trajectory[i].acceleration_mps2) / dt;
    max_jerk = std::max(max_jerk, std::abs(jerk));
    total_jerk += std::abs(jerk);
  }
  const double avg_jerk = total_jerk / static_cast<double>(N - 1);

  // Diagnostic logging with velocity, acceleration, and jerk metrics
  RCLCPP_DEBUG_THROTTLE(
    get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
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
  std::vector<double> & f_vec, std::vector<double> & l_vec, std::vector<double> & u_vec) const
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

  // Compute per-point fidelity weights (uniform if feature disabled, velocity-based if enabled)
  const std::vector<double> fidelity_weights = compute_velocity_based_weights(input_trajectory);

  // Fidelity term: minimize || p - p_orig ||² with per-point weights
  for (int i = 0; i < N; ++i) {
    const int x_i = 2 * i;
    const int y_i = 2 * i + 1;
    const double x_orig = input_trajectory[i].pose.position.x;
    const double y_orig = input_trajectory[i].pose.position.y;
    const double w_i = fidelity_weights[i];  // Per-point weight

    H(x_i, x_i) += w_i;
    H(y_i, y_i) += w_i;
    f_vec[x_i] = -w_i * x_orig;
    f_vec[y_i] = -w_i * y_orig;
  }

  const int num_points_start = std::max(0, qp_params_.num_constrained_points_start);
  const int num_points_end = std::max(0, qp_params_.num_constrained_points_end);

  // Constraints: fix points from start and end
  // Each point has 2 constraints (x, y)
  const int num_constraints = 2 * (num_points_start + num_points_end);
  A = Eigen::MatrixXd::Zero(num_constraints, num_variables);
  l_vec.resize(num_constraints);
  u_vec.resize(num_constraints);

  int constraint_idx = 0;

  // Fix first num_points_start points
  for (int i = 0; i < num_points_start; ++i) {
    const int x_idx = 2 * i;
    const int y_idx = 2 * i + 1;

    // Constrain x coordinate
    A(constraint_idx, x_idx) = 1.0;
    l_vec[constraint_idx] = input_trajectory[i].pose.position.x;
    u_vec[constraint_idx] = input_trajectory[i].pose.position.x;
    constraint_idx++;

    // Constrain y coordinate
    A(constraint_idx, y_idx) = 1.0;
    l_vec[constraint_idx] = input_trajectory[i].pose.position.y;
    u_vec[constraint_idx] = input_trajectory[i].pose.position.y;
    constraint_idx++;
  }

  // Fix last num_points_end points
  for (int i = 0; i < num_points_end; ++i) {
    const int point_idx = N - num_points_end + i;
    const int x_idx = 2 * point_idx;
    const int y_idx = 2 * point_idx + 1;

    // Constrain x coordinate
    A(constraint_idx, x_idx) = 1.0;
    l_vec[constraint_idx] = input_trajectory[point_idx].pose.position.x;
    u_vec[constraint_idx] = input_trajectory[point_idx].pose.position.x;
    constraint_idx++;

    // Constrain y coordinate
    A(constraint_idx, y_idx) = 1.0;
    l_vec[constraint_idx] = input_trajectory[point_idx].pose.position.y;
    u_vec[constraint_idx] = input_trajectory[point_idx].pose.position.y;
    constraint_idx++;
  }
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

  // Fourth pass: Recalculate accelerations from velocities using constant dt
  utils::recalculate_longitudinal_acceleration(output_trajectory, true, dt);
}

std::vector<double> TrajectoryQPSmoother::compute_velocity_based_weights(
  const TrajectoryPoints & input_trajectory) const
{
  const size_t N = input_trajectory.size();
  std::vector<double> weights(N);

  // If velocity-based fidelity is DISABLED, return uniform weights (original behavior)
  if (!qp_params_.use_velocity_based_fidelity) {
    std::fill(weights.begin(), weights.end(), qp_params_.weight_fidelity);
    return weights;
  }

  // Velocity-based fidelity is ENABLED - compute sigmoid-based weights
  const double v_th = qp_params_.velocity_threshold_mps;
  const double k = qp_params_.sigmoid_sharpness;
  const double w_min = qp_params_.min_fidelity_weight;
  const double w_max = qp_params_.max_fidelity_weight;

  for (size_t i = 0; i < N; ++i) {
    // Use absolute value of velocity to handle negative velocities (reverse driving)
    const double speed =
      std::abs(static_cast<double>(input_trajectory[i].longitudinal_velocity_mps));

    // Sigmoid function: σ(k * (|v| - v_th))
    const double sigmoid_arg = k * (speed - v_th);
    const double sigmoid_val = 1.0 / (1.0 + std::exp(-sigmoid_arg));

    // Map to weight range: w = w_min + (w_max - w_min) * σ
    weights[i] = w_min + (w_max - w_min) * sigmoid_val;
  }

  return weights;
}

}  // namespace autoware::trajectory_optimizer::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_optimizer::plugin::TrajectoryQPSmoother,
  autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase)
