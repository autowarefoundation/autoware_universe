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
#include <autoware_utils/ros/update_param.hpp>
#include <rclcpp/logging.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

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

  qp_params_.weight_jerk = get_or_declare_parameter<double>(*node_ptr, "qp_weight_jerk");
  qp_params_.weight_acceleration =
    get_or_declare_parameter<double>(*node_ptr, "qp_weight_acceleration");
  qp_params_.weight_fidelity = get_or_declare_parameter<double>(*node_ptr, "qp_weight_fidelity");
  qp_params_.max_longitudinal_jerk_mps3 =
    get_or_declare_parameter<double>(*node_ptr, "qp_max_longitudinal_jerk_mps3");
  qp_params_.max_acceleration_mps2 =
    get_or_declare_parameter<double>(*node_ptr, "qp_max_acceleration_mps2");
  qp_params_.min_acceleration_mps2 =
    get_or_declare_parameter<double>(*node_ptr, "qp_min_acceleration_mps2");
  qp_params_.max_speed_mps = get_or_declare_parameter<double>(*node_ptr, "qp_max_speed_mps");
  qp_params_.osqp_eps_abs = get_or_declare_parameter<double>(*node_ptr, "qp_osqp_eps_abs");
  qp_params_.osqp_eps_rel = get_or_declare_parameter<double>(*node_ptr, "qp_osqp_eps_rel");
  qp_params_.osqp_max_iter = get_or_declare_parameter<int>(*node_ptr, "qp_osqp_max_iter");
  qp_params_.osqp_verbose = get_or_declare_parameter<bool>(*node_ptr, "qp_osqp_verbose");
}

rcl_interfaces::msg::SetParametersResult TrajectoryQPSmoother::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param<double>(parameters, "qp_weight_jerk", qp_params_.weight_jerk);
  update_param<double>(parameters, "qp_weight_acceleration", qp_params_.weight_acceleration);
  update_param<double>(parameters, "qp_weight_fidelity", qp_params_.weight_fidelity);
  update_param<double>(
    parameters, "qp_max_longitudinal_jerk_mps3", qp_params_.max_longitudinal_jerk_mps3);
  update_param<double>(parameters, "qp_max_acceleration_mps2", qp_params_.max_acceleration_mps2);
  update_param<double>(parameters, "qp_min_acceleration_mps2", qp_params_.min_acceleration_mps2);
  update_param<double>(parameters, "qp_max_speed_mps", qp_params_.max_speed_mps);
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

  // Validate input
  if (traj_points.size() < 3) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "QP Smoother: Trajectory too short (< 3 points), skipping optimization");
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

  // Diagnostic logging
  RCLCPP_INFO(
    get_node_ptr()->get_logger(),
    "QP Smoother: N=%d, dt=%.3f, iters=%d, obj=%.2e, v_range=[%.2f, %.2f] m/s, "
    "path_dev=[avg=%.3fm, max=%.3fm], status=%s",
    N, calculate_time_step(input_trajectory), result.iteration_status, osqp_solver.getObjVal(),
    output_trajectory.front().longitudinal_velocity_mps,
    output_trajectory.back().longitudinal_velocity_mps, avg_deviation, max_deviation,
    osqp_solver.getStatusMessage().c_str());

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

  // Calculate time step for temporal scaling
  const double dt = calculate_time_step(input_trajectory);
  const double dt_sq = dt * dt;

  // Scale curvature weight by 1/dt² for velocity-aware smoothing
  const double weight_curvature_scaled = qp_params_.weight_jerk / dt_sq;

  // Minimize path curvature: Σ ||(p_{i+1} - 2*p_i + p_{i-1})||²
  for (int i = 1; i < N - 1; ++i) {
    const int x_im1 = 2 * (i - 1);
    const int y_im1 = 2 * (i - 1) + 1;
    const int x_i = 2 * i;
    const int y_i = 2 * i + 1;
    const int x_ip1 = 2 * (i + 1);
    const int y_ip1 = 2 * (i + 1) + 1;

    // x-direction curvature
    H(x_im1, x_im1) += weight_curvature_scaled;
    H(x_i, x_i) += 4.0 * weight_curvature_scaled;
    H(x_ip1, x_ip1) += weight_curvature_scaled;
    H(x_im1, x_i) += -2.0 * weight_curvature_scaled;
    H(x_i, x_im1) += -2.0 * weight_curvature_scaled;
    H(x_i, x_ip1) += -2.0 * weight_curvature_scaled;
    H(x_ip1, x_i) += -2.0 * weight_curvature_scaled;
    H(x_im1, x_ip1) += weight_curvature_scaled;
    H(x_ip1, x_im1) += weight_curvature_scaled;

    // y-direction curvature
    H(y_im1, y_im1) += weight_curvature_scaled;
    H(y_i, y_i) += 4.0 * weight_curvature_scaled;
    H(y_ip1, y_ip1) += weight_curvature_scaled;
    H(y_im1, y_i) += -2.0 * weight_curvature_scaled;
    H(y_i, y_im1) += -2.0 * weight_curvature_scaled;
    H(y_i, y_ip1) += -2.0 * weight_curvature_scaled;
    H(y_ip1, y_i) += -2.0 * weight_curvature_scaled;
    H(y_im1, y_ip1) += weight_curvature_scaled;
    H(y_ip1, y_im1) += weight_curvature_scaled;
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
  TrajectoryPoints & output_trajectory)
{
  const size_t N = input_trajectory.size();
  output_trajectory.resize(N);

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
      const double dx = solution[2 * (i + 1)] - x;
      const double dy = solution[2 * (i + 1) + 1] - y;
      const double yaw = calculate_yaw_from_positions(dx, dy);
      output_trajectory[i].pose.orientation = yaw_to_quaternion(yaw);
    } else {
      if (i > 0) {
        output_trajectory[i].pose.orientation = output_trajectory[i - 1].pose.orientation;
      }
    }

    // IMPORTANT: Preserve input velocities and accelerations
    // The QP smoother only optimizes path geometry (x,y)
    // Velocity smoothing is handled by TrajectoryVelocityOptimizer plugin
    output_trajectory[i].longitudinal_velocity_mps = input_trajectory[i].longitudinal_velocity_mps;
    output_trajectory[i].acceleration_mps2 = input_trajectory[i].acceleration_mps2;
  }
}

double TrajectoryQPSmoother::calculate_time_step(const TrajectoryPoints & traj_points) const
{
  if (traj_points.size() < 2) {
    return 0.1;  // Default fallback
  }

  // Calculate average time step
  double total_time = 0.0;
  for (size_t i = 1; i < traj_points.size(); ++i) {
    // Calculate duration difference manually
    const auto & t1 = traj_points[i].time_from_start;
    const auto & t0 = traj_points[i - 1].time_from_start;
    const double dt = (t1.sec - t0.sec) + (t1.nanosec - t0.nanosec) * 1e-9;
    total_time += dt;
  }

  const double avg_dt = total_time / static_cast<double>(traj_points.size() - 1);
  return std::max(0.01, avg_dt);  // Ensure minimum time step
}

double TrajectoryQPSmoother::calculate_yaw_from_positions(double dx, double dy) const
{
  return std::atan2(dy, dx);
}

geometry_msgs::msg::Quaternion TrajectoryQPSmoother::yaw_to_quaternion(double yaw) const
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

}  // namespace autoware::trajectory_optimizer::plugin
