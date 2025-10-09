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
    RCLCPP_DEBUG(
      get_node_ptr()->get_logger(), "QP Smoother: Successfully smoothed trajectory with %zu points",
      traj_points.size());
  } else {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "QP Smoother: Optimization failed, using original trajectory");
  }
}

bool TrajectoryQPSmoother::solve_qp_problem(
  const TrajectoryPoints & input_trajectory, TrajectoryPoints & output_trajectory)
{
  const int N = static_cast<int>(input_trajectory.size());
  const int num_variables = 4 * N;  // [x, y, v, a] for each point

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
    RCLCPP_WARN(
      get_node_ptr()->get_logger(), "QP Smoother: Optimization failed with status: %d",
      result.solution_status);
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

  RCLCPP_DEBUG(
    get_node_ptr()->get_logger(), "QP Smoother: Solved in %d iterations, objective value: %f",
    result.iteration_status, osqp_solver.getObjVal());

  return true;
}

void TrajectoryQPSmoother::prepare_osqp_matrices(
  const TrajectoryPoints & input_trajectory, Eigen::MatrixXd & H, Eigen::MatrixXd & A,
  std::vector<double> & f_vec, std::vector<double> & l_vec, std::vector<double> & u_vec)
{
  const int N = static_cast<int>(input_trajectory.size());
  const int num_variables = 4 * N;
  const double dt = calculate_time_step(input_trajectory);

  // Initialize Hessian (H) and gradient (f)
  H = Eigen::MatrixXd::Zero(num_variables, num_variables);
  std::fill(f_vec.begin(), f_vec.end(), 0.0);

  // Add jerk minimization cost: min Σ (a_{i+1} - a_i)^2 / dt^2
  const double jerk_coeff = qp_params_.weight_jerk / (dt * dt);
  for (int i = 0; i < N - 1; ++i) {
    const int a_i = 4 * i + 3;
    const int a_ip1 = 4 * (i + 1) + 3;
    H(a_i, a_i) += jerk_coeff;
    H(a_ip1, a_ip1) += jerk_coeff;
    H(a_i, a_ip1) -= jerk_coeff;
    H(a_ip1, a_i) -= jerk_coeff;
  }

  // Add acceleration minimization cost: min Σ a_i^2
  for (int i = 0; i < N; ++i) {
    const int a_i = 4 * i + 3;
    H(a_i, a_i) += qp_params_.weight_acceleration;
  }

  // Add path fidelity cost: min Σ (x_i - x_orig)^2 + (y_i - y_orig)^2
  for (int i = 0; i < N; ++i) {
    const int x_i = 4 * i;
    const int y_i = 4 * i + 1;
    const double x_orig = input_trajectory[i].pose.position.x;
    const double y_orig = input_trajectory[i].pose.position.y;

    H(x_i, x_i) += qp_params_.weight_fidelity;
    H(y_i, y_i) += qp_params_.weight_fidelity;
    f_vec[x_i] = -qp_params_.weight_fidelity * x_orig;
    f_vec[y_i] = -qp_params_.weight_fidelity * y_orig;
  }

  // Prepare constraint matrix A and bounds l, u
  // Constraints: jerk limits (2*(N-1)) + velocity bounds (2*N) + accel bounds (2*N)
  const int num_constraints = 2 * (N - 1) + 2 * N + 2 * N;
  A = Eigen::MatrixXd::Zero(num_constraints, num_variables);
  l_vec.resize(num_constraints);
  u_vec.resize(num_constraints);

  int row = 0;

  // Jerk constraints: -dt*J_max <= a_{i+1} - a_i <= dt*J_max
  const double jerk_bound = dt * qp_params_.max_longitudinal_jerk_mps3;
  for (int i = 0; i < N - 1; ++i) {
    const int a_i = 4 * i + 3;
    const int a_ip1 = 4 * (i + 1) + 3;

    // a_{i+1} - a_i <= jerk_bound
    A(row, a_i) = -1.0;
    A(row, a_ip1) = 1.0;
    l_vec[row] = -autoware::osqp_interface::INF;
    u_vec[row] = jerk_bound;
    row++;

    // a_i - a_{i+1} <= jerk_bound
    A(row, a_i) = 1.0;
    A(row, a_ip1) = -1.0;
    l_vec[row] = -autoware::osqp_interface::INF;
    u_vec[row] = jerk_bound;
    row++;
  }

  // Velocity constraints: 0 <= v_i <= V_max
  for (int i = 0; i < N; ++i) {
    const int v_i = 4 * i + 2;

    // v_i <= V_max
    A(row, v_i) = 1.0;
    l_vec[row] = 0.0;
    u_vec[row] = qp_params_.max_speed_mps;
    row++;

    // -v_i <= 0 (v_i >= 0)
    A(row, v_i) = -1.0;
    l_vec[row] = -autoware::osqp_interface::INF;
    u_vec[row] = 0.0;
    row++;
  }

  // Acceleration constraints: A_min <= a_i <= A_max
  for (int i = 0; i < N; ++i) {
    const int a_i = 4 * i + 3;

    // a_i <= A_max
    A(row, a_i) = 1.0;
    l_vec[row] = qp_params_.min_acceleration_mps2;
    u_vec[row] = qp_params_.max_acceleration_mps2;
    row++;

    // -a_i <= -A_min
    A(row, a_i) = -1.0;
    l_vec[row] = -qp_params_.max_acceleration_mps2;
    u_vec[row] = -qp_params_.min_acceleration_mps2;
    row++;
  }
}

void TrajectoryQPSmoother::post_process_trajectory(
  const Eigen::VectorXd & solution, const TrajectoryPoints & input_trajectory,
  TrajectoryPoints & output_trajectory)
{
  const size_t N = input_trajectory.size();
  output_trajectory.resize(N);

  // Extract optimized values and reconstruct trajectory
  for (size_t i = 0; i < N; ++i) {
    const int idx = 4 * i;
    const double x = solution[idx];
    const double y = solution[idx + 1];
    const double v = solution[idx + 2];
    const double a = solution[idx + 3];

    // Copy base structure from input
    output_trajectory[i] = input_trajectory[i];

    // Update position
    output_trajectory[i].pose.position.x = x;
    output_trajectory[i].pose.position.y = y;
    // Keep z from original
    output_trajectory[i].pose.position.z = input_trajectory[i].pose.position.z;

    // Update velocity and acceleration
    output_trajectory[i].longitudinal_velocity_mps = std::max(0.0, v);
    output_trajectory[i].acceleration_mps2 = a;

    // Calculate and update heading (yaw) from position changes
    if (i < N - 1) {
      const double dx = solution[4 * (i + 1)] - x;
      const double dy = solution[4 * (i + 1) + 1] - y;
      const double yaw = calculate_yaw_from_positions(dx, dy);
      output_trajectory[i].pose.orientation = yaw_to_quaternion(yaw);
    } else {
      // Last point: use previous heading
      if (i > 0) {
        output_trajectory[i].pose.orientation = output_trajectory[i - 1].pose.orientation;
      }
    }
  }

  // Recalculate time_from_start based on new velocities
  output_trajectory[0].time_from_start = input_trajectory[0].time_from_start;
  for (size_t i = 1; i < N; ++i) {
    const double dx =
      output_trajectory[i].pose.position.x - output_trajectory[i - 1].pose.position.x;
    const double dy =
      output_trajectory[i].pose.position.y - output_trajectory[i - 1].pose.position.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    const double avg_vel = (output_trajectory[i].longitudinal_velocity_mps +
                            output_trajectory[i - 1].longitudinal_velocity_mps) /
                           2.0;
    const double dt = (avg_vel > 0.01) ? (dist / avg_vel) : 0.0;

    // Convert to builtin_interfaces::msg::Duration
    rclcpp::Duration duration = rclcpp::Duration::from_seconds(dt);
    output_trajectory[i].time_from_start.sec =
      output_trajectory[i - 1].time_from_start.sec + static_cast<int32_t>(duration.seconds());
    output_trajectory[i].time_from_start.nanosec =
      output_trajectory[i - 1].time_from_start.nanosec +
      static_cast<uint32_t>(duration.nanoseconds() % 1000000000);

    // Handle nanosecond overflow
    if (output_trajectory[i].time_from_start.nanosec >= 1000000000) {
      output_trajectory[i].time_from_start.sec += 1;
      output_trajectory[i].time_from_start.nanosec -= 1000000000;
    }
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
