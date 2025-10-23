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

// NOLINTNEXTLINE
#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_QP_SMOOTHER_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_QP_SMOOTHER_HPP_

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_optimizer_plugin_base.hpp"

#include <Eigen/Dense>
#include <autoware/osqp_interface/osqp_interface.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

/**
 * @brief Parameters specific to the QP smoother
 */
struct QPSmootherParams
{
  // Optimization weights
  double weight_smoothness{10.0};  // Weight for path curvature/smoothness minimization
  double weight_fidelity{1.0};     // Baseline fidelity (used when velocity-based disabled)

  // Time discretization
  double time_step_s{0.1};  // Fixed time step for velocity/acceleration calculations [s]

  // Solver settings
  double osqp_eps_abs{1e-4};
  double osqp_eps_rel{1e-4};
  int osqp_max_iter{4000};
  bool osqp_verbose{false};

  // Orientation correction
  bool fix_orientation{true};  // Enable orientation correction
  double orientation_correction_threshold_deg{
    5.0};  // Yaw threshold for orientation correction [deg]

  // Velocity-based fidelity weighting
  bool use_velocity_based_fidelity{false};  // Master switch for velocity-based weighting
  double velocity_threshold_mps{0.2};       // Speed at sigmoid transition midpoint [m/s]
  double sigmoid_sharpness{40.0};           // Sigmoid steepness (higher = sharper)
  double min_fidelity_weight{0.1};          // Minimum fidelity at very low speeds
  double max_fidelity_weight{1.0};          // Maximum fidelity at high speeds

  // Endpoint constraints
  bool constrain_last_point{true};  // Fix last point as hard constraint
};

/**
 * @brief QP-based trajectory smoother for path geometry optimization
 *
 * This plugin smooths trajectories using quadratic programming optimization
 * that minimizes path curvature while maintaining path fidelity to the original trajectory.
 * After path smoothing, velocities and accelerations are recalculated from the smoothed
 * positions to ensure kinematic consistency.
 *
 * Decision variables: [x_0, y_0, ..., x_{N-1}, y_{N-1}] (path-only optimization)
 * Cost function: weighted sum of path curvature minimization and path fidelity
 * Constraints: fixed initial position
 * Post-processing: velocity/acceleration derived from smoothed path geometry
 */
class TrajectoryQPSmoother : public TrajectoryOptimizerPluginBase
{
public:
  TrajectoryQPSmoother(
    const std::string name, rclcpp::Node * node_ptr,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    const TrajectoryOptimizerParams & params);

  ~TrajectoryQPSmoother() = default;

  void optimize_trajectory(
    TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
    const TrajectoryOptimizerData & data) override;

  void set_up_params() override;

  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  // QP smoother specific parameters
  QPSmootherParams qp_params_;

  /**
   * @brief Solve the QP problem for trajectory smoothing
   * @param input_trajectory Original trajectory from planner
   * @param output_trajectory Smoothed trajectory (output)
   * @return true if optimization succeeded, false otherwise
   */
  bool solve_qp_problem(
    const TrajectoryPoints & input_trajectory, TrajectoryPoints & output_trajectory);

  /**
   * @brief Construct matrices for OSQP solver
   * @param input_trajectory Original trajectory for fidelity term
   * @param H Hessian matrix (objective quadratic term)
   * @param A Constraint matrix
   * @param f_vec Gradient vector (objective linear term)
   * @param l_vec Lower bounds for constraints
   * @param u_vec Upper bounds for constraints
   */
  void prepare_osqp_matrices(
    const TrajectoryPoints & input_trajectory, Eigen::MatrixXd & H, Eigen::MatrixXd & A,
    std::vector<double> & f_vec, std::vector<double> & l_vec, std::vector<double> & u_vec) const;

  /**
   * @brief Convert QP solution back to trajectory format
   * @param solution Optimization solution vector
   * @param input_trajectory Original trajectory (for reference)
   * @param output_trajectory Reconstructed trajectory with smoothed values
   */
  void post_process_trajectory(
    const Eigen::VectorXd & solution, const TrajectoryPoints & input_trajectory,
    TrajectoryPoints & output_trajectory) const;

  /**
   * @brief Compute velocity-dependent fidelity weights for each trajectory point
   * @param input_trajectory Original trajectory with velocity data
   * @return Vector of per-point fidelity weights (length N)
   * @note Returns uniform weights if use_velocity_based_fidelity is false
   */
  std::vector<double> compute_velocity_based_weights(
    const TrajectoryPoints & input_trajectory) const;
};

}  // namespace autoware::trajectory_optimizer::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_QP_SMOOTHER_HPP_
