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
#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_QP_SMOOTHER_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_QP_SMOOTHER_HPP_

#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_optimizer_plugin_base.hpp"
#include "autoware/trajectory_processor/trajectory_optimizer_structs.hpp"

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

class TrajectoryQPSmoother : public TrajectoryOptimizerPluginBase
{
public:
  TrajectoryQPSmoother() = default;
  ~TrajectoryQPSmoother() = default;

  void optimize_trajectory(TrajectoryPoints & traj_points, TrajectoryOptimizerData & data) override;

  void update_params(const TrajectoryOptimizerParams & params) override;

protected:
  void on_initialize(const TrajectoryOptimizerParams & params) override;

private:
  // QP smoother specific parameters
  trajectory_optimizer_node_params::Params::TrajectoryQpSmoother qp_params_;

  /**
   * @brief Solve the QP problem for trajectory smoothing
   * @param input_trajectory Original trajectory from planner
   * @param semantic_speed_tracker Tracker containing semantic speed information (e.g., stop points)
   * @param output_trajectory Smoothed trajectory (output)
   * @return true if optimization succeeded, false otherwise
   */
  bool solve_qp_problem(
    const TrajectoryPoints & input_trajectory, const SemanticSpeedTracker & semantic_speed_tracker,
    TrajectoryPoints & output_trajectory) const;

  /**
   * @brief Construct matrices for OSQP solver
   * @param input_trajectory Original trajectory for fidelity term
   * @param semantic_speed_tracker Tracker used to add hard equality constraints at detected stop
   *        positions (pinning them in place regardless of smoothness weight)
   * @param H Hessian matrix (objective quadratic term)
   * @param A Constraint matrix
   * @param f_vec Gradient vector (objective linear term)
   * @param l_vec Lower bounds for constraints
   * @param u_vec Upper bounds for constraints
   */
  void prepare_osqp_matrices(
    const TrajectoryPoints & input_trajectory, const SemanticSpeedTracker & semantic_speed_tracker,
    Eigen::MatrixXd & H, Eigen::MatrixXd & A, std::vector<double> & f_vec,
    std::vector<double> & l_vec, std::vector<double> & u_vec) const;

  /**
   * @brief Convert QP solution back to trajectory format
   * @param solution Optimization solution vector
   * @param input_trajectory Original trajectory (for reference)
   * @param output_trajectory Reconstructed trajectory with smoothed values
   */
  void post_process_trajectory(
    const Eigen::VectorXd & solution, const TrajectoryPoints & input_trajectory,
    const SemanticSpeedTracker & semantic_speed_tracker,
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
#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_QP_SMOOTHER_HPP_
