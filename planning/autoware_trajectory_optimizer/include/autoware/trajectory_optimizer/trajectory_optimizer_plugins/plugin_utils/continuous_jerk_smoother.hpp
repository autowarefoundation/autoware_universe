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
#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__CONTINUOUS_JERK_SMOOTHER_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__CONTINUOUS_JERK_SMOOTHER_HPP_

#include <autoware/qp_interface/qp_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <memory>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

/**
 * @brief Parameters for the ContinuousJerkSmoother
 *
 * This smoother is adapted from JerkFilteredSmoother but works with continuous
 * trajectories. It supports stop point detection and reference velocity tracking.
 */
struct ContinuousJerkSmootherParams
{
  // QP optimization weights
  double jerk_weight{10.0};                ///< Weight for jerk minimization
  double over_v_weight{10000.0};           ///< Weight for velocity limit violation
  double over_a_weight{5000.0};            ///< Weight for acceleration limit violation
  double over_j_weight{200.0};             ///< Weight for jerk limit violation
  double velocity_tracking_weight{100.0};  ///< Weight for tracking reference velocity
  double accel_tracking_weight{10.0};      ///< Weight for tracking reference acceleration

  // Kinematic limits
  double max_accel{2.0};   ///< Maximum acceleration [m/s²]
  double min_decel{-3.0};  ///< Minimum deceleration [m/s²]
  double max_jerk{1.5};    ///< Maximum jerk [m/s³]
  double min_jerk{-1.5};   ///< Minimum jerk [m/s³]
};

/**
 * @brief Jerk-filtered velocity smoother for continuous trajectories
 *
 * This class provides jerk-constrained velocity smoothing for trajectories.
 * Key features:
 * - Stop point detection: optimizes only up to zero velocity points
 * - Reference velocity tracking: tracks input trajectory velocities (soft constraint)
 * - Velocity upper bound: enforces per-point max velocity limits (hard constraint)
 *
 * The optimization uses QP (Quadratic Programming) to find smooth velocity
 * profiles that respect acceleration and jerk constraints while tracking
 * the reference velocity profile.
 */
class ContinuousJerkSmoother
{
public:
  /**
   * @brief Construct a new ContinuousJerkSmoother
   * @param params Smoother parameters
   */
  explicit ContinuousJerkSmoother(const ContinuousJerkSmootherParams & params);

  /**
   * @brief Default destructor
   */
  ~ContinuousJerkSmoother() = default;

  /**
   * @brief Apply jerk-filtered smoothing to a trajectory
   *
   * @param v0 Initial velocity [m/s]
   * @param a0 Initial acceleration [m/s²]
   * @param input Input trajectory points
   * @param output Output smoothed trajectory points
   * @param max_velocity_per_point Per-point velocity upper bounds [m/s].
   *        Should be initialized with max_speed_mps and updated by lateral acceleration limits.
   *        This allows enforcing lateral acceleration limits as hard constraints.
   * @return true if optimization succeeded
   * @return false if optimization failed
   */
  bool apply(
    const double v0, const double a0, const TrajectoryPoints & input, TrajectoryPoints & output,
    const std::vector<double> & max_velocity_per_point = {});

  /**
   * @brief Set smoother parameters
   * @param params New parameters
   */
  void set_params(const ContinuousJerkSmootherParams & params);

  /**
   * @brief Get current smoother parameters
   * @return Current parameters
   */
  ContinuousJerkSmootherParams get_params() const;

private:
  /**
   * @brief Calculate interval distances between trajectory points
   * @param trajectory Input trajectory
   * @return Vector of distances between consecutive points
   */
  std::vector<double> calc_trajectory_interval_distance(const TrajectoryPoints & trajectory) const;

  ContinuousJerkSmootherParams params_;
  std::shared_ptr<autoware::qp_interface::QPInterface> qp_interface_;
  rclcpp::Logger logger_{rclcpp::get_logger("continuous_jerk_smoother")};
};

}  // namespace autoware::trajectory_optimizer::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__CONTINUOUS_JERK_SMOOTHER_HPP_
