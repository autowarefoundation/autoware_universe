// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_INTERFACE_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_INTERFACE_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params.hpp"

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::mppi_optimizer
{

using autoware_perception_msgs::msg::TrackedObjects;
using autoware_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

struct FirstOrderDubinsMppiState
{
  float x{0.0F};
  float y{0.0F};
  float yaw{0.0F};
  float vel_x{0.0F};
};

struct FirstOrderDubinsMppiControl
{
  float accel_cmd{0.0F};
  float steer_cmd{0.0F};
};

struct FirstOrderDubinsMppiRollout
{
  std::vector<std::pair<float, float>> points;
  float cost{0.0F};
};

struct FirstOrderDubinsMppiDebug
{
  Trajectory reference_trajectory;
  Trajectory optimized_trajectory;
  std::vector<std::pair<float, float>> optimal_horizon;
  std::vector<FirstOrderDubinsMppiRollout> rollouts;
  float baseline_cost{0.0F};
};

struct FirstOrderDubinsMppiOptimizationResult
{
  Trajectory trajectory;
  FirstOrderDubinsMppiDebug debug;
};

/**
 * @brief Host-side interface to the first-order Dubins MPPI controller used in the
 *        two-lane double-park path-tracking example.
 */
class FirstOrderDubinsMppiInterface
{
public:
  FirstOrderDubinsMppiInterface();
  ~FirstOrderDubinsMppiInterface();

  FirstOrderDubinsMppiInterface(const FirstOrderDubinsMppiInterface &) = delete;
  FirstOrderDubinsMppiInterface & operator=(const FirstOrderDubinsMppiInterface &) = delete;
  FirstOrderDubinsMppiInterface(FirstOrderDubinsMppiInterface &&) noexcept;
  FirstOrderDubinsMppiInterface & operator=(FirstOrderDubinsMppiInterface &&) noexcept;

  /** Initialize GPU resources and the two-lane double-park scenario. */
  void initialize();

  /** Whether initialize() completed successfully. */
  bool isInitialized() const;

  /** Configure vehicle geometry and limits from Autoware vehicle_info. */
  void setVehicleParams(const FirstOrderDubinsMppiVehicleParams & params);

  /** Configure MPPI cost weights (FirstOrderDubinsBicycleCostParams). */
  void setCostParams(const FirstOrderDubinsMppiCostParams & params);

  /**
   * @brief Run one MPPI control step and propagate the vehicle state forward.
   * @param state Current ego state (updated in place).
   * @param arc_length Current arc length along the reference path (updated in place).
   * @param sim_time Current simulation time [s].
   */
  FirstOrderDubinsMppiControl computeStep(
    FirstOrderDubinsMppiState & state, float & arc_length, float sim_time);

  /**
   * @brief Track a diffusion-planner reference (poses + velocities) with one MPPI step.
   *
   * Uses the diffusion trajectory directly as the MPPI reference horizon (x, y, yaw, v),
   * keeps warm-started controls between calls, and returns the MPPI-predicted feasible
   * state rollout that best tracks that reference.
   *
   * @param input Reference trajectory from the diffusion planner (map frame).
   * @param odometry Current ego odometry in the same frame as the trajectory.
   * @param acceleration Optional ego longitudinal acceleration [m/s^2] in base_link.
   * @param steering_status Optional ego tire steering angle [rad] from vehicle status.
   * @param tracked_objects Perception tracked objects used as dynamic obstacles
   * (constant-velocity).
   */
  FirstOrderDubinsMppiOptimizationResult optimizeTrajectory(
    const Trajectory & input, const Odometry & odometry,
    const std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> & acceleration,
    const std::optional<autoware_vehicle_msgs::msg::SteeringReport> & steering_status,
    const TrackedObjects & tracked_objects);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_INTERFACE_HPP_
