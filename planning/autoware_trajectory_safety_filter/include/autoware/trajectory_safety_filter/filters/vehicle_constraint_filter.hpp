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

#ifndef AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__VEHICLE_CONSTRAINT_FILTER_HPP_
#define AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__VEHICLE_CONSTRAINT_FILTER_HPP_

#include "autoware/trajectory_safety_filter/safety_filter_interface.hpp"

#include <string>
#include <unordered_map>

namespace autoware::trajectory_safety_filter::plugin
{
/**
 * @brief VehicleConstraintFilter class - checks if the trajectory respects vehicle constraints
 * (e.g., max velocity, max acceleration/deceleration).
 */
class VehicleConstraintFilter final : public SafetyFilterInterface
{
public:
  /**
   * @brief Params struct - holds parameters for the VehicleConstraintFilter
   *
   * max_velocity: Maximum allowed velocity (m/s)
   * max_acceleration: Maximum allowed acceleration (m/s^2)
   * max_deceleration: Maximum allowed deceleration (m/s^2, positive value representing
   * deceleration)
   * max_steering_angle: Maximum allowed steering angle (rad)
   * max_steering_angle_rate: Maximum allowed rate of change of steering angle (rad/s)
   */
  struct Params
  {
    double max_velocity = 10.0;            //!< m/s
    double max_acceleration = 2.0;         //!< m/s^2
    double max_deceleration = 2.0;         //!< m/s^2 (positive value, but represents deceleration)
    double max_steering_angle = 0.5;       //!< rad
    double max_steering_angle_rate = 0.1;  //!< rad/s
  };

  VehicleConstraintFilter();

  tl::expected<void, std::string> is_feasible(
    const TrajectoryPoints & traj_points, const FilterContext & context) final;

  void set_parameters(const std::unordered_map<std::string, std::any> & params) override;

private:
  Params params_;  //!< Parameters for this filter
};

// --- Helper functions for constraint checks ---

/**
 * @brief Check if the trajectory respects the maximum velocity constraint.
 *
 * @param traj_points Vector of trajectory points to check
 * @param max_velocity Maximum allowed velocity (m/s)
 * @return Return true if the trajectory respects the velocity constraint, false otherwise
 */
bool check_velocity(const TrajectoryPoints & traj_points, double max_velocity);

/**
 * @brief Check if the trajectory respects the maximum acceleration constraint.
 *
 * @param traj_points Vector of trajectory points to check
 * @param max_acceleration Maximum allowed acceleration (m/s^2)
 * @return Return true if the trajectory respects the acceleration constraint, false otherwise
 */
bool check_acceleration(const TrajectoryPoints & traj_points, double max_acceleration);

/**
 * @brief Check if the trajectory respects the maximum deceleration constraint.
 *
 * @param traj_points Vector of trajectory points to check
 * @param max_deceleration Maximum allowed deceleration (m/s^2, positive value representing
 * deceleration)
 * @return Return true if the trajectory respects the deceleration constraint, false otherwise
 */
bool check_deceleration(const TrajectoryPoints & traj_points, double max_deceleration);

/**
 * @brief Check if the trajectory respects the maximum steering angle constraint.
 *
 * @param traj_points Vector of trajectory points to check
 * @param vehicle_info Vehicle information needed to calculate steering angle
 * @param max_steering_angle Maximum allowed steering angle (rad)
 * @return Return true if the trajectory respects the steering angle constraint, false otherwise
 */
bool check_steering_angle(
  const TrajectoryPoints & traj_points, const VehicleInfo & vehicle_info,
  double max_steering_angle);

/**
 * @brief Check if the trajectory respects the maximum steering angle rate constraint.
 *
 * @param traj_points Vector of trajectory points to check
 * @param vehicle_info Vehicle information needed to calculate steering angle rate
 * @param max_steering_angle_rate Maximum allowed rate of change of steering angle (rad/s)
 * @return Return true if the trajectory respects the steering angle rate constraint, false
 * otherwise
 */
bool check_steering_angle_rate(
  const TrajectoryPoints & traj_points, const VehicleInfo & vehicle_info,
  double max_steering_angle_rate);
}  // namespace autoware::trajectory_safety_filter::plugin
#endif  // AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__VEHICLE_CONSTRAINT_FILTER_HPP_
