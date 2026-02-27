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

#include <array>
#include <string>
#include <unordered_map>

namespace autoware::trajectory_safety_filter::plugin
{
/**
 * @brief VehicleConstraintFilter class - checks if the trajectory respects vehicle constraints
 * (e.g., max speed, max acceleration/deceleration).
 */
class VehicleConstraintFilter final : public SafetyFilterInterface
{
public:
  using result_t = tl::expected<void, std::string>;

  VehicleConstraintFilter();

  result_t is_feasible(const TrajectoryPoints & traj_points, const FilterContext & context) final;

  void set_parameters(rclcpp::Node & node) final;

  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) final;

private:
  result_t check_speed(const TrajectoryPoints & traj_points) const;
  result_t check_acceleration(const TrajectoryPoints & traj_points) const;
  result_t check_deceleration(const TrajectoryPoints & traj_points) const;
  result_t check_steering_angle(const TrajectoryPoints & traj_points) const;
  result_t check_steering_rate(const TrajectoryPoints & traj_points) const;

  using Checker = result_t (VehicleConstraintFilter::*)(const TrajectoryPoints &) const;

  inline static const std::array<Checker, 5> checkers_ = {{
    &VehicleConstraintFilter::check_speed,
    &VehicleConstraintFilter::check_acceleration,
    &VehicleConstraintFilter::check_deceleration,
    &VehicleConstraintFilter::check_steering_angle,
    &VehicleConstraintFilter::check_steering_rate,
  }};  //!< Array of checker functions

  struct Params
  {
    double max_speed = 16.7;          //!< m/s
    double max_acceleration = 5.0;    //!< m/s^2
    double max_deceleration = 5.0;    //!< m/s^2 (positive but represents deceleration)
    double max_steering_angle = 0.8;  //!< rad
    double max_steering_rate = 0.3;   //!< rad/s
  };

  Params params_;  //!< Parameters for this filter
};

// --- Helper functions for constraint checks ---

/**
 * @brief Check if the trajectory respects the maximum speed constraint.
 *
 * @param traj_points Vector of trajectory points to check
 * @param max_speed Maximum allowed speed (m/s)
 * @return Return true if the trajectory respects the speed constraint, false otherwise
 */
bool is_speed_ok(const TrajectoryPoints & traj_points, double max_speed);

/**
 * @brief Check if the trajectory respects the maximum acceleration constraint.
 *
 * @param traj_points Vector of trajectory points to check
 * @param max_acceleration Maximum allowed acceleration (m/s^2)
 * @return Return true if the trajectory respects the acceleration constraint, false otherwise
 */
bool is_acceleration_ok(const TrajectoryPoints & traj_points, double max_acceleration);

/**
 * @brief Check if the trajectory respects the maximum deceleration constraint.
 *
 * @param traj_points Vector of trajectory points to check
 * @param max_deceleration Maximum allowed deceleration (m/s^2, positive value representing
 * deceleration)
 * @return Return true if the trajectory respects the deceleration constraint, false otherwise
 */
bool is_deceleration_ok(const TrajectoryPoints & traj_points, double max_deceleration);

/**
 * @brief Check if the trajectory respects the maximum steering angle constraint.
 *
 * @param traj_points Vector of trajectory points to check
 * @param vehicle_info Vehicle information needed to calculate steering angle
 * @param max_steering_angle Maximum allowed steering angle (rad)
 * @return Return true if the trajectory respects the steering angle constraint, false otherwise
 */
bool is_steering_angle_ok(
  const TrajectoryPoints & traj_points, const VehicleInfo & vehicle_info,
  double max_steering_angle);

/**
 * @brief Check if the trajectory respects the maximum steering rate constraint.
 *
 * @param traj_points Vector of trajectory points to check
 * @param vehicle_info Vehicle information needed to calculate steering rate
 * @param max_steering_rate Maximum allowed steering rate (rad/s)
 * @return Return true if the trajectory respects the steering rate constraint, false
 * otherwise
 */
bool is_steering_rate_ok(
  const TrajectoryPoints & traj_points, const VehicleInfo & vehicle_info, double max_steering_rate);
}  // namespace autoware::trajectory_safety_filter::plugin
#endif  // AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__VEHICLE_CONSTRAINT_FILTER_HPP_
