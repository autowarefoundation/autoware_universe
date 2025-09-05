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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__STEERING_ABNORMALITY_UTILS_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__STEERING_ABNORMALITY_UTILS_HPP_

#include "autoware/boundary_departure_checker/parameters.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <tf2/utils.hpp>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <cmath>
#include <vector>

namespace autoware::boundary_departure_checker::utils::steering
{
/**
 * @brief simulate the motion of the given pose using a bicycle model
 *
 * @param pose initial pose
 * @param steering_angle [rad] initial steering angle
 * @param velocity [m/s] initial velocity
 * @param dt [s] duration to simulate
 * @param wheel_base [m] wheel base of the vehicle
 */
geometry_msgs::msg::Pose update_pose_with_bicycle_model(
  const geometry_msgs::msg::Pose & pose, const double steering_angle, const double velocity,
  const double dt, const double wheel_base);

/**
 * @brief Generate vehicle footprints along the given trajectory using some steering issues.
 *
 * @param trajectory        Predicted trajectory of the ego vehicle.
 * @param vehicle_info      Vehicle dimensions.
 * @param current_steering  Latest steering angle from the vehicle.
 * @return Footprints that adapt laterally to simulate steering influence over time.
 */
std::vector<autoware_utils_geometry::LinearRing2d> create_vehicle_footprints(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const vehicle_info_utils::VehicleInfo & vehicle_info,
  [[maybe_unused]] const autoware_vehicle_msgs::msg::SteeringReport & current_steering,
  const SteeringConfig & config);

}  // namespace autoware::boundary_departure_checker::utils::steering

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__STEERING_ABNORMALITY_UTILS_HPP_
