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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__FOOTPRINTS_GENERATOR_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__FOOTPRINTS_GENERATOR_HPP_

#include <autoware/boundary_departure_checker/side_struct.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <vector>

namespace autoware::boundary_departure_checker::footprints
{
using Footprint = autoware_utils_geometry::LinearRing2d;
using Footprints = std::vector<Footprint>;
using autoware_planning_msgs::msg::TrajectoryPoint;

/**
 * @brief Structure representing lateral and longitudinal margins for a footprint.
 */
struct FootprintMargin
{
  // Member Variables
  /**
   * @brief Lateral margin [m].
   */
  double lat_m{1000.0};

  /**
   * @brief Longitudinal margin [m].
   */
  double lon_m{1000.0};
};

/**
 * @brief Generate vehicle footprints along a trajectory.
 *
 * Calculates the polygon representing the vehicle's spatial occupancy at each
 * trajectory point, considering vehicle dimensions and pose uncertainty.
 *
 * @param[in] trajectory_points The list of points defining the vehicle's path.
 * @param[in] vehicle_info Static dimensions and properties of the vehicle.
 * @param[in] covariance The pose covariance used to calculate safety margins.
 * @return A vector of Footprint polygons.
 */
Footprints generate(
  const std::vector<TrajectoryPoint> & trajectory_points,
  const vehicle_info_utils::VehicleInfo & vehicle_info,
  const geometry_msgs::msg::PoseWithCovariance & covariance);

/**
 * @brief Extract the left and right segments from a sequence of footprints.
 *
 * Identifies the lateral boundaries of the vehicle's swept path.
 *
 * @param[in] footprints The sequence of vehicle footprints.
 * @return A vector of Side objects, each containing the left and right segments of a footprint.
 */
std::vector<Side<autoware_utils_geometry::Segment2d>> get_sides_from_footprints(
  const Footprints & footprints);

/**
 * @brief Calculate footprint margins based on pose covariance.
 *
 * Determines the safety buffer to be added to the vehicle dimensions based on
 * the uncertainty in its current pose estimate.
 *
 * @param[in] covariance The current pose covariance.
 * @param[in] scale Optional scaling factor for the calculated margins.
 * @return A FootprintMargin structure with lateral and longitudinal values.
 */
FootprintMargin calc_margin_from_covariance(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const double scale = 0.0);

}  // namespace autoware::boundary_departure_checker::footprints

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__FOOTPRINTS_GENERATOR_HPP_
