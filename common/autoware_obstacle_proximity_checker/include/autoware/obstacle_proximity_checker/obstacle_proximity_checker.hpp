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

#ifndef AUTOWARE__OBSTACLE_PROXIMITY_CHECKER__OBSTACLE_PROXIMITY_CHECKER_HPP_
#define AUTOWARE__OBSTACLE_PROXIMITY_CHECKER__OBSTACLE_PROXIMITY_CHECKER_HPP_

#include "autoware/obstacle_proximity_checker/structs.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <optional>

namespace autoware::obstacle_proximity_checker
{

/// @brief class to check for nearby obstacles around the ego vehicle
class ProximityChecker
{
public:
  /**
   * @brief constructor
   * @param parameters parameters for proximity check
   * @param vehicle_info vehicle information
   */
  ProximityChecker(
    const Parameters & parameters, const vehicle_info_utils::VehicleInfo & vehicle_info);

  /**
   * @brief check for nearby obstacles
   * @param input input data for proximity check
   * @param contact_distance_threshold distance threshold below which an obstacle is considered
   * nearby [m]
   * @return result specifying if a nearby obstacle is found and the nearest obstacle
   */
  [[nodiscard]] CheckResult check(const Inputs & input, double contact_distance_threshold) const;

  /**
   * @brief update parameters
   * @param parameters new parameters
   */
  void update_parameters(const Parameters & parameters);

private:
  [[nodiscard]] bool getUseDynamicObject() const;

  [[nodiscard]] std::optional<ProximityObstacle> getNearestObstacle(const Inputs & input) const;

  [[nodiscard]] std::optional<ProximityObstacle> getNearestObstacleByPointCloud(
    const Inputs & input) const;

  [[nodiscard]] std::optional<ProximityObstacle> getNearestObstacleByDynamicObject(
    const Inputs & input) const;

  [[nodiscard]] bool isObstacleFound(
    const std::optional<ProximityObstacle> & nearest_obstacle,
    double contact_distance_threshold) const;

  Parameters parameters_;
  vehicle_info_utils::VehicleInfo vehicle_info_;
};

}  // namespace autoware::obstacle_proximity_checker

#endif  // AUTOWARE__OBSTACLE_PROXIMITY_CHECKER__OBSTACLE_PROXIMITY_CHECKER_HPP_
