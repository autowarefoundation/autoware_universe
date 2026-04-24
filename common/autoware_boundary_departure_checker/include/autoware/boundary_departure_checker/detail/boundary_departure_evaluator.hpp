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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DETAIL__BOUNDARY_DEPARTURE_EVALUATOR_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DETAIL__BOUNDARY_DEPARTURE_EVALUATOR_HPP_

#include "autoware/boundary_departure_checker/detail/data_structs.hpp"
#include "autoware/boundary_departure_checker/detail/type_alias.hpp"
#include "autoware/boundary_departure_checker/detail/uncrossable_boundaries_rtree.hpp"
#include "autoware/boundary_departure_checker/parameters.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <optional>

namespace autoware::boundary_departure_checker
{
/**
 * @brief Stateless pipeline class for evaluating boundary departures.
 */
class BoundaryDepartureEvaluator
{
public:
  /**
   * @brief Constructor.
   * @param[in] map lanelet map
   * @param[in] param checker parameters
   * @param[in] vehicle_info vehicle information
   */
  BoundaryDepartureEvaluator(
    const lanelet::LaneletMapPtr & map, const UncrossableBoundaryDepartureParam & param,
    const vehicle_info_utils::VehicleInfo & vehicle_info);

  /**
   * @brief Evaluate boundary departure along a predicted trajectory.
   * @param[in] predicted_traj predicted trajectory
   * @param[in] footprints_sides side segments of footprints along trajectory
   * @param[in] ego_state current ego dynamic state
   * @return evaluated critical point pairs if successful
   */
  [[nodiscard]] std::optional<Side<std::optional<CriticalPointPair>>> evaluate(
    const TrajectoryPoints & predicted_traj, const FootprintSideSegmentsArray & footprints_sides,
    const EgoDynamicState & ego_state) const;

  /**
   * @brief Update parameters.
   * @param[in] param parameters
   */
  void update_parameters(const UncrossableBoundaryDepartureParam & param) { param_ = param; }

private:
  lanelet::LaneletMapPtr map_;
  UncrossableBoundaryDepartureParam param_;
  vehicle_info_utils::VehicleInfo vehicle_info_;
  UncrossableBoundariesRTree rtree_;
};

}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DETAIL__BOUNDARY_DEPARTURE_EVALUATOR_HPP_
