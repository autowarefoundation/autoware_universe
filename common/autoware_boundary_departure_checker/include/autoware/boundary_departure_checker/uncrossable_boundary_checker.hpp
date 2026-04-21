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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARY_CHECKER_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARY_CHECKER_HPP_

#include "autoware/boundary_departure_checker/boundary_departure_evaluator.hpp"
#include "autoware/boundary_departure_checker/data_structs.hpp"
#include "autoware/boundary_departure_checker/hysteresis_logic.hpp"
#include "autoware/boundary_departure_checker/parameters.hpp"

#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>

namespace autoware::boundary_departure_checker
{
/**
 * @brief Orchestrator class for checking uncrossable boundary departure.
 */
class UncrossableBoundaryChecker
{
public:
  /**
   * @brief Constructor.
   */
  UncrossableBoundaryChecker(
    const lanelet::LaneletMapPtr & map, const UncrossableBoundaryDepartureParam & param,
    const VehicleInfo & vehicle_info);

  /**
   * @brief Update parameters for the checker.
   * @param[in] param parameters
   */
  void update_parameters(const UncrossableBoundaryDepartureParam & param);

  /**
   * @brief Update departure status along a predicted trajectory.
   * @param[in] predicted_traj predicted trajectory
   * @param[in] ego_state current ego dynamic state
   * @return departure result
   */
  DepartureResult update_departure_status(
    const TrajectoryPoints & predicted_traj, const EgoDynamicState & ego_state);

private:
  UncrossableBoundaryDepartureParam param_;
  VehicleInfo vehicle_info_;
  std::unique_ptr<BoundaryDepartureEvaluator> evaluator_ptr_;
  HysteresisState hysteresis_state_;

  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>();
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARY_CHECKER_HPP_
