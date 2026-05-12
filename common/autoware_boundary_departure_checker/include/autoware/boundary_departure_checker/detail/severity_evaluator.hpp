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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DETAIL__SEVERITY_EVALUATOR_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DETAIL__SEVERITY_EVALUATOR_HPP_

#include "autoware/boundary_departure_checker/detail/data_structs.hpp"
#include "autoware/boundary_departure_checker/parameters.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <optional>

namespace autoware::boundary_departure_checker::severity_evaluator
{
/**
 * @brief Filter projections and assign departure types based on parameters.
 * @param[in] side_value list of projections for a side
 * @param[in] param checker parameters
 * @param[in] min_braking_dist minimum braking distance [m]
 * @return filtered projections with assigned types
 */
ProjectionsToBound filter_and_assign_departure_types(
  const ProjectionsToBound & side_value, const UncrossableBoundaryDepartureParam & param,
  const double min_braking_dist);

/**
 * @brief Apply backward buffer to projections and filter them.
 * @param[in] side_value list of projections for a side
 * @param[in] param checker parameters
 * @return evaluated critical point pair if found
 */
std::optional<CriticalPointPair> apply_backward_buffer_and_filter(
  const ProjectionsToBound & side_value, const UncrossableBoundaryDepartureParam & param);

/**
 * @brief Assign departure type based on evaluation metrics and thresholds.
 * @param[in] metrics evaluation metrics
 * @param[in] thresholds departure check thresholds
 * @return assigned departure type
 */
DepartureType assign_departure_type(
  const ProjectionEvaluationMetrics & metrics, const DepartureCheckThresholds & thresholds);

/**
 * @brief Evaluate the severity of projections for both sides.
 * @param[in] projections_to_bound projections for both sides
 * @param[in] param checker parameters
 * @param[in] ego_state current ego dynamic state
 * @param[in] vehicle_info vehicle information
 * @return evaluated critical point pairs for both sides
 */
Side<std::optional<CriticalPointPair>> evaluate_projections_severity(
  const Side<ProjectionsToBound> & projections_to_bound,
  const UncrossableBoundaryDepartureParam & param, const EgoDynamicState & ego_state,
  const vehicle_info_utils::VehicleInfo & vehicle_info);

/**
 * @brief Check if any side has a critical departure.
 * @param[in] evaluated_projections evaluated critical point pairs for both sides
 * @return true if critical
 */
bool is_critical(const Side<std::optional<CriticalPointPair>> & evaluated_projections);

/**
 * @brief Calculate minimum braking distance.
 * @param[in] ego_state current ego dynamic state
 * @param[in] param checker parameters
 * @param[in] vehicle_info vehicle information
 * @return minimum braking distance [m]
 */
double calc_minimum_braking_distance(
  const EgoDynamicState & ego_state, const UncrossableBoundaryDepartureParam & param,
  const vehicle_info_utils::VehicleInfo & vehicle_info);

}  // namespace autoware::boundary_departure_checker::severity_evaluator

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DETAIL__SEVERITY_EVALUATOR_HPP_
