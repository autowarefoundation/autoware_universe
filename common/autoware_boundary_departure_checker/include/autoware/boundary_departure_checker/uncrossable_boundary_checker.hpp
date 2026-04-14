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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARY_CHECKER_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARY_CHECKER_HPP_

#include "autoware/boundary_departure_checker/data_structs.hpp"
#include "autoware/boundary_departure_checker/parameters.hpp"

#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::boundary_departure_checker
{

class FootprintManager;

/**
 * @brief Class for checking uncrossable boundary departure.
 */
class UncrossableBoundaryChecker
{
public:
  /**
   * @brief Default constructor.
   */
  UncrossableBoundaryChecker() = default;

  /**
   * @brief Set the lanelet map.
   * @param[in] lanelet_map_ptr pointer to the lanelet map
   */
  void set_lanelet_map(const lanelet::LaneletMapPtr lanelet_map_ptr);

  /**
   * @brief Initialize the checker.
   * @return success if initialization is successful, error message otherwise
   */
  tl::expected<void, std::string> initialize();

  /**
   * @brief Set parameters for the checker.
   * @param[in] param parameters
   */
  void set_param(const UncrossableBoundaryDepartureParam & param);

  /**
   * @brief Check for boundary departure along a predicted trajectory.
   * @param[in] predicted_traj predicted trajectory
   * @param[in] vehicle_info vehicle information
   * @param[in] ego_state current ego dynamic state
   * @return departure data if successful, error message otherwise
   */
  tl::expected<DepartureData, std::string> check_departure(
    const TrajectoryPoints & predicted_traj, const vehicle_info_utils::VehicleInfo & vehicle_info,
    const EgoDynamicState & ego_state);

private:
  /**
   * @brief Find closest boundary segments from R-tree.
   * @param[in] ego_ref_segment reference segment of ego vehicle
   * @param[in] ego_opposite_ref_segment opposite reference segment of ego vehicle
   * @param[in] ego_z_position z-position of ego vehicle
   * @param[in] ego_vehicle_height height of ego vehicle
   * @param[in] unique_id set of already processed segment IDs
   * @return list of closest boundary segments
   */
  [[nodiscard]] std::vector<SegmentWithIdx> find_closest_boundary_segments(
    const Segment2d & ego_ref_segment, const Segment2d & ego_opposite_ref_segment,
    const double ego_z_position, const double ego_vehicle_height,
    const std::unordered_set<IdxForRTreeSegment, IdxForRTreeSegmentHash> & unique_id) const;

  /**
   * @brief Get boundary segments along the footprints.
   * @param[in] footprints_sides side segments of footprints along trajectory
   * @param[in] trimmed_pred_trajectory trimmed predicted trajectory
   * @param[in] ego_vehicle_height height of ego vehicle
   * @return boundary segments grouped by side
   */
  [[nodiscard]] BoundarySegmentsBySide get_boundary_segments(
    const FootprintSideSegmentsArray & footprints_sides,
    const TrajectoryPoints & trimmed_pred_trajectory, const double ego_vehicle_height) const;

  /**
   * @brief Determine departure type based on evaluation.
   * @param[in] evaluated_projections evaluated critical point pairs
   * @param[in] current_time_s current time [s]
   * @return determined departure type
   */
  DepartureType determine_departure_type(
    const Side<std::optional<CriticalPointPair>> & evaluated_projections,
    const double current_time_s);

  UncrossableBoundaryDepartureParam param_;  ///< checker parameters
  lanelet::LaneletMapPtr lanelet_map_ptr_;   ///< pointer to lanelet map
  std::unique_ptr<UncrossableBoundsRTree>
    uncrossable_boundaries_rtree_ptr_;  ///< R-tree of boundary segments
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>();  ///< time keeper for performance analysis
  double last_no_critical_dpt_time_{0.0};     ///< last time no critical departure was found [s]
  double last_found_critical_dpt_time_{0.0};  ///< last time critical departure was found [s]
  Side<ProjectionsToBound> critical_departure_history_;  ///< history of critical departures
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARY_CHECKER_HPP_
