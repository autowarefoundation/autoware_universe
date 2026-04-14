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
 * @brief Main class for checking vehicle departure from uncrossable boundaries.
 *
 * This class manages spatial indexing of road boundaries and performs real-time
 * checks against predicted trajectories to detect potential lane departures.
 */
class UncrossableBoundaryChecker
{
public:
  UncrossableBoundaryChecker() = default;

  // Public/API Methods
  /**
   * @brief Set the Lanelet2 map and initialize the spatial index.
   * @param[in] lanelet_map_ptr Shared pointer to the Lanelet2 map.
   */
  void set_lanelet_map(const lanelet::LaneletMapPtr lanelet_map_ptr);

  /**
   * @brief Initialize the checker, building necessary internal structures.
   * @return A tl::expected containing void on success, or a string error message.
   * @retval Error message if the map is not set or indexing fails.
   */
  tl::expected<void, std::string> initialize();

  /**
   * @brief Update the checker parameters.
   * @param[in] param The new parameters to apply.
   */
  void set_param(const UncrossableBoundaryDepartureParam & param);

  /**
   * @brief Check for boundary departures along a predicted trajectory.
   *
   * Generates ego footprints and computes closest boundary projections to
   * determine if the vehicle is approaching or has crossed a boundary.
   *
   * @param[in] predicted_traj Ego's predicted trajectory.
   * @param[in] vehicle_info Static vehicle dimensions and properties.
   * @param[in] ego_state Current dynamic state of the ego vehicle.
   * @return DepartureData containing footprints and projection results on success.
   * @retval Error message string if calculation fails.
   */
  tl::expected<DepartureData, std::string> check_departure(
    const TrajectoryPoints & predicted_traj, const vehicle_info_utils::VehicleInfo & vehicle_info,
    const EgoDynamicState & ego_state);

private:
  // Private Methods
  /**
   * @brief Find nearby uncrossable boundary segments using the R-tree.
   *
   * @param[in] ego_ref_segment Reference side segment of the ego vehicle.
   * @param[in] ego_opposite_ref_segment Opposite side segment for filtering.
   * @param[in] ego_z_position Current vertical position of the ego vehicle.
   * @param[in] ego_vehicle_height Total height of the vehicle.
   * @param[in] unique_id Set of already processed segment IDs to avoid duplicates.
   * @return Filtered vector of relevant boundary segments.
   */
  [[nodiscard]] std::vector<SegmentWithIdx> find_closest_boundary_segments(
    const Segment2d & ego_ref_segment, const Segment2d & ego_opposite_ref_segment,
    const double ego_z_position, const double ego_vehicle_height,
    const std::unordered_set<IdxForRTreeSegment, IdxForRTreeSegmentHash> & unique_id) const;

  /**
   * @brief Collect all relevant boundary segments along the predicted path.
   *
   * @param[in] footprints_sides Sides of the vehicle footprints along the trajectory.
   * @param[in] trimmed_pred_trajectory The trajectory points corresponding to the footprints.
   * @param[in] ego_vehicle_height Total height of the vehicle.
   * @return BoundarySegmentsBySide containing relevant segments for both sides.
   */
  [[nodiscard]] BoundarySegmentsBySide get_boundary_segments(
    const FootprintSideSegmentsArray & footprints_sides,
    const TrajectoryPoints & trimmed_pred_trajectory, const double ego_vehicle_height) const;

  /**
   * @brief Determine the overall departure severity based on evaluated projections.
   *
   * @param[in] evaluated_projections The result of projection evaluations for both sides.
   * @param[in] current_time_s Current timestamp for hysteresis handling.
   * @return The final DepartureType.
   */
  DepartureType determine_departure_type(
    const Side<std::optional<CriticalPointPair>> & evaluated_projections,
    const double current_time_s);

  // Member Variables
  /**
   * @brief Configuration parameters for the checker.
   */
  UncrossableBoundaryDepartureParam param_;

  /**
   * @brief Pointer to the map containing road boundaries.
   */
  lanelet::LaneletMapPtr lanelet_map_ptr_;

  /**
   * @brief Spatial index for fast lookup of boundary segments.
   */
  std::unique_ptr<UncrossableBoundsRTree> uncrossable_boundaries_rtree_ptr_;

  /**
   * @brief Shared utility for performance monitoring and debug logging.
   */
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>();

  /**
   * @brief Timestamp of the last time no critical departure was detected [s].
   */
  double last_no_critical_dpt_time_{0.0};

  /**
   * @brief Timestamp of the last time a critical departure was found [s].
   */
  double last_found_critical_dpt_time_{0.0};

  /**
   * @brief Cached projections for detected critical departures.
   */
  Side<ProjectionsToBound> critical_departure_;
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARY_CHECKER_HPP_
