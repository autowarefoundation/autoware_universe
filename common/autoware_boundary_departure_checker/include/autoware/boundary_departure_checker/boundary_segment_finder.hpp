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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__BOUNDARY_SEGMENT_FINDER_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__BOUNDARY_SEGMENT_FINDER_HPP_

#include "autoware/boundary_departure_checker/data_structs.hpp"
#include "autoware/boundary_departure_checker/parameters.hpp"
#include "autoware/boundary_departure_checker/type_alias.hpp"
#include "autoware/boundary_departure_checker/uncrossable_boundaries_rtree.hpp"

#include <lanelet2_core/LaneletMap.h>

#include <unordered_set>
#include <vector>

namespace autoware::boundary_departure_checker::boundary_segment_finder
{
/**
 * @brief Calculate closest projections from ego footprint sides to road boundaries.
 * @param[in] ego_pred_traj predicted trajectory
 * @param[in] boundaries R-tree indexed boundary segments
 * @param[in] footprints_sides list of side segments for each footprint
 * @return closest projections to boundaries for both sides
 */
Side<ProjectionsToBound> get_closest_boundary_segments_from_side(
  const TrajectoryPoints & ego_pred_traj, const BoundarySegmentsBySide & boundaries,
  const FootprintSideSegmentsArray & footprints_sides);

/**
 * @brief Check if a boundary segment is closer to the reference ego side than the opposite side.
 * @param[in] boundary_segment boundary segment to check
 * @param[in] ego_side_ref_segment reference side of the ego vehicle
 * @param[in] ego_side_opposite_ref_segment opposite side of the ego vehicle
 * @return true if closer to reference side
 */
bool is_closest_to_boundary_segment(
  const autoware_utils_geometry::Segment2d & boundary_segment,
  const autoware_utils_geometry::Segment2d & ego_side_ref_segment,
  const autoware_utils_geometry::Segment2d & ego_side_opposite_ref_segment);

/**
 * @brief Check if a 3D boundary segment is vertically within the height range of the ego vehicle.
 * @param[in] boundary_segment 3D boundary segment to check
 * @param[in] ego_z_position vertical position of ego base
 * @param[in] ego_height height of ego vehicle
 * @return true if within height range
 */
bool is_segment_within_ego_height(
  const autoware_utils_geometry::Segment3d & boundary_segment, const double ego_z_position,
  const double ego_height);

/**
 * @brief Find closest boundary segments from R-tree.
 * @param[in] rtree R-tree of boundary segments
 * @param[in] lanelet_map_ptr pointer to lanelet map
 * @param[in] param checker parameters
 * @param[in] ego_ref_segment reference segment of ego vehicle
 * @param[in] ego_opposite_ref_segment opposite reference segment of ego vehicle
 * @param[in] ego_z_position z-position of ego vehicle
 * @param[in] ego_vehicle_height height of ego vehicle
 * @param[in] unique_id set of already processed segment IDs
 * @return list of closest boundary segments
 */
std::vector<SegmentWithIdx> find_closest_boundary_segments(
  const UncrossableBoundariesRTree & rtree, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const UncrossableBoundaryDepartureParam & param, const Segment2d & ego_ref_segment,
  const Segment2d & ego_opposite_ref_segment, const double ego_z_position,
  const double ego_vehicle_height,
  std::unordered_set<IdxForRTreeSegment, IdxForRTreeSegmentHash> & unique_id);

/**
 * @brief Get boundary segments along the footprints.
 * @param[in] rtree R-tree of boundary segments
 * @param[in] lanelet_map_ptr pointer to lanelet map
 * @param[in] param checker parameters
 * @param[in] footprints_sides side segments of footprints along trajectory
 * @param[in] trimmed_pred_trajectory trimmed predicted trajectory
 * @param[in] ego_vehicle_height height of ego vehicle
 * @return boundary segments grouped by side
 */
BoundarySegmentsBySide get_boundary_segments(
  const UncrossableBoundariesRTree & rtree, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const UncrossableBoundaryDepartureParam & param,
  const FootprintSideSegmentsArray & footprints_sides,
  const TrajectoryPoints & trimmed_pred_trajectory, const double ego_vehicle_height);

}  // namespace autoware::boundary_departure_checker::boundary_segment_finder

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__BOUNDARY_SEGMENT_FINDER_HPP_
