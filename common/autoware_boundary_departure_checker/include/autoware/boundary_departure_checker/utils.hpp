// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UTILS_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UTILS_HPP_

#include "autoware/boundary_departure_checker/data_structs.hpp"
#include "autoware/boundary_departure_checker/parameters.hpp"
#include "autoware/boundary_departure_checker/side_struct.hpp"

#include <geometry_msgs/msg/pose_with_covariance.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <string>
#include <vector>

namespace autoware::boundary_departure_checker::utils
{
/**
 * @brief Filter boundary projections and assign departure types based on safety margins.
 *
 * @param[in] side_value List of projections for a specific side.
 * @param[in] param Configuration parameters.
 * @param[in] min_braking_dist Minimum distance required to stop.
 * @return Filtered projections with assigned departure types.
 */
ProjectionsToBound filter_and_assign_departure_types(
  const ProjectionsToBound & side_value, const UncrossableBoundaryDepartureParam & param,
  const double min_braking_dist);

/**
 * @brief Apply a backward time/distance buffer to filtered projections.
 *
 * Helps account for reaction time and system latency by finding the earliest
 * critical point within a buffer.
 *
 * @param[in] side_value List of filtered projections.
 * @param[in] param Configuration parameters.
 * @return Optional pair of physical and buffered critical points.
 */
std::optional<CriticalPointPair> apply_backward_buffer_and_filter(
  const ProjectionsToBound & side_value, const UncrossableBoundaryDepartureParam & param);

/**
 * @brief Evaluate the severity of boundary projections for both sides.
 *
 * Performs lateral and longitudinal checks to identify potential departures.
 *
 * @param[in] projections_to_bound Projections categorized by side.
 * @param[in] param Configuration parameters.
 * @param[in] ego_state Current vehicle dynamic state.
 * @param[in] vehicle_info Static vehicle properties.
 * @return Evaluated critical point pairs for each side.
 */
Side<std::optional<CriticalPointPair>> evaluate_projections_severity(
  const Side<ProjectionsToBound> & projections_to_bound,
  const UncrossableBoundaryDepartureParam & param, const EgoDynamicState & ego_state,
  const vehicle_info_utils::VehicleInfo & vehicle_info);

/**
 * @brief Assign a departure type to a single projection based on metrics and thresholds.
 *
 * @param[in] metrics Calculated metrics for the projection (e.g., lat/lon distance).
 * @param[in] thresholds Defined limits for severity levels.
 * @return Assigned DepartureType.
 */
DepartureType assign_departure_type(
  const ProjectionEvaluationMetrics & metrics, const DepartureCheckThresholds & thresholds);

/**
 * @brief Check if a line string matches one of the uncrossable boundary types.
 *
 * @param[in] boundary_types_to_detect List of boundary type strings to match.
 * @param[in] ls The lanelet line string to inspect.
 * @return True if the line string is uncrossable.
 */
bool is_uncrossable_type(
  std::vector<std::string> boundary_types_to_detect, const lanelet::ConstLineString3d & ls);

/**
 * @brief Construct an R-tree of uncrossable boundary segments from a map.
 *
 * @param[in] lanelet_map Input map containing all line strings.
 * @param[in] boundary_types_to_detect List of boundary type names to consider.
 * @return R-tree of filtered uncrossable segments.
 */
UncrossableBoundsRTree build_uncrossable_boundaries_rtree(
  const lanelet::LaneletMap & lanelet_map,
  const std::vector<std::string> & boundary_types_to_detect);

/**
 * @brief Projects a point onto a line segment.
 *
 * @param[in] p The point to be projected.
 * @param[in] segment The line segment to project onto.
 * @return A pair containing the projected point and distance on success.
 * @retval Error message if the point lies outside the segment bounds.
 */
tl::expected<std::pair<Point2d, double>, std::string> point_to_segment_projection(
  const Point2d & p, const Segment2d & segment);

/**
 * @brief Computes the nearest projection between two segments.
 *
 * Handles both lateral projections and intersection cases.
 *
 * @param[in] ego_seg A side segment of the vehicle footprint.
 * @param[in] lane_seg A road boundary segment.
 * @param[in] pose_index Index of the corresponding footprint.
 * @return Projection data on success.
 * @retval Error message if no valid projection found.
 */
tl::expected<ProjectionToBound, std::string> calc_nearest_projection(
  const Segment2d & ego_seg, const Segment2d & lane_seg, const size_t pose_index);

/**
 * @brief Finds the nearest boundary segment to an ego side segment.
 *
 * @param[in] ego_side_seg One side of the vehicle footprint.
 * @param[in] ego_rear_seg Rear edge segment (used as fallback).
 * @param[in] curr_fp_idx Index of the current footprint.
 * @param[in] boundary_segments Candidate boundary segments.
 * @return Projection data for the closest segment.
 */
ProjectionToBound find_closest_segment(
  const Segment2d & ego_side_seg, const Segment2d & ego_rear_seg, const size_t curr_fp_idx,
  const std::vector<SegmentWithIdx> & boundary_segments);

/**
 * @brief Calculates closest projections from ego footprint sides to road boundaries.
 *
 * @param[in] ego_pred_traj Predicted ego trajectory.
 * @param[in] boundaries Spatial index of road boundaries.
 * @param[in] footprints_sides Vehicle footprint sides along the trajectory.
 * @return Closest projections for both sides.
 */
Side<ProjectionsToBound> get_closest_boundary_segments_from_side(
  const TrajectoryPoints & ego_pred_traj, const BoundarySegmentsBySide & boundaries,
  const FootprintSideSegmentsArray & footprints_sides);

/**
 * @brief Calculate signed lateral distance from a pose to a boundary line string.
 *
 * @param[in] boundary The road boundary.
 * @param[in] reference_pose The vehicle pose.
 * @return Lateral distance if calculation succeeds.
 */
std::optional<double> calc_signed_lateral_distance_to_boundary(
  const lanelet::ConstLineString3d & boundary, const Pose & reference_pose);

/**
 * @brief Retrieves a 3D line segment from the Lanelet2 map using its ID.
 *
 * @param[in] lanelet_map_ptr Pointer to the map.
 * @param[in] seg_id ID structure for the segment.
 * @return The corresponding 3D segment.
 */
autoware_utils_geometry::Segment3d get_segment_3d_from_id(
  const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const autoware::boundary_departure_checker::IdxForRTreeSegment & seg_id);

/**
 * @brief Checks if a boundary segment is closer to the reference side than the opposite side.
 *
 * @param[in] boundary_segment The segment to check.
 * @param[in] ego_side_ref_segment The reference side segment.
 * @param[in] ego_side_opposite_ref_segment The opposite side segment.
 * @return True if it is the closest boundary for this side.
 */
bool is_closest_to_boundary_segment(
  const autoware_utils_geometry::Segment2d & boundary_segment,
  const autoware_utils_geometry::Segment2d & ego_side_ref_segment,
  const autoware_utils_geometry::Segment2d & ego_side_opposite_ref_segment);

/**
 * @brief Checks if a 3D boundary segment is vertically within the vehicle height.
 *
 * @param[in] boundary_segment The 3D segment.
 * @param[in] ego_z_position Vehicle base Z position.
 * @param[in] ego_height Vehicle height.
 * @return True if vertically relevant.
 */
bool is_segment_within_ego_height(
  const autoware_utils_geometry::Segment3d & boundary_segment, const double ego_z_position,
  const double ego_height);

/**
 * @brief Check if any evaluated projection is critical.
 *
 * @param[in] evaluated_projections Severity results for both sides.
 * @return True if critical departure detected on any side.
 */
bool is_critical(const Side<std::optional<CriticalPointPair>> & evaluated_projections);

/**
 * @brief Calculate the minimum distance required to stop the vehicle.
 *
 * @param[in] ego_state Current dynamic state.
 * @param[in] param Configuration parameters.
 * @param[in] vehicle_info Static vehicle properties.
 * @return Calculated braking distance [m].
 */
double calc_minimum_braking_distance(
  const EgoDynamicState & ego_state, const UncrossableBoundaryDepartureParam & param,
  const vehicle_info_utils::VehicleInfo & vehicle_info);
}  // namespace autoware::boundary_departure_checker::utils

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UTILS_HPP_
