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
ProjectionsToBound filter_and_assign_departure_types(
  const ProjectionsToBound & side_value, const UncrossableBoundaryDepartureParam & param,
  const double min_braking_dist);

std::optional<CriticalPointPair> apply_backward_buffer_and_filter(
  const ProjectionsToBound & side_value, const UncrossableBoundaryDepartureParam & param);

Side<std::optional<CriticalPointPair>> evaluate_projections_severity(
  const Side<ProjectionsToBound> & projections_to_bound,
  const UncrossableBoundaryDepartureParam & param, const EgoDynamicState & ego_state,
  const vehicle_info_utils::VehicleInfo & vehicle_info);

DepartureType assign_departure_type(
  const ProjectionEvaluationMetrics & metrics, const DepartureCheckThresholds & thresholds);

/**
 * @brief Check if a line string matches one of the uncrossable boundary types.
 *
 * This function examines the `type` attribute of a lanelet line string and checks whether it
 * matches any of the specified types in `boundary_types_to_detect`.
 *
 * @param boundary_types_to_detect List of boundary type strings to match (e.g., "road_border").
 * @param ls                       The lanelet line string to inspect.
 * @return True if the line string has a matching type and is considered uncrossable, false
 * otherwise.
 */
bool is_uncrossable_type(
  std::vector<std::string> boundary_types_to_detect, const lanelet::ConstLineString3d & ls);

/**
 * @brief Construct an R-tree of uncrossable boundary segments from the given lanelet map.
 *
 * This function scans all line strings in the map and filters them based on the specified
 * boundary types (e.g., "road_border", etc.). For each matching line string,
 * it splits the geometry into individual 2D segments and wraps them with indexing information.
 *
 * Internally, it:
 * - Uses `is_uncrossable_type()` to select relevant line strings.
 * - Calls an internal helper to convert each line string into indexed segments.
 * - Builds an R-tree from all valid segments for fast nearest-neighbor queries.
 *
 * The result is a spatial index used to detect potential departure violations.
 *
 * @param lanelet_map               Input map containing all line strings.
 * @param boundary_types_to_detect  List of boundary type names to consider as uncrossable.
 * @return R-tree of uncrossable segments, ready for spatial lookup operations.
 */
UncrossableBoundsRTree build_uncrossable_boundaries_rtree(
  const lanelet::LaneletMap & lanelet_map,
  const std::vector<std::string> & boundary_types_to_detect);

/**
 * @brief Projects a point onto a line segment and returns the closest point and distance.
 *
 * This function checks if the input point lies before, on, or after the given segment.
 * If it is within the segment's bounds, it returns the projection point. If it's outside,
 * an error is returned indicating the relative position.
 *
 * @param p            The point to be projected.
 * @param segment      The line segment to project onto.
 * @return A tuple containing the original point, projection point, and the distance between them,
 *         or an error message if the point lies outside the segment.
 */
tl::expected<std::pair<Point2d, double>, std::string> point_to_segment_projection(
  const Point2d & p, const Segment2d & segment);

/**
 * @brief Computes the nearest projection between two segments, used to assess lateral distance.
 *
 * First checks whether the ego segment and lane segment intersect. If so, the intersection point
 * is returned directly. If not, the function projects the endpoints of each segment onto the other
 * and selects the projection with the smallest lateral distance.
 *
 * The result includes the projected points, source segment, distance, and index of the footprint
 * that generated the ego segment.
 *
 * @param ego_seg        One side of the ego vehicle's footprint segment.
 * @param lane_seg       A road boundary segment.
 * @param pose_index  Index of the footprint point corresponding to the ego segment.
 * @return Closest projection data or an error message if no valid projection was found.
 */
tl::expected<ProjectionToBound, std::string> calc_nearest_projection(
  const Segment2d & ego_seg, const Segment2d & lane_seg, const size_t pose_index);

/**
 * @brief Finds the nearest boundary segment to an ego side segment.
 *
 * Iterates through all boundary segments and applies segment-to-segment projection logic
 * to find the closest one. If no lateral projections are found, it falls back to checking
 * for intersections with the rear edge of the ego vehicle's footprint.
 *
 * This function is used to determine where the ego vehicle is closest to the road boundary.
 *
 * @param ego_side_seg     One side of the ego vehicle's footprint.
 * @param ego_rear_seg     Rear edge segment of the ego footprint (used as fallback).
 * @param curr_fp_idx      Index of the current footprint in the trajectory.
 * @param boundary_segments Candidate boundary segments to compare against.
 * @return Projection data containing the closest segment and related information.
 */
ProjectionToBound find_closest_segment(
  const Segment2d & ego_side_seg, const Segment2d & ego_rear_seg, const size_t curr_fp_idx,
  const std::vector<SegmentWithIdx> & boundary_segments);

/**
 * @brief Calculates closest projections from ego footprint sides to road boundaries.
 *
 * For each footprint in the ego trajectory, this function finds the nearest road boundary
 * segment for both the left and right sides of the vehicle. It considers both lateral projections
 * and rear intersection checks as a fallback if no lateral proximity is found.
 *
 * The result is organized per side, and each footprint index corresponds to a projection result.
 *
 * @param ego_pred_traj           Predicted trajectory of the ego vehicle.
 * @param boundaries                 Preprocessed R-tree indexed boundary segments.
 * @param footprints_sides List of left/right segments derived from ego footprint polygons.
 * @return Closest projections to boundaries, separated by side.
 */
Side<ProjectionsToBound> get_closest_boundary_segments_from_side(
  const TrajectoryPoints & ego_pred_traj, const BoundarySegmentsBySide & boundaries,
  const FootprintSideSegmentsArray & footprints_sides);

std::optional<double> calc_signed_lateral_distance_to_boundary(
  const lanelet::ConstLineString3d & boundary, const Pose & reference_pose);

/**
 * @brief Retrieves a 3D line segment from the Lanelet2 map.
 *
 * @param lanelet_map_ptr A pointer to the Lanelet2 map from which to retrieve the data.
 * @param seg_id An identifier struct containing the ID of the parent LineString and the start/end
 * indices of the specific segment within it.
 * @return The corresponding Segment3d defined by the start and end points.
 */
autoware_utils_geometry::Segment3d get_segment_3d_from_id(
  const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const autoware::boundary_departure_checker::IdxForRTreeSegment & seg_id);

/**
 * @brief Checks if a given boundary segment is closer to the reference ego side than the opposite
 * side.
 *
 * @param boundary_segment The boundary segment to check.
 * @param ego_side_ref_segment The reference side of the ego vehicle (e.g., the left side).
 * @param ego_side_opposite_ref_segment The opposite side of the ego vehicle (e.g., the right side).
 * @return True if the boundary is closer to or equidistant to the reference side; false otherwise.
 */
bool is_closest_to_boundary_segment(
  const autoware_utils_geometry::Segment2d & boundary_segment,
  const autoware_utils_geometry::Segment2d & ego_side_ref_segment,
  const autoware_utils_geometry::Segment2d & ego_side_opposite_ref_segment);

/**
 * @brief Checks if a 3D boundary segment is vertically within the height range of the ego vehicle.
 *
 * This helps filter out irrelevant boundaries like overpasses (too high) or underpass (too low).
 *
 * @param boundary_segment The 3D boundary segment to check.
 * @param ego_z_position The reference vertical (Z-axis) position of the ego vehicle (e.g., at its
 * base).
 * @param ego_height The total height of the ego vehicle.
 * @return True if the segment's closest vertical point is within the vehicle's height; false
 * otherwise.
 */
bool is_segment_within_ego_height(
  const autoware_utils_geometry::Segment3d & boundary_segment, const double ego_z_position,
  const double ego_height);

bool is_critical(const Side<std::optional<CriticalPointPair>> & evaluated_projections);

double calc_minimum_braking_distance(
  const EgoDynamicState & ego_state, const UncrossableBoundaryDepartureParam & param,
  const vehicle_info_utils::VehicleInfo & vehicle_info);
}  // namespace autoware::boundary_departure_checker::utils

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UTILS_HPP_
