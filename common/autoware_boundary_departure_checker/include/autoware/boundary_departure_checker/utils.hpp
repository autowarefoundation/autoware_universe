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
 * @brief Assign departure type based on evaluation metrics and thresholds.
 * @param[in] metrics evaluation metrics
 * @param[in] thresholds departure check thresholds
 * @return assigned departure type
 */
DepartureType assign_departure_type(
  const ProjectionEvaluationMetrics & metrics, const DepartureCheckThresholds & thresholds);

/**
 * @brief Check if a line string matches one of the uncrossable boundary types.
 * @param[in] boundary_types_to_detect list of boundary type strings to match
 * @param[in] ls lanelet line string to inspect
 * @return true if the line string is uncrossable
 */
bool is_uncrossable_type(
  const std::vector<std::string> & boundary_types_to_detect, const lanelet::ConstLineString3d & ls);

/**
 * @brief Construct an R-tree of uncrossable boundary segments from the lanelet map.
 * @param[in] lanelet_map input map containing all line strings
 * @param[in] boundary_types_to_detect list of boundary type names to consider as uncrossable
 * @return R-tree of uncrossable segments
 */
UncrossableBoundsRTree build_uncrossable_boundaries_rtree(
  const lanelet::LaneletMap & lanelet_map,
  const std::vector<std::string> & boundary_types_to_detect);

/**
 * @brief Project a point onto a line segment.
 * @param[in] p point to be projected
 * @param[in] segment line segment to project onto
 * @return pair of projected point and distance if successful, error message otherwise
 */
tl::expected<std::pair<Point2d, double>, std::string> point_to_segment_projection(
  const Point2d & p, const Segment2d & segment);

/**
 * @brief Calculate the nearest projection between two segments.
 * @param[in] ego_seg ego vehicle's footprint segment
 * @param[in] lane_seg road boundary segment
 * @param[in] pose_index index of the footprint point
 * @return closest projection data if successful, error message otherwise
 */
tl::expected<ProjectionToBound, std::string> calc_nearest_projection(
  const Segment2d & ego_seg, const Segment2d & lane_seg, const size_t pose_index);

/**
 * @brief Find the nearest boundary segment to an ego side segment.
 * @param[in] ego_side_seg side segment of the ego footprint
 * @param[in] ego_rear_seg rear segment of the ego footprint
 * @param[in] curr_fp_idx index of the current footprint
 * @param[in] boundary_segments candidate boundary segments
 * @return closest projection data
 */
ProjectionToBound find_closest_segment(
  const Segment2d & ego_side_seg, const Segment2d & ego_rear_seg, const size_t curr_fp_idx,
  const std::vector<SegmentWithIdx> & boundary_segments);

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
 * @brief Calculate signed lateral distance to a boundary.
 * @param[in] boundary boundary line string
 * @param[in] reference_pose reference pose
 * @return signed lateral distance if successful
 */
std::optional<double> calc_signed_lateral_distance_to_boundary(
  const lanelet::ConstLineString3d & boundary, const Pose & reference_pose);

/**
 * @brief Retrieve a 3D line segment from the Lanelet2 map.
 * @param[in] lanelet_map_ptr pointer to the Lanelet2 map
 * @param[in] seg_id identifier for the segment
 * @return corresponding 3D segment
 */
autoware_utils_geometry::Segment3d get_segment_3d_from_id(
  const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const autoware::boundary_departure_checker::IdxForRTreeSegment & seg_id);

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
}  // namespace autoware::boundary_departure_checker::utils

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UTILS_HPP_
