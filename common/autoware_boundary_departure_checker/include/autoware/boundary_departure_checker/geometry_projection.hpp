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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__GEOMETRY_PROJECTION_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__GEOMETRY_PROJECTION_HPP_

#include "autoware/boundary_departure_checker/data_structs.hpp"
#include "autoware/boundary_departure_checker/type_alias.hpp"

#include <optional>
#include <utility>
#include <vector>

namespace autoware::boundary_departure_checker::geometry_projection
{
/**
 * @brief Project a point onto a line segment.
 * @param[in] p point to be projected
 * @param[in] segment line segment to project onto
 * @return pair of projected point and distance if successful
 */
std::optional<std::pair<Point2d, double>> point_to_segment_projection(
  const Point2d & p, const Segment2d & segment);

/**
 * @brief Calculate the nearest projection between two segments.
 * @param[in] ego_seg ego vehicle's footprint segment
 * @param[in] lane_seg road boundary segment
 * @param[in] pose_index index of the footprint point
 * @return closest projection data if successful
 */
std::optional<ProjectionToBound> calc_nearest_projection(
  const Segment2d & ego_seg, const Segment2d & lane_seg, const size_t pose_index);

/**
 * @brief Find the nearest boundary segment to an ego side segment.
 * @param[in] ego_side_seg side segment of the ego footprint
 * @param[in] ego_front_seg front segment of the ego footprint
 * @param[in] ego_rear_seg rear segment of the ego footprint
 * @param[in] curr_fp_idx index of the current footprint
 * @param[in] boundary_segments candidate boundary segments
 * @return closest projection data
 */
ProjectionToBound find_closest_segment(
  const Segment2d & ego_side_seg, const Segment2d & ego_front_seg, const Segment2d & ego_rear_seg,
  const size_t curr_fp_idx, const std::vector<SegmentWithIdx> & boundary_segments);

/**
 * @brief Calculate signed lateral distance to a boundary.
 * @param[in] boundary boundary line string
 * @param[in] reference_pose reference pose
 * @return signed lateral distance if successful
 */
std::optional<double> calc_signed_lateral_distance_to_boundary(
  const lanelet::ConstLineString3d & boundary, const Pose & reference_pose);

}  // namespace autoware::boundary_departure_checker::geometry_projection

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__GEOMETRY_PROJECTION_HPP_
