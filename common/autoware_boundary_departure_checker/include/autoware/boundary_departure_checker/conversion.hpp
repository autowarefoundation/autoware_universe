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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__CONVERSION_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__CONVERSION_HPP_

#include "autoware/boundary_departure_checker/type_alias.hpp"

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Polygon.h>

namespace autoware::boundary_departure_checker::utils
{
/**
 * @brief Convert a 3D Eigen vector to a 2D point by dropping the z-coordinate.
 * @param ll_pt A 3D point in Eigen format.
 * @return A 2D point using the x and y values.
 */
Point2d to_point_2d(const Eigen::Matrix<double, 3, 1> & ll_pt);

/**
 * @brief Convert two 3D Eigen points to a 2D line segment.
 * @param ll_pt1 First 3D point.
 * @param ll_pt2 Second 3D point.
 * @return A 2D line segment made from the projected 2D points.
 */
Segment2d to_segment_2d(
  const Eigen::Matrix<double, 3, 1> & ll_pt1, const Eigen::Matrix<double, 3, 1> & ll_pt2);

/**
 * @brief Converts a 3D segment into its 2D representation by discarding the Z-coordinates.
 *
 * @param segment The input 3D segment to convert.
 * @return A new 2D segment with the Z-coordinates of the original segment's endpoints removed.
 */
Segment2d to_segment_2d(const Segment3d & segment);

/**
 * @brief Convert a 2D point and a z value into a 3D ROS geometry_msgs Point.
 * @param point A 2D point.
 * @param z The z-coordinate to assign.
 * @return A ROS Point with x, y from the 2D point and the given z.
 */
Point to_geom_pt(const Point2d & point, const double z = 0.0);
}  // namespace autoware::boundary_departure_checker::utils
#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__CONVERSION_HPP_
