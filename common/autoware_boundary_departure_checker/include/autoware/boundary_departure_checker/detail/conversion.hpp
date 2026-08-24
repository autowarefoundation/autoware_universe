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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DETAIL__CONVERSION_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DETAIL__CONVERSION_HPP_

#include "autoware/boundary_departure_checker/detail/type_alias.hpp"

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Polygon.h>

namespace autoware::boundary_departure_checker::utils
{
/**
 * @brief Convert a 3D Eigen vector to a 2D point by dropping the z-coordinate.
 * @param[in] ll_pt 3D point in Eigen format
 * @return 2D point
 */
Point2d to_point_2d(const Eigen::Matrix<double, 3, 1> & ll_pt);

/**
 * @brief Convert two 3D Eigen points to a 2D line segment.
 * @param[in] ll_pt1 first 3D point
 * @param[in] ll_pt2 second 3D point
 * @return 2D line segment
 */
Segment2d to_segment_2d(
  const Eigen::Matrix<double, 3, 1> & ll_pt1, const Eigen::Matrix<double, 3, 1> & ll_pt2);

/**
 * @brief Convert a 3D segment to a 2D segment.
 * @param[in] segment input 3D segment
 * @return 2D segment
 */
Segment2d to_segment_2d(const Segment3d & segment);
}  // namespace autoware::boundary_departure_checker::utils
#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DETAIL__CONVERSION_HPP_
