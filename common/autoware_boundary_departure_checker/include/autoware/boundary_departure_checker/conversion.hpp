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

#include "autoware/boundary_departure_checker/parameters.hpp"
#include "autoware/boundary_departure_checker/type_alias.hpp"

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <algorithm>
#include <string>

namespace autoware::boundary_departure_checker::utils
{
/**
 * @brief Convert an enum value to its string name.
 * @tparam enum_type The enum type.
 * @param value The enum value to convert.
 * @return The name of the enum value as a string.
 * @note Requires magic_enum. The enum type must be supported by magic_enum.
 */
template <
  typename E,
  std::enable_if_t<std::is_same_v<E, DepartureType> || std::is_same_v<E, AbnormalityType>, int> = 0>
std::string to_enum_str(const E & value, const bool to_lower_case = true)
{
  auto value_str = magic_enum::enum_name(value);
  if (to_lower_case) {
    std::transform(value_str.begin(), value_str.end(), value_str.begin(), [](unsigned char c) {
      return std::tolower(c);
    });
  }
  return value_str;
}

/**
 * @brief Convert a 2D convex hull to a Lanelet2 BasicPolygon2d.
 * @param footprint_hull A sequence of 2D points representing a closed polygon.
 * @return A Lanelet2 BasicPolygon2d with the same point sequence.
 */
lanelet::BasicPolygon2d to_basic_polygon_2d(const LinearRing2d & footprint_hull);

/**
 * @brief Convert a 2D line segment to a LineString2d.
 * @param segment A pair of 2D points representing a line segment.
 * @return A LineString2d containing the two endpoints.
 */
LineString2d to_linestring_2d(const Segment2d & segment);

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
 * @brief Convert a 2D point and a z value into a 3D ROS geometry_msgs Point.
 * @param point A 2D point.
 * @param z The z-coordinate to assign.
 * @return A ROS Point with x, y from the 2D point and the given z.
 */
Point to_geom_pt(const Point2d & point, const double z = 0.0);

/**
 * @brief Convert a lanelet2 BasicPolygon2d to boost polygon2d.
 * @param  Lanelet2 BasicPolygon2d.
 * @return A Polygon2d with the same point sequence.
 */
Polygon2d to_polygon_2d(const lanelet::BasicPolygon2d & poly);
}  // namespace autoware::boundary_departure_checker::utils
#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__CONVERSION_HPP_
