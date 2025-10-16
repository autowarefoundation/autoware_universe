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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <autoware_utils_geometry/boost_polygon_utils.hpp>

#include <lanelet2_core/primitives/Polygon.h>

namespace autoware::motion_velocity_planner::road_user_stop::utils
{

/**
 * @brief Convert a lanelet2 BasicPolygon2d to autoware_utils_geometry Polygon2d.
 * @param poly Lanelet2 BasicPolygon2d.
 * @return A Polygon2d with the same point sequence.
 */
autoware_utils_geometry::Polygon2d to_polygon_2d(const lanelet::BasicPolygon2d & poly);

/**
 * @brief Convert a lanelet2 BasicPolygon3d to autoware_utils_geometry Polygon2d.
 * @param poly Lanelet2 BasicPolygon3d.
 * @return A Polygon2d with the same point sequence (z-coordinate ignored).
 */
autoware_utils_geometry::Polygon2d to_polygon_2d(const lanelet::BasicPolygon3d & poly);

}  // namespace autoware::motion_velocity_planner::road_user_stop::utils

#endif  // UTILS_HPP_
