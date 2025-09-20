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

#ifndef AUTOWARE__UNIVERSE_UTILS__GEOMETRY__EAR_CLIPPING_HPP_
#define AUTOWARE__UNIVERSE_UTILS__GEOMETRY__EAR_CLIPPING_HPP_

#include "autoware/universe_utils/geometry/alt_geometry.hpp"

#include <optional>
#include <vector>

namespace autoware::universe_utils
{
struct LinkedPoint
{
  explicit LinkedPoint(const alt::Point2d & point)
  : pt(point), steiner(false), prev_index(std::nullopt), next_index(std::nullopt)
  {
  }

  alt::Point2d pt;
  bool steiner;
  std::optional<std::size_t> prev_index;
  std::optional<std::size_t> next_index;
  [[nodiscard]] double x() const { return pt.x(); }
  [[nodiscard]] double y() const { return pt.y(); }
};

/**
 * @brief main ear slicing loop which triangulates a polygon using linked list
 * @details iterates over the linked list of polygon points, cutting off triangular ears one by one
 * handles different stages for fixing intersections and splitting if necessary
 */
void ear_clipping_linked(
  std::size_t ear_index, std::vector<std::size_t> & indices, std::vector<LinkedPoint> & points,
  const int pass = 0);
void split_ear_clipping(
  std::vector<LinkedPoint> & points, std::size_t start_index, std::vector<std::size_t> & indices);

/**
 * @brief creates a linked list from a ring of points
 * @details converts a polygon ring into a doubly linked list with optional clockwise ordering
 * @return the last index of the created linked list
 */
std::size_t linked_list(
  const alt::PointList2d & ring, const bool clockwise, std::size_t & vertices,
  std::vector<LinkedPoint> & points);

/**
 * @brief David Eberly's algorithm for finding a bridge between hole and outer polygon
 * @details connects a hole to the outer polygon by finding the closest bridge point
 * @return index of the bridge point
 */
std::size_t find_hole_bridge(
  const std::size_t hole_index, const std::size_t outer_point_index,
  const std::vector<LinkedPoint> & points);

/**
 * @brief eliminates a single hole by connecting it to the outer polygon
 * @details finds a bridge and modifies the linked list to remove the hole
 * @return the updated outer_index after the hole is eliminated
 */
std::size_t eliminate_hole(
  const std::size_t hole_index, const std::size_t outer_index, std::vector<LinkedPoint> & points);

/**
 * @brief eliminates all holes from a polygon
 * @details processes multiple holes by connecting each to the outer polygon in sequence
 * @return the updated outer_index after all holes are eliminated
 */
std::size_t eliminate_holes(
  const std::vector<alt::PointList2d> & inners, std::size_t outer_index, std::size_t & vertices,
  std::vector<LinkedPoint> & points);

/**
 * @brief triangulates a polygon into convex triangles
 * @details simplifies a concave polygon, with or without holes, into a set of triangles
 * the size of the `points` vector at the end of the perform_triangulation algorithm is described as
 * follow:
 * - `outer_points`: Number of points in the initial outer linked list.
 * - `hole_points`: Number of points in all inner polygons.
 * - `2 * n_holes`: Additional points for bridging holes, where `n_holes` is the number of holes.
 * the final size of `points` vector is: `outer_points + hole_points + 2 * n_holes`.
 * @return A vector of convex triangles representing the triangulated polygon.
 */
std::vector<alt::ConvexPolygon2d> triangulate(const alt::Polygon2d & polygon);

/**
 * @brief Boost.Geometry version of triangulate()
 * @return A vector of convex triangles representing the triangulated polygon.
 */
std::vector<Polygon2d> triangulate(const Polygon2d & polygon);
}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__EAR_CLIPPING_HPP_
