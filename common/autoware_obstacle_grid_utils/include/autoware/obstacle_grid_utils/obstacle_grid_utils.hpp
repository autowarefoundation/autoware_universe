// Copyright 2026 The Autoware Contributors
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
#ifndef AUTOWARE__OBSTACLE_GRID_UTILS__OBSTACLE_GRID_UTILS_HPP_
#define AUTOWARE__OBSTACLE_GRID_UTILS__OBSTACLE_GRID_UTILS_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <grid_map_core/grid_map_core.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace autoware::obstacle_grid_utils
{
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

/// A cell qualifies iff point_count >= min_point_count_cell && max_height >= min_height.
struct Gate
{
  std::uint32_t min_point_count_cell;
  double min_height;
};

/// Nearest qualifying cell together with WHERE it is. distance is edge-aware (to the cell
/// footprint box, identical semantics to nearest_distance); position is the cell center and
/// index its grid index, for debug markers / SafetyFactor.points / StopObstacle.nearest_point.
struct NearestCell
{
  double distance;
  grid_map::Position position;  // cell center
  grid_map::Index index;
};

/// One 8-connected component of qualifying cells; point_sum is the sum of the point_count layer
/// over its cells (the grid analog of a Euclidean cluster's point total, gated by the caller).
struct CellComponent
{
  std::vector<grid_map::Index> cells;
  double point_sum;
};

/// Whether one cell passes the gate. NaN-safe: a NaN point_count or a NaN max_height rejects.
bool cell_qualifies(
  const grid_map::GridMap & grid, const grid_map::Index & index, const Gate & gate);

/// The 4 corner points of a cell's footprint box, wound min-min, min-max, max-max, max-min.
/// Emitting corners rather than the center keeps corridor/lane membership edge-conservative:
/// a cell counts as inside if any corner is.
std::array<Point2d, 4> cell_corners(const grid_map::Position & center, double resolution);

/// 2D distance from ego_polygon to the nearest qualifying cell. Cells are areas, so the distance
/// is measured to the cell's footprint box rather than to its center. Returns +inf when nothing
/// qualifies inside the ROI (the caller treats that as "clear within ROI").
double nearest_distance(
  const grid_map::GridMap & grid, const Polygon2d & ego_polygon, const Gate & gate);

/// std::nullopt when nothing qualifies inside the ROI (the nullopt <-> +inf analog of
/// nearest_distance; a consumer treats nullopt as "clear within ROI"). On a distance tie the
/// first cell encountered by the grid iterator wins (either-of; callers must not rely on which).
std::optional<NearestCell> nearest_cell(
  const grid_map::GridMap & grid, const Polygon2d & ego_polygon, const Gate & gate);

/// 8-connected labeling over the SUPPLIED qualifying cell indices. This function does NOT gate:
/// the caller applies its per-cell Gate first and passes the surviving indices; here we only
/// group them into components and sum point_count.
///
/// Adjacency is computed in grid_map's UNWRAPPED index space, so it stays geometrically correct
/// even when the grid's circular buffer has been move()d: geometrically adjacent cells that
/// straddle the buffer wrap seam are still merged into one component, and cells that are merely
/// neighbors in raw buffer-index space across the seam are not falsely merged. The reported
/// CellComponent::cells are the caller's ORIGINAL buffer indices, so they remain usable directly
/// with grid.at(). Components come out in first-seen order, for deterministic output.
///
/// Precondition: every supplied index is a valid in-grid cell. point_count is expected non-NaN
/// (the caller's Gate already rejects NaN point_count); a NaN contribution is skipped rather than
/// poisoning the whole component's point_sum to NaN and silently dropping a real cluster.
std::vector<CellComponent> connected_components(
  const grid_map::GridMap & grid, const std::vector<grid_map::Index> & qualifying);
}  // namespace autoware::obstacle_grid_utils
#endif  // AUTOWARE__OBSTACLE_GRID_UTILS__OBSTACLE_GRID_UTILS_HPP_
