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
#include <autoware/obstacle_grid_utils/obstacle_grid_utils.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::obstacle_grid_utils
{
namespace
{
/// Footprint box of one cell. Cells are areas, so measuring a distance to this box rather than
/// to the cell center is what makes nearest_distance()/nearest_cell() edge-aware. Internal: the
/// public edge-aware surface is nearest_distance(), nearest_cell() and cell_corners().
Polygon2d cell_footprint(const grid_map::Position & center, double resolution)
{
  const double half = 0.5 * resolution;
  Polygon2d box;
  boost::geometry::append(box.outer(), Point2d(center.x() - half, center.y() - half));
  boost::geometry::append(box.outer(), Point2d(center.x() - half, center.y() + half));
  boost::geometry::append(box.outer(), Point2d(center.x() + half, center.y() + half));
  boost::geometry::append(box.outer(), Point2d(center.x() + half, center.y() - half));
  boost::geometry::correct(box);
  return box;
}

/// The 8 neighbour offsets, in row-major order.
constexpr std::array<std::pair<int, int>, 8> neighbor_offsets{
  {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}}};

/// Grows 8-connected components over a fixed set of qualifying cells.
///
/// Every index is handled in grid_map's UNWRAPPED space, so adjacency stays geometric even when
/// the circular buffer has been move()d; cells are converted back to the caller's buffer-index
/// space only when they are recorded into a component.
class ComponentBuilder
{
public:
  ComponentBuilder(const grid_map::GridMap & grid, const std::vector<grid_map::Index> & qualifying)
  : grid_(grid),
    size_(grid.getSize()),
    start_(grid.getStartIndex()),
    rows_(size_(0)),
    columns_(size_(1))
  {
    members_.reserve(qualifying.size());
    visited_.reserve(qualifying.size());
    for (const auto & buffer_index : qualifying) {
      members_.insert(key(unwrap(buffer_index)));
    }
  }

  bool visited(const grid_map::Index & buffer_index) const
  {
    return visited_.count(key(unwrap(buffer_index))) != 0;
  }

  /// Flood-fill the component reachable from `seed`, marking every cell it reaches as visited.
  CellComponent grow_from(const grid_map::Index & seed)
  {
    CellComponent component;
    component.point_sum = 0.0;
    std::vector<grid_map::Index> stack{unwrap(seed)};
    visited_.insert(key(stack.back()));
    while (!stack.empty()) {
      const grid_map::Index current = stack.back();
      stack.pop_back();
      absorb(current, component);
      for (const auto & neighbor : unvisited_members_around(current)) {
        visited_.insert(key(neighbor));
        stack.push_back(neighbor);
      }
    }
    return component;
  }

private:
  grid_map::Index unwrap(const grid_map::Index & buffer_index) const
  {
    return grid_map::getIndexFromBufferIndex(buffer_index, size_, start_);
  }

  std::int64_t key(const grid_map::Index & unwrapped) const
  {
    return static_cast<std::int64_t>(unwrapped(0)) * columns_ +
           static_cast<std::int64_t>(unwrapped(1));
  }

  bool inside_grid(const grid_map::Index & unwrapped) const
  {
    const bool row_inside = unwrapped(0) >= 0 && unwrapped(0) < rows_;
    const bool column_inside = unwrapped(1) >= 0 && unwrapped(1) < columns_;
    return row_inside && column_inside;
  }

  /// Off-grid neighbours are rejected before keying, so the linear key can never alias one of
  /// them onto a real cell in the next row.
  bool is_unvisited_member(const grid_map::Index & unwrapped) const
  {
    if (!inside_grid(unwrapped)) {
      return false;
    }
    const std::int64_t neighbor_key = key(unwrapped);
    return members_.count(neighbor_key) != 0 && visited_.count(neighbor_key) == 0;
  }

  std::vector<grid_map::Index> unvisited_members_around(const grid_map::Index & unwrapped) const
  {
    std::vector<grid_map::Index> neighbors;
    neighbors.reserve(neighbor_offsets.size());
    for (const auto & offset : neighbor_offsets) {
      const grid_map::Index neighbor(unwrapped(0) + offset.first, unwrapped(1) + offset.second);
      if (is_unvisited_member(neighbor)) {
        neighbors.push_back(neighbor);
      }
    }
    return neighbors;
  }

  /// Record one cell into the component, in the caller's buffer-index space. A NaN point_count is
  /// skipped rather than poisoning the whole sum and silently dropping a real cluster.
  void absorb(const grid_map::Index & unwrapped, CellComponent & component) const
  {
    const grid_map::Index buffer_index =
      grid_map::getBufferIndexFromIndex(unwrapped, size_, start_);
    component.cells.push_back(buffer_index);
    const float point_count = grid_.at("point_count", buffer_index);
    if (!std::isnan(point_count)) {
      component.point_sum += static_cast<double>(point_count);
    }
  }

  const grid_map::GridMap & grid_;
  grid_map::Size size_;
  grid_map::Index start_;
  int rows_;
  int columns_;
  std::unordered_set<std::int64_t> members_;
  std::unordered_set<std::int64_t> visited_;
};
}  // namespace

bool cell_qualifies(
  const grid_map::GridMap & grid, const grid_map::Index & index, const Gate & gate)
{
  const float point_count = grid.at("point_count", index);
  const bool count_passes = !std::isnan(point_count) &&
                            static_cast<std::uint32_t>(point_count) >= gate.min_point_count_cell;
  if (!count_passes) {
    return false;
  }
  const float max_height = grid.at("max_height", index);
  return !std::isnan(max_height) && static_cast<double>(max_height) >= gate.min_height;
}

std::array<Point2d, 4> cell_corners(const grid_map::Position & center, double resolution)
{
  const double half = 0.5 * resolution;
  return {
    Point2d(center.x() - half, center.y() - half), Point2d(center.x() - half, center.y() + half),
    Point2d(center.x() + half, center.y() + half), Point2d(center.x() + half, center.y() - half)};
}

double nearest_distance(
  const grid_map::GridMap & grid, const Polygon2d & ego_polygon, const Gate & gate)
{
  double nearest = std::numeric_limits<double>::infinity();
  const double resolution = grid.getResolution();
  for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
    if (!cell_qualifies(grid, *it, gate)) {
      continue;
    }
    grid_map::Position center;
    grid.getPosition(*it, center);
    nearest =
      std::min(nearest, boost::geometry::distance(ego_polygon, cell_footprint(center, resolution)));
  }
  return nearest;
}

std::optional<NearestCell> nearest_cell(
  const grid_map::GridMap & grid, const Polygon2d & ego_polygon, const Gate & gate)
{
  std::optional<NearestCell> nearest;
  const double resolution = grid.getResolution();
  for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
    if (!cell_qualifies(grid, *it, gate)) {
      continue;
    }
    grid_map::Position center;
    grid.getPosition(*it, center);
    const double distance =
      boost::geometry::distance(ego_polygon, cell_footprint(center, resolution));
    if (!nearest || distance < nearest->distance) {
      nearest = NearestCell{distance, center, *it};
    }
  }
  return nearest;
}

std::vector<CellComponent> connected_components(
  const grid_map::GridMap & grid, const std::vector<grid_map::Index> & qualifying)
{
  ComponentBuilder builder(grid, qualifying);
  std::vector<CellComponent> components;
  for (const auto & seed : qualifying) {
    if (builder.visited(seed)) {
      continue;  // already absorbed into an earlier component (or a duplicate index)
    }
    components.push_back(builder.grow_from(seed));
  }
  return components;
}
}  // namespace autoware::obstacle_grid_utils
