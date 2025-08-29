// Copyright 2025 Tier IV, Inc.
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

#include "grid_processor.hpp"

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware::compare_map_segmentation
{

GridProcessor::GridProcessor(double grid_resolution)
: grid_resolution_(grid_resolution), inv_grid_resolution_(1.0 / grid_resolution)
{
}

void GridProcessor::processLanelets(const lanelet::LaneletMapPtr & lanelet_map)
{
  reset();

  if (!lanelet_map) {
    return;
  }

  // Process all lanelets in the map
  for (const auto & lanelet : lanelet_map->laneletLayer) {
    processLaneletBoundary(lanelet);
  }

  calculateGridStatistics();
}

GridIndex GridProcessor::getGridIndex(double x, double y) const
{
  GridIndex index;
  index.x = static_cast<int>(x * inv_grid_resolution_);
  index.y = static_cast<int>(y * inv_grid_resolution_);

  // Handle negative coordinates correctly
  if (x < 0.0 && (x * inv_grid_resolution_) != index.x) index.x--;
  if (y < 0.0 && (y * inv_grid_resolution_) != index.y) index.y--;

  return index;
}

void GridProcessor::addPointToGrid(double x, double y, double z)
{
  GridIndex index = getGridIndex(x, y);

  auto it = grid_cells_.find(index);
  if (it == grid_cells_.end()) {
    grid_cells_[index] = GridCell();
  }

  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;

  grid_cells_[index].points.push_back(point);
}

void GridProcessor::processLaneletBoundary(const lanelet::Lanelet & lanelet)
{
  double sampling_distance = grid_resolution_ * 0.5;  // Sample at half grid resolution

  // Sample points from left boundary
  auto left_points = samplePointsFromLineString(lanelet.leftBound(), sampling_distance);
  for (const auto & point : left_points) {
    addPointToGrid(point.x, point.y, point.z);
  }

  // Sample points from right boundary
  auto right_points = samplePointsFromLineString(lanelet.rightBound(), sampling_distance);
  for (const auto & point : right_points) {
    addPointToGrid(point.x, point.y, point.z);
  }

  // Sample points from centerline if available
  if (lanelet.hasAttribute(lanelet::AttributeName::Subtype)) {
    auto centerline = lanelet.centerline();
    auto center_points = samplePointsFromLineString(centerline, sampling_distance);
    for (const auto & point : center_points) {
      addPointToGrid(point.x, point.y, point.z);
    }
  }
}

std::vector<geometry_msgs::msg::Point> GridProcessor::samplePointsFromLineString(
  const lanelet::ConstLineString3d & linestring, double sampling_distance) const
{
  std::vector<geometry_msgs::msg::Point> sampled_points;

  if (linestring.size() < 2) {
    return sampled_points;
  }

  for (size_t i = 0; i < linestring.size() - 1; ++i) {
    const auto & start_point = linestring[i];
    const auto & end_point = linestring[i + 1];

    double dx = end_point.x() - start_point.x();
    double dy = end_point.y() - start_point.y();
    double dz = end_point.z() - start_point.z();
    double segment_length = std::sqrt(dx * dx + dy * dy);

    if (segment_length < 1e-6) {
      continue;
    }

    int num_samples = static_cast<int>(segment_length / sampling_distance) + 1;

    for (int j = 0; j <= num_samples; ++j) {
      double ratio = static_cast<double>(j) / num_samples;

      geometry_msgs::msg::Point point;
      point.x = start_point.x() + ratio * dx;
      point.y = start_point.y() + ratio * dy;
      point.z = start_point.z() + ratio * dz;

      sampled_points.push_back(point);
    }
  }

  return sampled_points;
}

void GridProcessor::calculateGridStatistics()
{
  for (auto & [index, cell] : grid_cells_) {
    if (cell.points.empty()) {
      continue;
    }

    // Calculate average height in single pass
    double sum_z = 0.0;
    const size_t num_points = cell.points.size();

    for (size_t i = 0; i < num_points; ++i) {
      sum_z += cell.points[i].z;
    }

    cell.average_height = sum_z / num_points;
    cell.is_valid = true;

    // Clear points vector to save memory after statistics calculation
    cell.points.clear();
    cell.points.shrink_to_fit();
  }
}

double GridProcessor::getElevationAtPoint(double x, double y) const
{
  GridIndex index = getGridIndex(x, y);

  // Check cache first
  auto cache_it = elevation_cache_.find(index);
  if (cache_it != elevation_cache_.end()) {
    return cache_it->second;
  }

  auto it = grid_cells_.find(index);
  if (it != grid_cells_.end() && it->second.is_valid) {
    // Cache the direct result
    double elevation = it->second.average_height;
    elevation_cache_[index] = elevation;
    return elevation;
  }

  double min_distance = std::numeric_limits<double>::max();
  double nearest_elevation = 0.0;
  bool found_neighbor = false;

  // Check immediate neighbors only (3x3 grid)
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;

      GridIndex neighbor_index;
      neighbor_index.x = index.x + dx;
      neighbor_index.y = index.y + dy;

      auto neighbor_it = grid_cells_.find(neighbor_index);
      if (neighbor_it != grid_cells_.end() && neighbor_it->second.is_valid) {
        double distance = std::abs(dx) + std::abs(dy);
        if (distance < min_distance) {
          min_distance = distance;
          nearest_elevation = neighbor_it->second.average_height;
          found_neighbor = true;
        }
      }
    }
  }

  double result = found_neighbor ? nearest_elevation : 0.0;
  elevation_cache_[index] = result;
  return result;
}

bool GridProcessor::isPointValid(double x, double y, double z, double threshold) const
{
  double expected_elevation = getElevationAtPoint(x, y);
  double height_difference = std::abs(z - expected_elevation);
  return height_difference <= threshold;
}

void GridProcessor::reset()
{
  grid_cells_.clear();
  elevation_cache_.clear();
}

std::vector<std::pair<GridIndex, GridCell>> GridProcessor::getGridCells() const
{
  std::vector<std::pair<GridIndex, GridCell>> result;
  result.reserve(grid_cells_.size());

  for (const auto & [index, cell] : grid_cells_) {
    if (cell.is_valid) {
      result.emplace_back(index, cell);
    }
  }

  return result;
}

std::pair<double, double> GridProcessor::getGridBounds() const
{
  if (grid_cells_.empty()) {
    return {0.0, 0.0};
  }

  double min_elevation = std::numeric_limits<double>::max();
  double max_elevation = std::numeric_limits<double>::lowest();

  for (const auto & [index, cell] : grid_cells_) {
    if (cell.is_valid) {
      min_elevation = std::min(min_elevation, cell.average_height);
      max_elevation = std::max(max_elevation, cell.average_height);
    }
  }

  return {min_elevation, max_elevation};
}

}  // namespace autoware::compare_map_segmentation
