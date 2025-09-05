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

#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace autoware::compare_map_segmentation
{

GridProcessor::GridProcessor(double grid_resolution)
: grid_resolution_(grid_resolution),
  inv_grid_resolution_(1.0 / grid_resolution),
  logger_(rclcpp::get_logger("grid_processor"))
{
}

void GridProcessor::processLanelets(const lanelet::LaneletMapPtr & lanelet_map, int extension_count)
{
  reset();

  if (!lanelet_map) {
    return;
  }

  // Process all lanelets to generate map from original points
  for (const auto & lanelet : lanelet_map->laneletLayer) {
    processLaneletBoundary(lanelet);
  }

  // Calculate statistics for original points
  calculateGridStatistics();

  // Fill empty grids around original points with specified extension count
  fillEmptyGrids(extension_count);
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

void GridProcessor::fillEmptyGrids(int extension_radius)
{
  // Get all current valid grid cells (original points)
  std::vector<GridIndex> original_indices;
  for (const auto & [index, cell] : grid_cells_) {
    if (cell.is_valid) {
      original_indices.push_back(index);
    }
  }

  // Safety limits to prevent excessive memory usage and processing time
  const size_t MAX_GRID_CELLS = 1000000;       // 1 million cells max
  const size_t MAX_CELLS_PER_RADIUS = 100000;  // 100k cells per radius max
  const size_t MAX_TOTAL_EXTENSIONS = 500000;  // 500k total extension cells max

  size_t total_extensions_added = 0;

  // Fill grids in multiple passes, from closest to farthest
  // This allows later extensions to use previously filled cells for better interpolation
  for (int radius = 1; radius <= extension_radius; ++radius) {
    size_t cells_added_this_radius = 0;
    bool radius_limit_reached = false;

    for (const auto & center_index : original_indices) {
      if (radius_limit_reached) {
        break;  // Exit the center_index loop if radius limit was reached
      }

      // Safety check - if we have too many cells, stop processing
      if (grid_cells_.size() > MAX_GRID_CELLS) {
        RCLCPP_WARN(
          logger_, "Grid size limit reached (%zu cells), stopping extension at radius %d",
          grid_cells_.size(), radius);
        return;
      }

      // Safety check - if we've added too many extension cells in total, stop
      if (total_extensions_added > MAX_TOTAL_EXTENSIONS) {
        RCLCPP_WARN(
          logger_, "Total extension limit reached (%zu extensions), stopping at radius %d",
          total_extensions_added, radius);
        return;
      }

      // Fill cells at current radius distance
      for (int dx = -radius; dx <= radius && !radius_limit_reached; ++dx) {
        for (int dy = -radius; dy <= radius && !radius_limit_reached; ++dy) {
          // Only process cells at exactly this radius (Manhattan distance)
          if (std::abs(dx) + std::abs(dy) != radius) continue;

          GridIndex extended_index;
          extended_index.x = center_index.x + dx;
          extended_index.y = center_index.y + dy;

          // Only add if the grid cell doesn't already exist
          auto it = grid_cells_.find(extended_index);
          if (it == grid_cells_.end()) {
            grid_cells_[extended_index] = GridCell();

            // Calculate elevation as weighted average of neighboring valid cells
            double elevation = calculateAverageElevationFromNeighbors(extended_index);

            // Calculate the world coordinates for this extended grid cell
            double extended_x = extended_index.x * grid_resolution_;
            double extended_y = extended_index.y * grid_resolution_;

            geometry_msgs::msg::Point extended_point;
            extended_point.x = extended_x;
            extended_point.y = extended_y;
            extended_point.z = elevation;

            grid_cells_[extended_index].points.push_back(extended_point);
            cells_added_this_radius++;
            total_extensions_added++;

            // Safety check - if we're adding too many cells for this radius, stop this radius
            if (cells_added_this_radius > MAX_CELLS_PER_RADIUS) {
              RCLCPP_WARN(
                logger_, "Per-radius extension limit reached (%zu cells), stopping radius %d",
                cells_added_this_radius, radius);
              radius_limit_reached = true;
            }
          }
        }
      }
    }

    // After each radius, calculate statistics for the newly added cells
    // This makes them valid for the next radius iteration
    calculateGridStatistics();

    // If we didn't add any cells this radius, we can stop early
    if (cells_added_this_radius == 0) {
      break;
    }
  }
}

double GridProcessor::calculateAverageElevationFromNeighbors(const GridIndex & index) const
{
  std::vector<std::pair<double, double>> neighbor_elevations_with_distance;

  // Check 5x5 grid to find valid original cells
  for (int dx = -2; dx <= 2; ++dx) {
    for (int dy = -2; dy <= 2; ++dy) {
      if (dx == 0 && dy == 0) continue;  // Skip center point

      GridIndex neighbor_index;
      neighbor_index.x = index.x + dx;
      neighbor_index.y = index.y + dy;

      auto neighbor_it = grid_cells_.find(neighbor_index);
      if (neighbor_it != grid_cells_.end() && neighbor_it->second.is_valid) {
        // Calculate distance (Manhattan distance)
        double distance = std::abs(dx) + std::abs(dy);
        neighbor_elevations_with_distance.emplace_back(
          neighbor_it->second.average_height, distance);
      }
    }
  }

  // If we have neighbors, return weighted average based on inverse distance
  if (!neighbor_elevations_with_distance.empty()) {
    double weighted_sum = 0.0;
    double weight_sum = 0.0;

    for (const auto & [elevation, distance] : neighbor_elevations_with_distance) {
      // Use inverse distance weighting (closer points have more influence)
      double weight = 1.0 / (distance + 1.0);  // +1 to avoid division by zero
      weighted_sum += elevation * weight;
      weight_sum += weight;
    }

    return weighted_sum / weight_sum;
  }

  // If no neighbors found, return 0.0 as fallback
  return 0.0;
}

void GridProcessor::processLaneletBoundary(const lanelet::Lanelet & lanelet)
{
  double sampling_distance = grid_resolution_ * 0.5;

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

    double inv_num_samples = 1 / num_samples;
    for (int j = 0; j <= num_samples; ++j) {
      double ratio = static_cast<double>(j) * inv_num_samples;

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

  // Clear interpolation cache since grid data has been updated
  // This ensures cached interpolated values are recalculated with new data
  elevation_cache_.clear();
}

double GridProcessor::getElevationAtPoint(double x, double y) const
{
  GridIndex index = getGridIndex(x, y);

  // First check if we have a direct grid cell with valid data
  auto it = grid_cells_.find(index);
  if (it != grid_cells_.end() && it->second.is_valid) {
    return it->second.average_height;
  }

  // Check cache for interpolated values (only for cells without direct grid data)
  auto cache_it = elevation_cache_.find(index);
  if (cache_it != elevation_cache_.end()) {
    return cache_it->second;
  }

  // No direct data and not in cache - need to interpolate from neighbors
  double interpolated_elevation = interpolateElevationFromNeighbors(index);

  // Smart cache management: remove half when threshold exceeded
  constexpr size_t MAX_CACHE_SIZE = 131072;

  if (elevation_cache_.size() >= MAX_CACHE_SIZE) {
    // Remove half of the cache entries when threshold is exceeded
    size_t target_size = MAX_CACHE_SIZE / 2;
    auto it = elevation_cache_.begin();

    // Remove entries until we reach target size
    while (elevation_cache_.size() > target_size && it != elevation_cache_.end()) {
      it = elevation_cache_.erase(it);
    }
  }

  // Always add the new entry after cache cleanup
  elevation_cache_[index] = interpolated_elevation;

  return interpolated_elevation;
}

double GridProcessor::interpolateElevationFromNeighbors(const GridIndex & index) const
{
  std::vector<std::pair<double, double>> neighbor_elevations_with_weight;

  // Check immediate neighbors first (3x3 grid) with higher priority
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;

      GridIndex neighbor_index;
      neighbor_index.x = index.x + dx;
      neighbor_index.y = index.y + dy;

      auto neighbor_it = grid_cells_.find(neighbor_index);
      if (neighbor_it != grid_cells_.end() && neighbor_it->second.is_valid) {
        double distance =
          std::sqrt(dx * dx + dy * dy);          // Euclidean distance for better interpolation
        double weight = 1.0 / (distance + 0.1);  // Higher weight for closer neighbors
        neighbor_elevations_with_weight.emplace_back(neighbor_it->second.average_height, weight);
      }
    }
  }

  // If no immediate neighbors, check a larger area (5x5 grid)
  if (neighbor_elevations_with_weight.empty()) {
    for (int dx = -2; dx <= 2; ++dx) {
      for (int dy = -2; dy <= 2; ++dy) {
        if (std::abs(dx) <= 1 && std::abs(dy) <= 1) continue;  // Skip already checked cells

        GridIndex neighbor_index;
        neighbor_index.x = index.x + dx;
        neighbor_index.y = index.y + dy;

        auto neighbor_it = grid_cells_.find(neighbor_index);
        if (neighbor_it != grid_cells_.end() && neighbor_it->second.is_valid) {
          double distance = std::sqrt(dx * dx + dy * dy);
          double weight = 1.0 / (distance + 0.1);
          neighbor_elevations_with_weight.emplace_back(neighbor_it->second.average_height, weight);
        }
      }
    }
  }

  // Calculate weighted average if neighbors found
  if (!neighbor_elevations_with_weight.empty()) {
    double weighted_sum = 0.0;
    double weight_sum = 0.0;

    for (const auto & [elevation, weight] : neighbor_elevations_with_weight) {
      weighted_sum += elevation * weight;
      weight_sum += weight;
    }

    return weighted_sum / weight_sum;
  }

  // No neighbors found - return default elevation
  return 0.0;
}

bool GridProcessor::isPointValid(
  double x, double y, double z, double threshold, bool require_map_coverage) const
{
  GridIndex index = getGridIndex(x, y);

  // Check if we have a direct grid cell with valid data from the map
  auto it = grid_cells_.find(index);
  bool has_map_data = (it != grid_cells_.end() && it->second.is_valid);

  if (require_map_coverage && !has_map_data) {
    // If require_map_coverage is true and no direct map data exists, reject the point
    return false;
  }

  // Get elevation (either from direct map data or interpolated)
  double expected_elevation;
  if (has_map_data) {
    expected_elevation = it->second.average_height;
  } else {
    expected_elevation = getElevationAtPoint(x, y);
  }

  double height_difference = std::abs(z - expected_elevation);
  return height_difference <= threshold;
}

void GridProcessor::reset()
{
  grid_cells_.clear();
  elevation_cache_.clear();  // Clear interpolation cache when grid is reset
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

void GridProcessor::processLaneletsWithCache(
  const lanelet::LaneletMapPtr & lanelet_map, double sampling_distance, int extension_size,
  const std::string & cache_directory)
{
  if (!lanelet_map) {
    return;
  }

  // Generate cache filename based on map content and parameters
  std::string cache_filename =
    generateCacheFilename(lanelet_map, sampling_distance, extension_size, cache_directory);

  // Try to load from cache first
  if (loadGridFromCache(cache_filename)) {
    RCLCPP_DEBUG(logger_, "Loaded grid from cache: %s", cache_filename.c_str());
    return;
  }

  // If cache miss, generate the grid
  RCLCPP_DEBUG(logger_, "Cache miss, generating new grid...");
  reset();

  // First, process all lanelets to generate map from original points
  for (const auto & lanelet : lanelet_map->laneletLayer) {
    processLaneletBoundary(lanelet);
  }

  // Calculate statistics for original points
  calculateGridStatistics();

  // Then, fill empty grids around original points with specified extension
  fillEmptyGrids(extension_size);

  // Save to cache for future use
  saveGridToCache(cache_filename);
}

std::string GridProcessor::generateCacheFilename(
  const lanelet::LaneletMapPtr & lanelet_map, double sampling_distance, int extension_size,
  const std::string & cache_directory) const
{
  // Create a comprehensive signature based on map content and parameters
  std::stringstream signature_stream;

  // Add processing parameters first
  signature_stream << "res_" << std::fixed << std::setprecision(3) << grid_resolution_ << "_sample_"
                   << std::setprecision(3) << sampling_distance << "_ext_" << extension_size << "_";

  // Create sorted list of lanelets for consistent signature
  std::vector<lanelet::Lanelet> sorted_lanelets;
  for (const auto & lanelet : lanelet_map->laneletLayer) {
    sorted_lanelets.push_back(lanelet);
  }
  std::sort(
    sorted_lanelets.begin(), sorted_lanelets.end(),
    [](const lanelet::Lanelet & a, const lanelet::Lanelet & b) { return a.id() < b.id(); });

  // Add detailed lanelet information for robust change detection
  for (const auto & lanelet : sorted_lanelets) {
    signature_stream << "L" << lanelet.id() << "_";

    // Add left boundary points
    auto leftBound = lanelet.leftBound();
    for (size_t i = 0; i < leftBound.size();
         i += std::max(1, static_cast<int>(leftBound.size() / 10))) {
      const auto & point = leftBound[i];
      signature_stream << std::fixed << std::setprecision(2) << point.x() << "," << point.y() << ","
                       << point.z() << "_";
    }

    // Add right boundary points
    auto rightBound = lanelet.rightBound();
    for (size_t i = 0; i < rightBound.size();
         i += std::max(1, static_cast<int>(rightBound.size() / 10))) {
      const auto & point = rightBound[i];
      signature_stream << std::fixed << std::setprecision(2) << point.x() << "," << point.y() << ","
                       << point.z() << "_";
    }
  }

  // Generate hash from the complete signature
  std::hash<std::string> hasher;
  size_t content_hash = hasher(signature_stream.str());

  // Use provided cache directory or default to package data directory
  std::string cache_dir = cache_directory;
  if (cache_dir.empty()) {
    cache_dir = "/tmp/autoware_lanelet_cache/";
  }

  // Create cache directory if it doesn't exist
  std::string mkdir_command = "mkdir -p " + cache_dir;
  int result = system(mkdir_command.c_str());
  if (result != 0) {
    // Silent failure, cache will be regenerated
  }

  return cache_dir + "/lanelet_grid_" + std::to_string(content_hash) + ".cache";
}

bool GridProcessor::loadGridFromCache(const std::string & cache_filename)
{
  std::ifstream file(cache_filename, std::ios::binary);
  if (!file.is_open()) {
    return false;
  }

  try {
    grid_cells_.clear();
    elevation_cache_.clear();  // Clear interpolation cache when loading new grid data

    // Read number of grid cells
    size_t num_cells;
    file.read(reinterpret_cast<char *>(&num_cells), sizeof(num_cells));

    if (file.fail()) {
      file.close();
      return false;
    }

    // Read each grid cell
    for (size_t i = 0; i < num_cells; ++i) {
      GridIndex index;
      file.read(reinterpret_cast<char *>(&index.x), sizeof(index.x));
      file.read(reinterpret_cast<char *>(&index.y), sizeof(index.y));

      GridCell cell;
      file.read(reinterpret_cast<char *>(&cell.average_height), sizeof(cell.average_height));
      file.read(reinterpret_cast<char *>(&cell.is_valid), sizeof(cell.is_valid));

      if (file.fail()) {
        file.close();
        return false;
      }

      // We don't need to store the individual points, just the statistics
      grid_cells_[index] = cell;
    }

    file.close();
    return true;
  } catch (const std::exception & e) {
    file.close();
    return false;
  }
}

void GridProcessor::saveGridToCache(const std::string & cache_filename) const
{
  std::ofstream file(cache_filename, std::ios::binary);
  if (!file.is_open()) {
    return;
  }

  try {
    // Write number of grid cells
    size_t num_cells = grid_cells_.size();
    file.write(reinterpret_cast<const char *>(&num_cells), sizeof(num_cells));

    if (file.fail()) {
      file.close();
      return;
    }

    // Write each grid cell
    for (const auto & [index, cell] : grid_cells_) {
      file.write(reinterpret_cast<const char *>(&index.x), sizeof(index.x));
      file.write(reinterpret_cast<const char *>(&index.y), sizeof(index.y));
      file.write(reinterpret_cast<const char *>(&cell.average_height), sizeof(cell.average_height));
      file.write(reinterpret_cast<const char *>(&cell.is_valid), sizeof(cell.is_valid));

      if (file.fail()) {
        file.close();
        return;
      }
    }

    file.close();
  } catch (const std::exception & e) {
    file.close();
  }
}

}  // namespace autoware::compare_map_segmentation
