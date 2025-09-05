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

#ifndef LANELET_ELEVATION_FILTER__GRID_PROCESSOR_HPP_
#define LANELET_ELEVATION_FILTER__GRID_PROCESSOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>

#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::compare_map_segmentation
{

struct GridCell
{
  std::vector<geometry_msgs::msg::Point> points;
  double average_height = 0.0;
  bool is_valid = false;
};

struct GridIndex
{
  int x;
  int y;

  bool operator==(const GridIndex & other) const { return x == other.x && y == other.y; }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & index) const
  {
    return std::hash<int>()(index.x) ^ (std::hash<int>()(index.y) << 1);
  }
};

class GridProcessor
{
public:
  explicit GridProcessor(double grid_resolution);

  void processLanelets(const lanelet::LaneletMapPtr & lanelet_map, int extension_count = 20);

  void processLaneletsWithCache(
    const lanelet::LaneletMapPtr & lanelet_map, double sampling_distance, int extension_size = 20,
    const std::string & cache_directory = "");

  double getElevationAtPoint(double x, double y) const;

  bool isPointValid(
    double x, double y, double z, double threshold, bool require_map_coverage) const;

  void reset();

  // Debug/Visualization methods
  std::vector<std::pair<GridIndex, GridCell>> getGridCells() const;

  std::pair<double, double> getGridBounds() const;

private:
  double grid_resolution_;
  double inv_grid_resolution_;  // Precomputed 1/grid_resolution for performance
  std::unordered_map<GridIndex, GridCell, GridIndexHash> grid_cells_;

  mutable std::unordered_map<GridIndex, double, GridIndexHash> elevation_cache_;

  // Logger for this class
  rclcpp::Logger logger_;

  GridIndex getGridIndex(double x, double y) const;

  void addPointToGrid(double x, double y, double z);

  void fillEmptyGrids(int extension_radius);

  double calculateAverageElevationFromNeighbors(const GridIndex & index) const;

  double interpolateElevationFromNeighbors(const GridIndex & index) const;

  void calculateGridStatistics();

  void processLaneletBoundary(const lanelet::Lanelet & lanelet);

  std::vector<geometry_msgs::msg::Point> samplePointsFromLineString(
    const lanelet::ConstLineString3d & linestring, double sampling_distance) const;

  // Cache-related functions
  std::string generateCacheFilename(
    const lanelet::LaneletMapPtr & lanelet_map, double sampling_distance, int extension_size,
    const std::string & cache_directory) const;

  bool loadGridFromCache(const std::string & cache_filename);

  void saveGridToCache(const std::string & cache_filename) const;
};

}  // namespace autoware::compare_map_segmentation

#endif  // LANELET_ELEVATION_FILTER__GRID_PROCESSOR_HPP_
