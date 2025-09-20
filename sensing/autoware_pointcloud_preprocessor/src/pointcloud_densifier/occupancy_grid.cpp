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

#include "autoware/pointcloud_preprocessor/pointcloud_densifier/occupancy_grid.hpp"

#include <cmath>
#include <stdexcept>

namespace autoware::pointcloud_preprocessor
{

OccupancyGrid::OccupancyGrid(
  double x_min, double x_max, double y_min, double y_max, double resolution)
: x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max), resolution_(resolution)
{
  if (resolution <= 0) {
    throw std::invalid_argument("Resolution must be positive");
  }
  if (x_min >= x_max || y_min >= y_max) {
    throw std::invalid_argument("Invalid grid dimensions");
  }

  cols_ = static_cast<size_t>(std::ceil((x_max_ - x_min_) / resolution_));
  rows_ = static_cast<size_t>(std::ceil((y_max_ - y_min_) / resolution_));

  grid_.resize(rows_ * cols_, false);
}

void OccupancyGrid::updateOccupancy(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::fill(grid_.begin(), grid_.end(), false);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    float x = *iter_x;
    float y = *iter_y;

    if (x < x_min_ || x >= x_max_ || y < y_min_ || y >= y_max_) {
      continue;
    }

    size_t col = static_cast<size_t>((x - x_min_) / resolution_);
    size_t row = static_cast<size_t>((y - y_min_) / resolution_);

    if (row < rows_ && col < cols_) {
      grid_[index(row, col)] = true;
    }
  }
}

bool OccupancyGrid::isOccupied(double x, double y) const
{
  if (x < x_min_ || x >= x_max_ || y < y_min_ || y >= y_max_) {
    return false;
  }

  size_t col = static_cast<size_t>((x - x_min_) / resolution_);
  size_t row = static_cast<size_t>((y - y_min_) / resolution_);

  if (row >= rows_ || col >= cols_) {
    return false;
  }

  return grid_[index(row, col)];
}

size_t OccupancyGrid::index(size_t row, size_t col) const
{
  return row * cols_ + col;
}

}  // namespace autoware::pointcloud_preprocessor
