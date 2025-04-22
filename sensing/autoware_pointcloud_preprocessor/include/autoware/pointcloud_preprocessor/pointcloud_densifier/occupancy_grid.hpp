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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__POINTCLOUD_DENSIFIER__OCCUPANCY_GRID_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__POINTCLOUD_DENSIFIER__OCCUPANCY_GRID_HPP_

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <vector>

namespace autoware::pointcloud_preprocessor
{

class OccupancyGrid
{
public:
  OccupancyGrid(double x_min, double x_max, double y_min, double y_max, double resolution);
  void updateOccupancy(const sensor_msgs::msg::PointCloud2 & cloud);
  bool isOccupied(double x, double y) const;

private:
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double resolution_;
  size_t cols_;
  size_t rows_;
  std::vector<bool> grid_;

  size_t index(size_t row, size_t col) const;
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__POINTCLOUD_DENSIFIER__OCCUPANCY_GRID_HPP_
