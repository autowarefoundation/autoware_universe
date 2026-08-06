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
#ifndef AUTOWARE__OBSTACLE_GRID_EXTRACTOR__OBSTACLE_GRID_EXTRACTOR_HPP_
#define AUTOWARE__OBSTACLE_GRID_EXTRACTOR__OBSTACLE_GRID_EXTRACTOR_HPP_

#include <grid_map_core/GridMap.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autoware::obstacle_grid_extractor
{
struct ExtractorParams
{
  double roi_length_x;  // forward extent [m]
  double roi_length_y;  // lateral extent [m]
  double roi_offset_x;  // grid-center +x offset from the cloud frame origin [m] (forward bias)
  double resolution;    // cell size [m]
  float crop_z_min;
  float crop_z_max;
  float overhead_split;  // low_max_height ceiling [m]: returns above it are overhead-only
};

/// Single-pass rasterizer: one cell index per point (grid_map::getIndex ==
/// costmap_generator's fetchGridIndexFromPoint), updating max_height/min_height/
/// point_count plus low_max_height (max z among returns at or below overhead_split,
/// so consumers can reject cells whose only tall content is an overhead structure).
/// O(N). No clustering, no hull. Non-finite points are dropped. Empty cells stay
/// NaN; an empty cloud yields an all-NaN grid with the header preserved (the
/// heartbeat).
class ObstacleGridExtractor
{
public:
  explicit ObstacleGridExtractor(const ExtractorParams & params);

  /// Rasterizes @p cloud in the cloud's own frame: the ROI is interpreted in that frame and the
  /// returned grid carries the cloud's header verbatim. Supplying a cloud in the frame the ROI was
  /// configured for (`base_link` for this node) is the caller's responsibility.
  /// @p cloud must expose float32 `x`/`y`/`z` fields unless it is empty.
  grid_map_msgs::msg::GridMap extract(const sensor_msgs::msg::PointCloud2 & cloud) const;

private:
  ExtractorParams params_;
  // pre-sized {max_height, min_height, point_count, low_max_height}
  mutable grid_map::GridMap grid_;
};
}  // namespace autoware::obstacle_grid_extractor
#endif  // AUTOWARE__OBSTACLE_GRID_EXTRACTOR__OBSTACLE_GRID_EXTRACTOR_HPP_
