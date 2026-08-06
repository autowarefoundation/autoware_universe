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
#include "autoware/obstacle_grid_extractor/obstacle_grid_extractor.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/time.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace autoware::obstacle_grid_extractor
{
namespace
{
constexpr char kMaxHeight[] = "max_height";
constexpr char kMinHeight[] = "min_height";
constexpr char kPointCount[] = "point_count";
constexpr char kLowMaxHeight[] = "low_max_height";
}  // namespace

ObstacleGridExtractor::ObstacleGridExtractor(const ExtractorParams & params)
: params_(params), grid_({kMaxHeight, kMinHeight, kPointCount, kLowMaxHeight})
{
  grid_.setGeometry(
    grid_map::Length(params_.roi_length_x, params_.roi_length_y), params_.resolution,
    grid_map::Position(params_.roi_offset_x, 0.0));  // rectangular ROI, biased by roi_offset_x
}

grid_map_msgs::msg::GridMap ObstacleGridExtractor::extract(
  const sensor_msgs::msg::PointCloud2 & cloud) const
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  grid_[kMaxHeight].setConstant(nan);
  grid_[kMinHeight].setConstant(nan);
  grid_[kPointCount].setConstant(nan);  // empty cell = NaN
  grid_[kLowMaxHeight].setConstant(nan);

  // Guarded: an empty cloud may carry no field descriptors at all, and PointCloud2ConstIterator
  // throws when the requested field is missing. Skipping straight to the all-NaN heartbeat keeps
  // that case on the same path as a cloud whose every point is cropped away.
  if (static_cast<uint64_t>(cloud.width) * cloud.height > 0) {
    auto & hi = grid_[kMaxHeight];
    auto & lo = grid_[kMinHeight];
    auto & cnt = grid_[kPointCount];
    auto & low_hi = grid_[kLowMaxHeight];

    sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {  // O(N) single pass
      const float x = *it_x;
      const float y = *it_y;
      const float z = *it_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;  // a NaN z would otherwise pass the crop and poison the cell's height layers
      }
      if (z < params_.crop_z_min || z > params_.crop_z_max) {
        continue;
      }
      grid_map::Index idx;
      if (!grid_.getIndex(grid_map::Position(x, y), idx)) {
        continue;  // outside ROI
      }
      float & c = cnt(idx(0), idx(1));
      if (std::isnan(c)) {
        c = 1.0f;
        hi(idx(0), idx(1)) = z;
        lo(idx(0), idx(1)) = z;
      } else {
        c += 1.0f;
        hi(idx(0), idx(1)) = std::max(hi(idx(0), idx(1)), z);
        lo(idx(0), idx(1)) = std::min(lo(idx(0), idx(1)), z);
      }
      if (z <= params_.overhead_split) {
        float & lm = low_hi(idx(0), idx(1));
        lm = std::isnan(lm) ? z : std::max(lm, z);
      }
    }
  }

  // The ROI is rasterized in the cloud's own frame, so the grid inherits that frame verbatim.
  grid_.setFrameId(cloud.header.frame_id);
  grid_.setTimestamp(rclcpp::Time(cloud.header.stamp).nanoseconds());
  grid_map_msgs::msg::GridMap msg = *grid_map::GridMapRosConverter::toMessage(grid_);
  msg.header = cloud.header;  // exact cloud frame + stamp -> all-NaN = heartbeat
  return msg;
}
}  // namespace autoware::obstacle_grid_extractor
