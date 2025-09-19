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

#ifndef LANELET_ELEVATION_FILTER__LANELET_ELEVATION_FILTER_HPP_
#define LANELET_ELEVATION_FILTER__LANELET_ELEVATION_FILTER_HPP_

#include "grid_processor.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>

namespace autoware::compare_map_segmentation
{

struct LaneletElevationFilterParams
{
  double grid_resolution;
  double height_threshold;
  double sampling_distance;
  std::string target_frame;
  std::string cache_directory;
  bool enable_debug;
  int extension_count;
  bool require_map_coverage;
};

class LaneletElevationFilter
{
public:
  explicit LaneletElevationFilter(const LaneletElevationFilterParams & params);

  void setLaneletMap(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & map_msg);

  visualization_msgs::msg::MarkerArray createDebugMarkers(const rclcpp::Time & stamp) const;

  std::shared_ptr<GridProcessor> getGridProcessor() const { return grid_processor_; }

private:
  LaneletElevationFilterParams params_;
  std::shared_ptr<GridProcessor> grid_processor_;
  lanelet::LaneletMapPtr lanelet_map_;
  bool map_initialized_;

  void initializeGridFromMap();
};

}  // namespace autoware::compare_map_segmentation

#endif  // LANELET_ELEVATION_FILTER__LANELET_ELEVATION_FILTER_HPP_
