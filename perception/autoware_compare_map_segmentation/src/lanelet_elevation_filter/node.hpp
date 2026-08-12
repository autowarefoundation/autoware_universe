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

#ifndef LANELET_ELEVATION_FILTER__NODE_HPP_
#define LANELET_ELEVATION_FILTER__NODE_HPP_

#include "lanelet_elevation_filter.hpp"

#include <autoware/pointcloud_preprocessor/filter.hpp>
#include <autoware/pointcloud_preprocessor/transform_info.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace autoware::compare_map_segmentation
{

class LaneletElevationFilterComponent : public autoware::pointcloud_preprocessor::Filter
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
  using IndicesPtr = pcl::IndicesPtr;
  using TransformInfo = autoware::pointcloud_preprocessor::TransformInfo;

public:
  explicit LaneletElevationFilterComponent(const rclcpp::NodeOptions & node_options);

private:
  void filter(
    const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
    PointCloud2 & output) override;

  void onMap(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg);

  LaneletElevationFilterParams params_;
  std::unique_ptr<LaneletElevationFilter> filter_;

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;

  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_;
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;

  void loadParameters();

  void printParameters();

  std::string resolvePackageSharePath(const std::string & path);
};

}  // namespace autoware::compare_map_segmentation

#endif  // LANELET_ELEVATION_FILTER__NODE_HPP_
