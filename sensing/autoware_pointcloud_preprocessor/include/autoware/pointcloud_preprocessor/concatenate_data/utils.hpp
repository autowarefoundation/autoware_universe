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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__UTILS_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__UTILS_HPP_

#include <autoware_sensing_msgs/msg/concatenated_point_cloud_info.hpp>
#include <autoware_sensing_msgs/msg/source_point_cloud_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <string>

namespace autoware::pointcloud_preprocessor::utils
{

inline void append_source_point_cloud_info(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & topic,
  autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_cloud_info)
{
  uint32_t idx_begin = out_concatenated_cloud_info.source_info.size() > 0
                         ? out_concatenated_cloud_info.source_info.back().idx_end
                         : 0;
  autoware_sensing_msgs::msg::SourcePointCloudInfo source_cloud_info;
  source_cloud_info.header = cloud.header;
  source_cloud_info.height = cloud.height;
  source_cloud_info.width = cloud.width;
  source_cloud_info.fields = cloud.fields;
  source_cloud_info.is_bigendian = cloud.is_bigendian;
  source_cloud_info.point_step = cloud.point_step;
  source_cloud_info.row_step = cloud.row_step;
  source_cloud_info.is_dense = cloud.is_dense;
  source_cloud_info.topic = topic;
  source_cloud_info.idx_begin = idx_begin;
  source_cloud_info.idx_end = idx_begin + cloud.width * cloud.height;
  out_concatenated_cloud_info.source_info.push_back(source_cloud_info);
}

inline void set_concatenated_point_cloud_info(
  const sensor_msgs::msg::PointCloud2 & cloud,
  autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_cloud_info)
{
  out_concatenated_cloud_info.header = cloud.header;
  out_concatenated_cloud_info.height = cloud.height;
  out_concatenated_cloud_info.width = cloud.width;
  out_concatenated_cloud_info.fields = cloud.fields;
  out_concatenated_cloud_info.is_bigendian = cloud.is_bigendian;
  out_concatenated_cloud_info.point_step = cloud.point_step;
  out_concatenated_cloud_info.row_step = cloud.row_step;
  out_concatenated_cloud_info.is_dense = cloud.is_dense;
}

}  // namespace autoware::pointcloud_preprocessor::utils

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__UTILS_HPP_
