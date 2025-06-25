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

#include <builtin_interfaces/msg/time.hpp>

#include <autoware_sensing_msgs/msg/concatenated_point_cloud_info.hpp>
#include <autoware_sensing_msgs/msg/source_point_cloud_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::pointcloud_preprocessor::utils
{

struct StrategyConfig
{
  [[nodiscard]] virtual std::vector<uint8_t> serialize() const = 0;
  virtual ~StrategyConfig() = default;
};

struct StrategyAdvancedConfig : public StrategyConfig
{
  StrategyAdvancedConfig(
    const builtin_interfaces::msg::Time & reference_timestamp_min,
    const builtin_interfaces::msg::Time & reference_timestamp_max)
  : reference_timestamp_min(reference_timestamp_min),
    reference_timestamp_max(reference_timestamp_max)
  {
  }

  explicit StrategyAdvancedConfig(const std::vector<uint8_t> & serialized_data)
  {
    if (
      serialized_data.size() != sizeof(reference_timestamp_min) + sizeof(reference_timestamp_max)) {
      throw std::invalid_argument("Invalid serialized data size for StrategyAdvancedConfig");
    }

    size_t offset = 0;

    // Deserialize reference_timestamp_min
    std::memcpy(
      &reference_timestamp_min, serialized_data.data() + offset, sizeof(reference_timestamp_min));
    offset += sizeof(reference_timestamp_min);

    // Deserialize reference_timestamp_max
    std::memcpy(
      &reference_timestamp_max, serialized_data.data() + offset, sizeof(reference_timestamp_max));
  }

  [[nodiscard]] std::vector<uint8_t> serialize() const final
  {
    std::vector<uint8_t> serialized;
    serialized.reserve(sizeof(reference_timestamp_min) + sizeof(reference_timestamp_max));

    // Serialize reference_timestamp_min
    const auto * reference_timestamp_min_ptr =
      reinterpret_cast<const uint8_t *>(&reference_timestamp_min);
    serialized.insert(
      serialized.end(), reference_timestamp_min_ptr,
      reference_timestamp_min_ptr + sizeof(reference_timestamp_min));

    // Serialize reference_timestamp_max
    const auto * reference_timestamp_max_ptr =
      reinterpret_cast<const uint8_t *>(&reference_timestamp_max);
    serialized.insert(
      serialized.end(), reference_timestamp_max_ptr,
      reference_timestamp_max_ptr + sizeof(reference_timestamp_max));

    return serialized;
  }

  builtin_interfaces::msg::Time reference_timestamp_min;
  builtin_interfaces::msg::Time reference_timestamp_max;
};

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

inline void set_concatenated_point_cloud_status(
  const bool concatenation_success, const uint8_t matching_strategy,
  const std::vector<uint8_t> & matching_strategy_config,
  autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_cloud_info)
{
  out_concatenated_cloud_info.concatenation_success = concatenation_success;
  out_concatenated_cloud_info.matching_strategy = matching_strategy;
  out_concatenated_cloud_info.matching_strategy_config = matching_strategy_config;
}

}  // namespace autoware::pointcloud_preprocessor::utils

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__UTILS_HPP_
