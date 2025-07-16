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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CLOUD_INFO_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CLOUD_INFO_HPP_

#include <builtin_interfaces/msg/time.hpp>

#include <autoware_sensing_msgs/msg/concatenated_point_cloud_info.hpp>
#include <autoware_sensing_msgs/msg/source_point_cloud_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
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

const std::unordered_map<std::string, uint8_t> matching_strategy_name_map = {
  {"naive", autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo::STRATEGY_NAIVE},
  {"advanced", autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo::STRATEGY_ADVANCED},
};

class CloudInfo
{
public:
  CloudInfo(
    const std::string & matching_strategy_name, const std::vector<std::string> & input_topics)
  : concat_info_base_(create_concat_info_base(matching_strategy_name, input_topics))
  {
  }
  ~CloudInfo() = default;

  [[nodiscard]] autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo get_concat_info_base() const
  {
    return concat_info_base_;
  }

  static void apply_source_with_point_cloud(
    const sensor_msgs::msg::PointCloud2 & cloud, const std::string & topic, uint8_t status,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_cloud_info)
  {
    auto [target_info, idx_begin] =
      find_source_info_and_next_idx(topic, out_concatenated_cloud_info);

    target_info->header = cloud.header;
    target_info->status = status;
    if (status != autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK) return;
    target_info->idx_begin = idx_begin;
    target_info->length = cloud.width * cloud.height;
  }

  static void apply_source_with_header(
    const std_msgs::msg::Header & header, const std::string & topic, uint8_t status,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_cloud_info)
  {
    auto [target_info, idx_begin] =
      find_source_info_and_next_idx(topic, out_concatenated_cloud_info);
    target_info->status = status;
    target_info->header = header;
  }

  static void apply_source_with_status(
    const std::string & topic, uint8_t status,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_cloud_info)
  {
    auto [target_info, idx_begin] =
      find_source_info_and_next_idx(topic, out_concatenated_cloud_info);
    target_info->status = status;
  }

  static void update_concatenated_point_cloud_header(
    const sensor_msgs::msg::PointCloud2 & cloud,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_cloud_info)
  {
    out_concatenated_cloud_info.header = cloud.header;
  }

  static void update_concatenated_point_cloud_config(
    const std::vector<uint8_t> & matching_strategy_config,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_cloud_info)
  {
    out_concatenated_cloud_info.matching_strategy_config = matching_strategy_config;
  }

  static void update_concatenated_point_cloud_success(
    bool concatenation_success,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_cloud_info)
  {
    out_concatenated_cloud_info.concatenation_success = concatenation_success;
  }

private:
  struct SourceInfoResult
  {
    autoware_sensing_msgs::msg::SourcePointCloudInfo * target_info;
    uint32_t idx_begin;
  };

  static SourceInfoResult find_source_info_and_next_idx(
    const std::string & topic,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_cloud_info)
  {
    autoware_sensing_msgs::msg::SourcePointCloudInfo * target_info = nullptr;
    uint32_t idx_begin = 0;

    for (auto & info : out_concatenated_cloud_info.source_info) {
      if (info.topic == topic) {
        target_info = &info;
      }
      uint32_t idx_begin_candidate = info.idx_begin + info.length;
      if (idx_begin_candidate > idx_begin) {
        idx_begin = idx_begin_candidate;
      }
    }

    if (!target_info) {
      throw std::runtime_error("Topic '" + topic + "' not found in ConcatenatedPointCloudInfo");
    }

    if (target_info->idx_begin != 0 || target_info->length != 0) {
      throw std::runtime_error(
        "ConcatenatedPointCloudInfo already has source info for topic '" + topic + "'");
    }

    return {target_info, idx_begin};
  }

  static autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo create_concat_info_base(
    const std::string & matching_strategy_name, const std::vector<std::string> & input_topics)
  {
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo concat_info_base;
    concat_info_base.matching_strategy = matching_strategy_name_map.at(matching_strategy_name);
    concat_info_base.source_info.reserve(input_topics.size());
    for (const auto & topic : input_topics) {
      autoware_sensing_msgs::msg::SourcePointCloudInfo info;
      info.topic = topic;
      info.status = autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_TIMEOUT;
      concat_info_base.source_info.emplace_back(std::move(info));
    }
    return concat_info_base;
  }

  const autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo concat_info_base_;
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CLOUD_INFO_HPP_
