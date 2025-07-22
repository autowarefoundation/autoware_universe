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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CONCATENATION_INFO_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CONCATENATION_INFO_HPP_

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

class ConcatenationInfo
{
public:
  ConcatenationInfo(
    const std::string & matching_strategy_name, const std::vector<std::string> & input_topics)
  : concatenated_point_cloud_info_base_msg_(create_concatenation_info_base(matching_strategy_name, input_topics)),
    num_expected_sources_(concatenated_point_cloud_info_base_msg_.source_info.size())
  {
  }
  ~ConcatenationInfo() = default;

  [[nodiscard]] autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo reset_and_get_base_info()
  {
    valid_cloud_count_ = 0;
    return concatenated_point_cloud_info_base_msg_;
  }

  void apply_source_with_point_cloud(
    const sensor_msgs::msg::PointCloud2 & source_cloud, const std::string & topic, uint8_t status,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_point_cloud_info_msg)
  {
    auto [target_info, idx_begin] =
      find_source_info_and_next_idx(topic, out_concatenated_point_cloud_info_msg);

    target_info->header = source_cloud.header;
    target_info->status = status;
    if (status != autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK) return;
    target_info->idx_begin = idx_begin;
    target_info->length = source_cloud.width * source_cloud.height;
    valid_cloud_count_++;
  }

  void apply_source_with_header(
    const std_msgs::msg::Header & header, const std::string & topic, uint8_t status,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_point_cloud_info_msg)
  {
    auto [target_info, idx_begin] =
      find_source_info_and_next_idx(topic, out_concatenated_point_cloud_info_msg);
    target_info->header = header;
    target_info->status = status;
    if (status != autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK) return;
    valid_cloud_count_++;
  }

  void apply_source_with_status(
    const std::string & topic, uint8_t status,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_point_cloud_info_msg)
  {
    auto [target_info, idx_begin] =
      find_source_info_and_next_idx(topic, out_concatenated_point_cloud_info_msg);
    target_info->status = status;
    if (status != autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK) return;
    valid_cloud_count_++;
  }

  void update_concatenated_point_cloud_result(
    const sensor_msgs::msg::PointCloud2 & concatenated_cloud,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_point_cloud_info_msg)
    const
  {
    out_concatenated_point_cloud_info_msg.header = concatenated_cloud.header;
    out_concatenated_point_cloud_info_msg.concatenation_success =
      valid_cloud_count_ == num_expected_sources_;
  }

  static void update_concatenated_point_cloud_config(
    const std::vector<uint8_t> & matching_strategy_config,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_point_cloud_info_msg)
  {
    out_concatenated_point_cloud_info_msg.matching_strategy_config = matching_strategy_config;
  }

private:
  struct SourceCloudInfo
  {
    autoware_sensing_msgs::msg::SourcePointCloudInfo * target_info;
    uint32_t idx_begin;
  };

  [[nodiscard]] SourceCloudInfo find_source_info_and_next_idx(
    const std::string & topic,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_point_cloud_info_msg)
    const
  {
    autoware_sensing_msgs::msg::SourcePointCloudInfo * target_info = nullptr;
    uint32_t idx_begin = 0;

    for (auto & info : out_concatenated_point_cloud_info_msg.source_info) {
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

  [[nodiscard]] autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo create_concatenation_info_base(
    const std::string & matching_strategy_name, const std::vector<std::string> & input_topics) const
  {
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo concatenated_point_cloud_info_base;
    auto strategy_it = matching_strategy_name_map.find(matching_strategy_name);
    if (strategy_it == matching_strategy_name_map.end()) {
      throw std::invalid_argument("Unknown matching strategy: '" + matching_strategy_name + "'");
    }
    concatenated_point_cloud_info_base.matching_strategy = strategy_it->second;
    concatenated_point_cloud_info_base.source_info.reserve(input_topics.size());
    for (const auto & topic : input_topics) {
      autoware_sensing_msgs::msg::SourcePointCloudInfo info;
      info.topic = topic;
      info.status = autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_TIMEOUT;
      concatenated_point_cloud_info_base.source_info.emplace_back(std::move(info));
    }
    return concatenated_point_cloud_info_base;
  }

  const autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo concatenated_point_cloud_info_base_msg_;
  const size_t num_expected_sources_{0};
  std::size_t valid_cloud_count_{0};
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CONCATENATION_INFO_HPP_
