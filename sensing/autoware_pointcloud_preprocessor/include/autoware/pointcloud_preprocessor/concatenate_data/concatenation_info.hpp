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

/**
 * @brief Abstract base class for concatenation strategy configurations.
 *
 * This class provides a common interface for serializing strategy-specific
 * configuration data that can be stored in concatenated point cloud info messages.
 */
struct StrategyConfig
{
  /**
   * @brief Serialize the configuration to a byte vector.
   * @return Serialized configuration data as a byte vector.
   */
  [[nodiscard]] virtual std::vector<uint8_t> serialize() const = 0;
  virtual ~StrategyConfig() = default;
};

/**
 * @brief Configuration for advanced concatenation strategy.
 *
 * This structure contains timing constraints for the advanced concatenation strategy,
 * including minimum and maximum reference timestamps that define the time window
 * for point cloud matching and concatenation.
 */
struct StrategyAdvancedConfig : public StrategyConfig
{
  /**
   * @brief Construct configuration with timestamp constraints.
   * @param reference_timestamp_min Minimum reference timestamp for concatenation window
   * @param reference_timestamp_max Maximum reference timestamp for concatenation window
   */
  StrategyAdvancedConfig(
    const builtin_interfaces::msg::Time & reference_timestamp_min,
    const builtin_interfaces::msg::Time & reference_timestamp_max)
  : reference_timestamp_min(reference_timestamp_min),
    reference_timestamp_max(reference_timestamp_max)
  {
  }

  /**
   * @brief Construct configuration from serialized data.
   * @param serialized_data Previously serialized configuration data
   * @throws std::invalid_argument if serialized data size is invalid
   */
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

  /**
   * @brief Serialize the configuration to a byte vector.
   * @return Serialized configuration data containing timestamp constraints
   */
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

  //! Minimum reference timestamp for concatenation time window
  builtin_interfaces::msg::Time reference_timestamp_min;
  //! Maximum reference timestamp for concatenation time window
  builtin_interfaces::msg::Time reference_timestamp_max;
};

/**
 * @brief Mapping from strategy name strings to strategy enumeration values.
 *
 * Supported strategies:
 * - "naive": Basic concatenation without temporal constraints
 * - "advanced": Time-window based concatenation with temporal constraints
 */
const std::unordered_map<std::string, uint8_t> matching_strategy_name_map = {
  {"naive", autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo::STRATEGY_NAIVE},
  {"advanced", autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo::STRATEGY_ADVANCED},
};

/**
 * @brief Manages concatenation information for point cloud preprocessing.
 *
 * This class provides functionality for tracking and managing the concatenation
 * of multiple point cloud sources. It maintains information about each source
 * including their status, indexing within the concatenated result, and metadata.
 *
 * @note This class is designed for single-threaded use and should be reset
 *       between concatenation cycles using reset_and_get_base_info().
 */
class ConcatenationInfo
{
public:
  /**
   * @brief Construct concatenation info manager.
   *
   * @param matching_strategy_name Name of the matching strategy ("naive" or "advanced")
   * @param input_topics List of input topic names to track for concatenation
   * @throws std::invalid_argument if matching_strategy_name is not recognized
   */
  ConcatenationInfo(
    const std::string & matching_strategy_name, const std::vector<std::string> & input_topics)
  : concatenated_point_cloud_info_base_msg_(
      create_concatenation_info_base(matching_strategy_name, input_topics)),
    num_expected_sources_(concatenated_point_cloud_info_base_msg_.source_info.size())
  {
  }
  ~ConcatenationInfo() = default;

  /**
   * @brief Reset internal state and get base concatenation info message.
   *
   * This method should be called at the beginning of each concatenation cycle
   * to reset the valid cloud count and get a fresh base message with all
   * source topics initialized to default values.
   *
   * @return Fresh ConcatenatedPointCloudInfo message with source topics initialized
   */
  [[nodiscard]] autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo reset_and_get_base_info()
  {
    valid_cloud_count_ = 0;
    return concatenated_point_cloud_info_base_msg_;
  }

  /**
   * @brief Update source information from a point cloud message.
   *
   * Updates the source information for a specific topic with data from a point cloud.
   * This includes header information, status, and if the status is OK, calculates
   * the proper indexing (idx_begin and length) for the concatenated result.
   *
   * @param source_cloud Source point cloud message containing data and header
   * @param topic Topic name that this point cloud belongs to
   * @param status Status of this source
   * @param out_concatenated_point_cloud_info_msg Output message to update
   * @throws std::runtime_error if topic is not found or already has data
   */
  void update_source_from_point_cloud(
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

  /**
   * @brief Update source information from a header only.
   *
   * Updates the source information for a specific topic with header data only.
   * This is useful when you have timing/frame information but no actual point cloud data.
   * Note that idx_begin and length are not updated with this method.
   *
   * @param header Header message containing timestamp and frame information
   * @param topic Topic name that this header belongs to
   * @param status Status of this source
   * @param out_concatenated_point_cloud_info_msg Output message to update
   * @throws std::runtime_error if topic is not found or already has data
   */
  void update_source_from_header(
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

  /**
   * @brief Update source status only.
   *
   * Updates only the status field for a specific topic. This is useful for
   * marking sources as failed, timed out, or other status conditions without
   * modifying header or indexing information.
   *
   * @param topic Topic name to update status for
   * @param status New status value
   * @param out_concatenated_point_cloud_info_msg Output message to update
   * @throws std::runtime_error if topic is not found or already has data
   */
  void update_source_from_status(
    const std::string & topic, uint8_t status,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_point_cloud_info_msg)
  {
    auto [target_info, idx_begin] =
      find_source_info_and_next_idx(topic, out_concatenated_point_cloud_info_msg);
    target_info->status = status;
    if (status != autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK) return;
    valid_cloud_count_++;
  }

  /**
   * @brief Set final concatenation result information.
   *
   * Updates the output message with the final concatenated point cloud header
   * and determines whether the concatenation was successful based on whether
   * all expected sources provided valid data.
   *
   * @param concatenated_cloud Final concatenated point cloud result
   * @param out_concatenated_point_cloud_info_msg Output message to update with result
   */
  void set_result(
    const sensor_msgs::msg::PointCloud2 & concatenated_cloud,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_point_cloud_info_msg)
    const
  {
    out_concatenated_point_cloud_info_msg.header = concatenated_cloud.header;
    out_concatenated_point_cloud_info_msg.concatenation_success =
      valid_cloud_count_ == num_expected_sources_;
  }

  /**
   * @brief Set strategy-specific configuration data.
   *
   * Sets the serialized configuration data for the concatenation strategy.
   * This is used to store parameters specific to the chosen concatenation strategy.
   *
   * @param matching_strategy_config Serialized configuration data
   * @param out_concatenated_point_cloud_info_msg Output message to update
   */
  static void set_config(
    const std::vector<uint8_t> & matching_strategy_config,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo & out_concatenated_point_cloud_info_msg)
  {
    out_concatenated_point_cloud_info_msg.matching_strategy_config = matching_strategy_config;
  }

private:
  /**
   * @brief Helper struct for source cloud information lookup results.
   *
   * Contains a pointer to the target source info structure and the calculated
   * starting index for concatenation.
   */
  struct SourceCloudInfo
  {
    //! Pointer to the source info structure to update
    autoware_sensing_msgs::msg::SourcePointCloudInfo * target_info;
    //! Starting index in the concatenated point cloud for this source
    uint32_t idx_begin;
  };

  /**
   * @brief Find source info structure and calculate next index for concatenation.
   *
   * Locates the source info structure for the given topic and calculates the
   * appropriate starting index (idx_begin) for this source in the concatenated
   * point cloud based on previously processed sources.
   *
   * @param topic Topic name to find source info for
   * @param out_concatenated_point_cloud_info_msg Message containing source info structures
   * @return SourceCloudInfo containing pointer to target info and calculated index
   * @throws std::runtime_error if topic not found or already has data
   */
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

  /**
   * @brief Create base concatenation info message with initialized source topics.
   *
   * Creates and initializes the base concatenation info message with the specified
   * strategy and input topics. All source info structures are initialized with
   * default values and TIMEOUT status.
   *
   * @param matching_strategy_name Name of the matching strategy to use
   * @param input_topics List of input topic names to initialize
   * @return Initialized ConcatenatedPointCloudInfo message
   * @throws std::invalid_argument if strategy name is not recognized
   */
  [[nodiscard]] autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo
  create_concatenation_info_base(
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

  //! Base message template with strategy and topics initialized
  const autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo
    concatenated_point_cloud_info_base_msg_;
  //! Total number of expected source point clouds
  const size_t num_expected_sources_{0};
  //! Current count of successfully processed source clouds (status == OK)
  std::size_t valid_cloud_count_{0};
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CONCATENATION_INFO_HPP_
