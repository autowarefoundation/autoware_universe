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

#pragma once

#include "collector_info.hpp"
#include "combine_cloud_handler_base.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ROS includes
#include "autoware/point_types/types.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autoware::pointcloud_preprocessor
{
using autoware::point_types::PointXYZIRC;
using point_cloud_msg_wrapper::PointCloud2Modifier;

template <typename PointCloudMsgT>
class CombineCloudHandler;

template <>
class CombineCloudHandler<sensor_msgs::msg::PointCloud2> : public CombineCloudHandlerBase
{
public:
  CombineCloudHandler(
    const std::vector<std::string> & input_topics, std::string output_frame,
    bool is_motion_compensated, bool publish_synchronized_pointcloud,
    bool keep_input_frame_in_synchronized_pointcloud, MatchingStrategyType matching_strategy)
  : CombineCloudHandlerBase(
      input_topics, output_frame, is_motion_compensated, publish_synchronized_pointcloud,
      keep_input_frame_in_synchronized_pointcloud, matching_strategy)
  {
  }

  ~CombineCloudHandler() override = default;

  ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> combine_pointclouds(
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> &
      topic_to_cloud_map,
    const std::shared_ptr<CollectorInfoBase> & collector_info);

  void allocate_pointclouds() override {};

protected:
  /// @brief TimeHash defines a custom hash function for builtin_interfaces::msg::Time by using
  /// its nanoseconds representation as the hash value.
  struct TimeHash
  {
    std::size_t operator()(const builtin_interfaces::msg::Time & t) const
    {
      return std::hash<int64_t>()(static_cast<int64_t>(t.sec) * 1'000'000'000LL + t.nanosec);
    }
  };

  static void convert_to_xyzirc_cloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_cloud,
    sensor_msgs::msg::PointCloud2::UniquePtr & xyzirc_cloud);

  /// @brief Transform `in` into `out` with a 4x4 matrix (replaces pcl_ros::transformPointCloud).
  static void transform_pointcloud(
    const Eigen::Matrix4f & transform, const sensor_msgs::msg::PointCloud2 & in,
    sensor_msgs::msg::PointCloud2 & out);

  /// @brief Append the points of `src` to `dst`. Both must use the PointXYZIRC layout.
  static void append_pointcloud(
    const sensor_msgs::msg::PointCloud2 & src, sensor_msgs::msg::PointCloud2 & dst);

  void correct_pointcloud_motion(
    const std::unique_ptr<sensor_msgs::msg::PointCloud2> & transformed_cloud_ptr,
    const std::vector<builtin_interfaces::msg::Time> & pc_stamps,
    std::unordered_map<builtin_interfaces::msg::Time, Eigen::Matrix4f, TimeHash> & transform_memo,
    std::unique_ptr<sensor_msgs::msg::PointCloud2> & transformed_delay_compensated_cloud_ptr,
    MotionCompensationStatus * status);

private:
  /// @brief Record each input cloud's original stamp into `result`.
  /// @return The input timestamps sorted newest first, as required by correct_pointcloud_motion().
  static std::vector<builtin_interfaces::msg::Time> collect_input_timestamps(
    const std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> &
      topic_to_cloud_map,
    ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> & result);

  /// @brief Allocate the output cloud (XYZIRC layout) and its concatenation info.
  void initialize_concatenated_cloud(
    const std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> &
      topic_to_cloud_map,
    ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> & result);

  /// @brief Write the matching strategy config of `collector_info` into the concatenation info.
  static void set_matching_strategy_config(
    const std::shared_ptr<CollectorInfoBase> & collector_info,
    ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> & result);

  /// @brief Store the already motion-compensated cloud of `topic` as its synchronized cloud,
  /// optionally transformed back into the input sensor frame.
  void store_synchronized_cloud(
    const std::string & topic, const std::string & input_frame_id,
    const Eigen::Matrix4f & sensor_to_output, const builtin_interfaces::msg::Time & oldest_stamp,
    std::unique_ptr<sensor_msgs::msg::PointCloud2> compensated_cloud,
    ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> & result);

  /// @brief Convert, transform, motion-compensate and append one input cloud to the output cloud.
  /// Clouds without a transform to the output frame are dropped and reported in `result`.
  void process_input_cloud(
    const std::string & topic, const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const std::vector<builtin_interfaces::msg::Time> & pc_stamps,
    const builtin_interfaces::msg::Time & oldest_stamp,
    std::unordered_map<builtin_interfaces::msg::Time, Eigen::Matrix4f, TimeHash> & transform_memo,
    ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> & result);

  /// @brief Set the stamp and the width/height/row_step of the output cloud.
  /// @throw std::runtime_error if the data size is not a multiple of point_step.
  static void finalize_concatenated_cloud(
    const builtin_interfaces::msg::Time & oldest_stamp,
    sensor_msgs::msg::PointCloud2 & concatenate_cloud);
};

}  // namespace autoware::pointcloud_preprocessor
