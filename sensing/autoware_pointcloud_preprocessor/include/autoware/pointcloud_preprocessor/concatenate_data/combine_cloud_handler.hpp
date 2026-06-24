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
    bool keep_input_frame_in_synchronized_pointcloud, const std::string & matching_strategy_name)
  : CombineCloudHandlerBase(
      input_topics, output_frame, is_motion_compensated, publish_synchronized_pointcloud,
      keep_input_frame_in_synchronized_pointcloud, matching_strategy_name)
  {
  }

  ~CombineCloudHandler() override = default;

  ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> combine_pointclouds(
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> &
      topic_to_cloud_map,
    const std::shared_ptr<CollectorInfoBase> & collector_info);

  void allocate_pointclouds() override {};

protected:
  static void convert_to_xyzirc_cloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_cloud,
    sensor_msgs::msg::PointCloud2::UniquePtr & xyzirc_cloud);

  // ROS-runtime-free replacement for pcl_ros::transformPointCloud(Eigen::Matrix4f, in, out):
  static void transform_pointcloud(
    const Eigen::Matrix4f & transform, const sensor_msgs::msg::PointCloud2 & in,
    sensor_msgs::msg::PointCloud2 & out);

  // Appends src's point data onto dst in place (both must share the PointXYZIRC layout). Replaces
  // pcl::concatenatePointCloud, whose sensor_msgs overload lives in rclcpp-pulling pcl_conversions.
  static void append_pointcloud(
    const sensor_msgs::msg::PointCloud2 & src, sensor_msgs::msg::PointCloud2 & dst);

  void correct_pointcloud_motion(
    const std::unique_ptr<sensor_msgs::msg::PointCloud2> & transformed_cloud_ptr,
    const std::vector<int64_t> & pc_nanoseconds,
    std::unordered_map<int64_t, Eigen::Matrix4f> & transform_memo,
    std::unique_ptr<sensor_msgs::msg::PointCloud2> & transformed_delay_compensated_cloud_ptr,
    MotionCompensationStatus * status);

private:
  // Collects each input cloud's timestamp (in nanoseconds), records the per-topic original stamp
  // into `result`, and returns the timestamps sorted in descending order (newest first). The
  // descending order is required by correct_pointcloud_motion().
  static std::vector<int64_t> collect_input_timestamps(
    const std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> &
      topic_to_cloud_map,
    ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> & result);

  // Allocates the output cloud and its concatenation info, forces the XYZIRC field layout (so the
  // output stays XYZIRC even when every input is empty), and reserves space for all input data.
  void initialize_concatenated_cloud(
    const std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> &
      topic_to_cloud_map,
    ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> & result);

  // Stores the per-sensor synchronized cloud for `topic`, optionally transforming it back into the
  // input sensor frame. Takes ownership of motion-compensation.
  void store_synchronized_cloud(
    const std::string & topic, const std::string & input_frame_id,
    const Eigen::Matrix4f & sensor_to_output, const builtin_interfaces::msg::Time & oldest_stamp,
    std::unique_ptr<sensor_msgs::msg::PointCloud2> compensated_cloud,
    ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> & result);

  // Converts one input cloud to XYZIRC, transforms it into the output frame, motion-compensates it,
  // appends it to the output cloud (folding the source's is_dense into the output's), records its
  // source info, and (when enabled) stores the synchronized per-sensor cloud. Clouds without an
  // available transform are dropped and recorded.
  void process_input_cloud(
    const std::string & topic, const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const std::vector<int64_t> & pc_nanoseconds,
    const builtin_interfaces::msg::Time & oldest_stamp,
    std::unordered_map<int64_t, Eigen::Matrix4f> & transform_memo,
    ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> & result);

  // Stamps the output cloud and recomputes width/height/row_step now that the concatenated cloud is
  // unstructured. Throws if the data size is not a multiple of point_step.
  static void finalize_concatenated_cloud(
    const builtin_interfaces::msg::Time & oldest_stamp,
    sensor_msgs::msg::PointCloud2 & concatenate_cloud);

  // When `collector_info` is an AdvancedCollectorInfo, serializes its reference-timestamp window
  // into the concatenation info's strategy config.
  static void apply_advanced_collector_config(
    const std::shared_ptr<CollectorInfoBase> & collector_info,
    ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> & result);
};

}  // namespace autoware::pointcloud_preprocessor
