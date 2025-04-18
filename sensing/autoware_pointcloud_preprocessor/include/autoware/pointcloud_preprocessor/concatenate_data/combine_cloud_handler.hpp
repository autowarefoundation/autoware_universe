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

#include "combine_cloud_handler_base.hpp"
#include "traits.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ROS includes
#include "autoware/point_types/types.hpp"

namespace autoware::pointcloud_preprocessor
{
using autoware::point_types::PointXYZIRC;
using point_cloud_msg_wrapper::PointCloud2Modifier;

template <typename MsgTraits>
class CombineCloudHandler;

template <>
class CombineCloudHandler<PointCloud2Traits> : public CombineCloudHandlerBase
{
public:
  CombineCloudHandler(
    rclcpp::Node & node, const std::vector<std::string> & input_topics, std::string output_frame,
    bool is_motion_compensated, bool publish_synchronized_pointcloud,
    bool keep_input_frame_in_synchronized_pointcloud)
  : CombineCloudHandlerBase(
      node, input_topics, output_frame, is_motion_compensated, publish_synchronized_pointcloud,
      keep_input_frame_in_synchronized_pointcloud)
  {
  }

  virtual ~CombineCloudHandler() {}

  ConcatenatedCloudResult<PointCloud2Traits> combine_pointclouds(
    std::unordered_map<std::string, typename PointCloud2Traits::PointCloudMessage::ConstSharedPtr> &
      topic_to_cloud_map);

  void allocate_pointclouds() override {};

protected:
  /// @brief RclcppTimeHash structure defines a custom hash function for the rclcpp::Time type by
  /// using its nanoseconds representation as the hash value.
  struct RclcppTimeHash
  {
    std::size_t operator()(const rclcpp::Time & t) const
    {
      return std::hash<int64_t>()(t.nanoseconds());
    }
  };

  static void convert_to_xyzirc_cloud(
    const typename PointCloud2Traits::PointCloudMessage::ConstSharedPtr & input_cloud,
    typename PointCloud2Traits::PointCloudMessage::UniquePtr & xyzirc_cloud);

  void correct_pointcloud_motion(
    const std::unique_ptr<PointCloud2Traits::PointCloudMessage> & transformed_cloud_ptr,
    const std::vector<rclcpp::Time> & pc_stamps,
    std::unordered_map<rclcpp::Time, Eigen::Matrix4f, RclcppTimeHash> & transform_memo,
    std::unique_ptr<PointCloud2Traits::PointCloudMessage> &
      transformed_delay_compensated_cloud_ptr);
};

}  // namespace autoware::pointcloud_preprocessor
