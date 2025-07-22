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

#include "concatenation_info.hpp"
#include "traits.hpp"

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// ROS includes
#include <managed_transform_buffer/managed_transform_buffer.hpp>

#include <autoware_sensing_msgs/msg/concatenated_point_cloud_info.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace autoware::pointcloud_preprocessor
{

template <typename MsgTraits>
struct ConcatenatedCloudResult
{
  typename MsgTraits::PointCloudMessage::UniquePtr concatenate_cloud_ptr{nullptr};
  autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo::UniquePtr concatenation_info_ptr;
  std::optional<std::unordered_map<std::string, typename MsgTraits::PointCloudMessage::UniquePtr>>
    topic_to_transformed_cloud_map;
  std::unordered_map<std::string, double> topic_to_original_stamp_map;
};

class CombineCloudHandlerBase
{
public:
  CombineCloudHandlerBase(
    rclcpp::Node & node, const std::vector<std::string> & input_topics, std::string output_frame,
    bool is_motion_compensated, bool publish_synchronized_pointcloud,
    bool keep_input_frame_in_synchronized_pointcloud)
  : node_(node),
    input_topics_(input_topics),
    output_frame_(output_frame),
    is_motion_compensated_(is_motion_compensated),
    publish_synchronized_pointcloud_(publish_synchronized_pointcloud),
    keep_input_frame_in_synchronized_pointcloud_(keep_input_frame_in_synchronized_pointcloud),
    managed_tf_buffer_(std::make_unique<managed_transform_buffer::ManagedTransformBuffer>()),
    concatenation_info_(node.get_parameter("matching_strategy.type").as_string(), input_topics)
  {
  }

  void process_twist(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr & twist_msg);

  void process_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr & input);

  std::deque<geometry_msgs::msg::TwistStamped> get_twist_queue();

  Eigen::Matrix4f compute_transform_to_adjust_for_old_timestamp(
    const rclcpp::Time & old_stamp, const rclcpp::Time & new_stamp);

  virtual void allocate_pointclouds() = 0;

protected:
  rclcpp::Node & node_;
  std::vector<std::string> input_topics_;
  std::string output_frame_;
  bool is_motion_compensated_;
  bool publish_synchronized_pointcloud_;
  bool keep_input_frame_in_synchronized_pointcloud_;
  std::unique_ptr<managed_transform_buffer::ManagedTransformBuffer> managed_tf_buffer_{nullptr};
  ConcatenationInfo concatenation_info_;

  std::deque<geometry_msgs::msg::TwistStamped> twist_queue_;
};

}  // namespace autoware::pointcloud_preprocessor
