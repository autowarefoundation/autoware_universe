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

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// ROS includes
#include "autoware/point_types/types.hpp"

#include <autoware_utils/ros/managed_transform_buffer.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

namespace autoware::pointcloud_preprocessor
{
using autoware::point_types::PointXYZIRC;
using point_cloud_msg_wrapper::PointCloud2Modifier;

struct ConcatenatedCloudResult
{
  sensor_msgs::msg::PointCloud2::SharedPtr concatenate_cloud_ptr{nullptr};
  std::optional<std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>>
    topic_to_transformed_cloud_map;
  std::unordered_map<std::string, double> topic_to_original_stamp_map;
};

class CombineCloudHandler
{
private:
  rclcpp::Node & node_;

  std::string output_frame_;
  bool is_motion_compensated_;
  bool publish_synchronized_pointcloud_;
  bool keep_input_frame_in_synchronized_pointcloud_;
  std::unique_ptr<autoware_utils::ManagedTransformBuffer> managed_tf_buffer_{nullptr};

  std::deque<geometry_msgs::msg::TwistStamped> twist_queue_;

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
    const sensor_msgs::msg::PointCloud2::SharedPtr & input_cloud,
    sensor_msgs::msg::PointCloud2::SharedPtr & xyzirc_cloud);

  void correct_pointcloud_motion(
    const std::shared_ptr<sensor_msgs::msg::PointCloud2> & transformed_cloud_ptr,
    const std::vector<rclcpp::Time> & pc_stamps,
    std::unordered_map<rclcpp::Time, Eigen::Matrix4f, RclcppTimeHash> & transform_memo,
    std::shared_ptr<sensor_msgs::msg::PointCloud2> transformed_delay_compensated_cloud_ptr);

public:
  CombineCloudHandler(
    rclcpp::Node & node, std::string output_frame, bool is_motion_compensated,
    bool publish_synchronized_pointcloud, bool keep_input_frame_in_synchronized_pointcloud,
    bool has_static_tf_only);
  void process_twist(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr & twist_msg);
  void process_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg);

  ConcatenatedCloudResult combine_pointclouds(
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> & topic_to_cloud_map);

  Eigen::Matrix4f compute_transform_to_adjust_for_old_timestamp(
    const rclcpp::Time & old_stamp, const rclcpp::Time & new_stamp);

  std::deque<geometry_msgs::msg::TwistStamped> get_twist_queue();
};

}  // namespace autoware::pointcloud_preprocessor
