// Copyright 2026 TIER IV, Inc.
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

#include "autoware/euclidean_cluster/label_based_euclidean_cluster.hpp"

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace autoware::euclidean_cluster
{

/// @brief ROS 2 node adapter for label-based euclidean clustering.
///
/// This node wraps the core LabelBasedEuclideanCluster class, handling ROS-specific concerns
/// like parameter loading, pub/sub lifecycle, and timing instrumentation.
class LabelBasedEuclideanClusterNode : public rclcpp::Node
{
public:
  /// @brief Construct the node and initialize parameters, publishers, subscribers, and core
  /// cluster.
  /// @param options ROS 2 node options.
  explicit LabelBasedEuclideanClusterNode(const rclcpp::NodeOptions & options);

private:
  /// @brief Process an input semantic point cloud and publish detected objects.
  /// @param input_msg Input point cloud containing xyz and optionally class_id / probability.
  void on_pointcloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);

  // Publishers and subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  AUTOWARE_PUBLISHER_PTR(autoware_perception_msgs::msg::DetectedObjects) objects_pub_;
  AUTOWARE_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) segments_pub_;

  // Core clustering processor
  std::unique_ptr<LabelBasedEuclideanCluster> processor_;

  // Timing and debug instrumentation
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_;
};
}  // namespace autoware::euclidean_cluster
