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

#include "autoware/euclidean_cluster/euclidean_cluster.hpp"

#include <autoware/shape_estimation/shape_estimator.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::euclidean_cluster
{
enum ShapePolicy : uint8_t {
  ALL_POLYGON = 0,
  LABEL_DEPEND = 1,
};

/// @brief ROS 2 node that performs euclidean clustering on semantic point clouds grouped by label.
class LabelBasedEuclideanClusterNode : public rclcpp::Node
{
public:
  /// @brief Construct the node and initialize parameters, publishers, subscribers, and helpers.
  /// @param options ROS 2 node options.
  explicit LabelBasedEuclideanClusterNode(const rclcpp::NodeOptions & options);

private:
  /// @brief Process an input semantic point cloud and publish detected objects.
  /// @param input_msg Input point cloud containing xyz and optionally class_id / probability.
  void on_pointcloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);

  /// @brief Build the mapping from semantic class IDs to Autoware object labels.
  /// @param class_mappings Ordered class mappings from original class name to Autoware label.
  /// @return True if at least one supported class is configured.
  bool update_target_label_map(
    const std::vector<std::pair<std::string, std::string>> & class_mappings);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;

  std::shared_ptr<EuclideanCluster> cluster_;
  std::unique_ptr<autoware::shape_estimation::ShapeEstimator> shape_estimator_;
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_;

  std::unordered_map<std::uint8_t, std::uint8_t> class_id_to_object_label_;
  float min_probability_;

  ShapePolicy shape_policy_;
};
}  // namespace autoware::euclidean_cluster
