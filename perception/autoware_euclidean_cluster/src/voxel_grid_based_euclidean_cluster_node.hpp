// Copyright 2020 Tier IV, Inc.
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

#include "autoware/euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <memory>

namespace autoware::euclidean_cluster
{
class VoxelGridBasedEuclideanClusterNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit VoxelGridBasedEuclideanClusterNode(const rclcpp::NodeOptions & options);

private:
  void onPointCloud(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::PointCloud2) & input_msg);

  AUTOWARE_SUBSCRIPTION_PTR(sensor_msgs::msg::PointCloud2) pointcloud_sub_;
  AUTOWARE_PUBLISHER_PTR(tier4_perception_msgs::msg::DetectedObjectsWithFeature) cluster_pub_;
  AUTOWARE_PUBLISHER_PTR(sensor_msgs::msg::PointCloud2) debug_pub_;

  std::shared_ptr<VoxelGridBasedEuclideanCluster> cluster_;
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::BasicDebugPublisher<autoware::agnocast_wrapper::Node>>
    debug_publisher_;
};

}  // namespace autoware::euclidean_cluster
