// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__PTV3__PTV3_NODE_HPP_
#define AUTOWARE__PTV3__PTV3_NODE_HPP_

#include "autoware/ptv3/ptv3_trt.hpp"
#include "autoware/ptv3/visibility_control.hpp"

#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>

namespace autoware::ptv3
{

class PTV3_PUBLIC PTv3Node : public rclcpp::Node
{
public:
  explicit PTv3Node(const rclcpp::NodeOptions & options);

  void publishSegmentedPointcloud(std::unique_ptr<const cuda_blackboard::CudaPointCloud2> msg_ptr);

  void publishGroundSegmentedPointcloud(
    std::unique_ptr<const cuda_blackboard::CudaPointCloud2> msg_ptr);

  // cSpell:ignore Probs
  void publishProbsPointcloud(std::unique_ptr<const cuda_blackboard::CudaPointCloud2> msg_ptr);

private:
  void cloudCallback(const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr);

  std::unique_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>
    pointcloud_sub_;

  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    segmented_pointcloud_pub_;

  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    ground_segmented_pointcloud_pub_;

  // cSpell:ignore probs
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    probs_pointcloud_pub_;

  std::unique_ptr<PTv3TRT> model_ptr_{nullptr};

  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  // debugger
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{nullptr};
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_ptr_{nullptr};
  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_pub_{nullptr};
};
}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PTV3_NODE_HPP_
