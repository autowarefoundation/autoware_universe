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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__CUDA_POINTCLOUD_PREPROCESSOR_DAG_NODE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__CUDA_POINTCLOUD_PREPROCESSOR_DAG_NODE_HPP_

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/dag_config_parser.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/dag_executor.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/filter_interface.hpp"

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

/**
 * @brief ROS2 node that executes a DAG-based pointcloud preprocessing pipeline
 */
class CudaPointcloudPreprocessorDagNode : public rclcpp::Node
{
public:
  explicit CudaPointcloudPreprocessorDagNode(const rclcpp::NodeOptions & node_options);

private:
  void pointcloudCallback(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr input_pointcloud_msg_ptr);

  void updateTwistQueue(std::uint64_t first_point_stamp);
  void updateImuQueue(std::uint64_t first_point_stamp);

  /**
   * @brief Parse DAG configuration from parameters
   */
  std::vector<DagNodeConfig> parseDagConfig();

  /**
   * @brief Create subscribers based on DAG input configuration
   */
  void createDynamicSubscribers();

  /**
   * @brief Create publishers based on DAG output configuration
   */
  void createDynamicPublishers();

  DagExecutor executor_;
  FilterContext context_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  // Dynamic subscribers (created based on DAG inputs)
  std::shared_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>
    pointcloud_sub_;
  autoware_utils::InterProcessPollingSubscriber<
    geometry_msgs::msg::TwistWithCovarianceStamped, autoware_utils::polling_policy::All>::SharedPtr
    twist_sub_;
  autoware_utils::InterProcessPollingSubscriber<
    sensor_msgs::msg::Imu, autoware_utils::polling_policy::All>::SharedPtr imu_sub_;

  // Dynamic publishers (created based on DAG outputs)
  // Map from output name to publisher
  std::map<
    std::string,
    std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>>
    publishers_;

  // Input queues
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> twist_queue_;
  std::deque<geometry_msgs::msg::Vector3Stamped> angular_velocity_queue_;

  // CUDA resources
  cudaStream_t cuda_stream_{};
  cudaMemPool_t cuda_memory_pool_{};

  // Shared preprocessor instance (used by all filters)
  std::unique_ptr<CudaPointcloudPreprocessor> shared_preprocessor_;

  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_;

  // DAG configuration (parsed from YAML)
  std::vector<DagInputConfig> dag_input_configs_;
  std::vector<DagOutputConfig> dag_output_configs_;
};

}  // namespace autoware::cuda_pointcloud_preprocessor::dag

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__CUDA_POINTCLOUD_PREPROCESSOR_DAG_NODE_HPP_
