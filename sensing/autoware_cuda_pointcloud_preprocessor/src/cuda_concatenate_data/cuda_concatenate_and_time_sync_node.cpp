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

#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_concatenate_and_time_sync_node.hpp"

#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_cloud_collector.hpp"
#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_traits.hpp"
#include "autoware/pointcloud_preprocessor/utility/memory.hpp"

#include <autoware_sensing_msgs/msg/concatenated_point_cloud_info.hpp>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

template <>
void PointCloudConcatenateDataSynchronizerComponentTemplated<
  CudaPointCloud2Traits>::initialize_pub_sub()
{
  concatenated_cloud_publisher_ =
    std::make_shared<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "output");
  concatenation_info_publisher_ =
    this->create_publisher<autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo>(
      "output_info", rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size));

  for (auto & topic : params_.input_topics) {
    std::string new_topic =
      replace_sync_topic_name_postfix(topic, params_.synchronized_pointcloud_postfix);
    auto publisher =
      std::make_shared<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
        *this, new_topic);
    topic_to_transformed_cloud_publisher_map_.insert({topic, publisher});
  }

  for (const std::string & topic : params_.input_topics) {
    auto callback = [&](const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg) {
      this->cloud_callback(msg, topic);
    };

    auto pointcloud_sub =
      std::make_shared<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
        *this, topic, false, callback);
    pointcloud_subs_.push_back(pointcloud_sub);
  }

  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "Subscribing to " << params_.input_topics.size() << " user given topics as inputs:");

  for (const auto & input_topic : params_.input_topics) {
    RCLCPP_DEBUG_STREAM(get_logger(), " - " << input_topic);
  }
}

}  // namespace autoware::pointcloud_preprocessor

template class autoware::pointcloud_preprocessor::
  PointCloudConcatenateDataSynchronizerComponentTemplated<
    autoware::pointcloud_preprocessor::CudaPointCloud2Traits>;

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor::CudaPointCloudConcatenateDataSynchronizerComponent)
