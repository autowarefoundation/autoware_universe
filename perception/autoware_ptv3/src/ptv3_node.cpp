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

#include "autoware/ptv3/ptv3_node.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::ptv3
{

PTv3Node::PTv3Node(const rclcpp::NodeOptions & options) : Node("ptv3", options)
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor{}.set__read_only(true);

  // TensorRT parameters
  const std::string plugins_path = this->declare_parameter<std::string>("plugins_path", descriptor);

  // Network parameters
  const std::string onnx_path = this->declare_parameter<std::string>("onnx_path", descriptor);
  const std::string engine_path = this->declare_parameter<std::string>("engine_path", descriptor);
  const std::string trt_precision =
    this->declare_parameter<std::string>("trt_precision", descriptor);

  auto to_float_vector = [](const auto & v) -> std::vector<float> {
    return std::vector<float>(v.begin(), v.end());
  };

  // Lidar branch parameters
  const auto cloud_capacity = this->declare_parameter<std::int64_t>("cloud_capacity", descriptor);

  const auto voxels_num =
    this->declare_parameter<std::vector<std::int64_t>>("voxels_num", descriptor);
  const auto point_cloud_range =
    to_float_vector(this->declare_parameter<std::vector<double>>("point_cloud_range", descriptor));
  const auto voxel_size =
    to_float_vector(this->declare_parameter<std::vector<double>>("voxel_size", descriptor));

  // Head parameters
  auto class_names = this->declare_parameter<std::vector<std::string>>("class_names", descriptor);

  const auto colors_red = this->declare_parameter<std::vector<int>>("colors_red", descriptor);
  const auto colors_green = this->declare_parameter<std::vector<int>>("colors_green", descriptor);
  const auto colors_blue = this->declare_parameter<std::vector<int>>("colors_blue", descriptor);
  const auto ground_prob_threshold =
    this->declare_parameter<float>("ground_prob_threshold", descriptor);

  if (point_cloud_range.size() != 6) {
    throw std::runtime_error("The size of point_cloud_range != 6");
  }
  if (voxel_size.size() != 3) {
    throw std::runtime_error("The size of voxel_size != 3");
  }

  PTv3Config config(
    plugins_path, cloud_capacity, voxels_num, point_cloud_range, voxel_size, colors_red,
    colors_green, colors_blue, class_names, ground_prob_threshold);

  auto trt_config =
    tensorrt_common::TrtCommonConfig(onnx_path, trt_precision, engine_path, 1ULL << 33U);
  model_ptr_ = std::make_unique<PTv3TRT>(trt_config, config);

  pointcloud_sub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(&PTv3Node::cloudCallback, this, std::placeholders::_1));

  segmented_pointcloud_pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/segmented/pointcloud");

  ground_segmented_pointcloud_pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/ground_segmented/pointcloud");

  // cSpell:ignore Probs probs
  probs_pointcloud_pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/probs/pointcloud");

  published_time_pub_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);

  model_ptr_->setPublishSegmentedPointcloud(
    std::bind(&PTv3Node::publishSegmentedPointcloud, this, std::placeholders::_1));
  model_ptr_->setPublishGroundSegmentedPointcloud(
    std::bind(&PTv3Node::publishGroundSegmentedPointcloud, this, std::placeholders::_1));
  model_ptr_->setPublishProbsPointcloud(
    std::bind(&PTv3Node::publishProbsPointcloud, this, std::placeholders::_1));

  // initialize debug tool
  {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, this->get_name());
    stop_watch_ptr_->tic("cyclic");
    stop_watch_ptr_->tic("processing/total");
  }

  if (this->declare_parameter<bool>("build_only", false, descriptor)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine was built. Shutting down the node.");
    rclcpp::shutdown();
  }
}

void PTv3Node::publishSegmentedPointcloud(
  std::unique_ptr<const cuda_blackboard::CudaPointCloud2> msg_ptr)
{
  segmented_pointcloud_pub_->publish(std::move(msg_ptr));
}

void PTv3Node::publishGroundSegmentedPointcloud(
  std::unique_ptr<const cuda_blackboard::CudaPointCloud2> msg_ptr)
{
  ground_segmented_pointcloud_pub_->publish(std::move(msg_ptr));
}

void PTv3Node::publishProbsPointcloud(
  std::unique_ptr<const cuda_blackboard::CudaPointCloud2> msg_ptr)
{
  probs_pointcloud_pub_->publish(std::move(msg_ptr));
}

void PTv3Node::cloudCallback(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr)
{
  const auto segmented_sub_count =
    segmented_pointcloud_pub_->get_subscription_count() +
    segmented_pointcloud_pub_->get_intra_process_subscription_count();

  const auto ground_segmented_sub_count =
    ground_segmented_pointcloud_pub_->get_subscription_count() +
    ground_segmented_pointcloud_pub_->get_intra_process_subscription_count();

  const auto probs_sub_count = probs_pointcloud_pub_->get_subscription_count() +
                               probs_pointcloud_pub_->get_intra_process_subscription_count();

  if (segmented_sub_count + ground_segmented_sub_count + probs_sub_count == 0) {
    return;
  }

  if (stop_watch_ptr_) {
    stop_watch_ptr_->toc("processing/total", true);
  }

  std::unordered_map<std::string, double> proc_timing;
  bool is_success = model_ptr_->segment(
    msg_ptr, segmented_sub_count, ground_segmented_sub_count, probs_sub_count, proc_timing);
  if (!is_success) {
    return;
  }

  // add processing time for debug
  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing/total", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((this->get_clock()->now() - msg_ptr->header.stamp).nanoseconds()))
        .count();
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time/total_ms", processing_time_ms);
    for (const auto & [topic, time_ms] : proc_timing) {
      debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
        topic, time_ms);
    }
  }
}

}  // namespace autoware::ptv3

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ptv3::PTv3Node)
