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

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::ptv3
{

PTv3Node::PTv3Node(const rclcpp::NodeOptions & options) : Node("ptv3", options)
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor{}.set__read_only(true);
  auto to_float_vector = [](const auto & values) {
    return std::vector<float>(values.begin(), values.end());
  };

  // Runtime parameters.
  const std::string plugins_path = this->declare_parameter<std::string>("plugins_path", descriptor);
  const std::string trt_precision =
    this->declare_parameter<std::string>("trt_precision", descriptor);
  const auto cloud_capacity = this->declare_parameter<std::int64_t>("cloud_capacity", descriptor);

  // Backbone paths.
  const std::string backbone_onnx_path =
    this->declare_parameter<std::string>("backbone.onnx_path", descriptor);
  const std::string backbone_engine_path =
    this->declare_parameter<std::string>("backbone.engine_path", descriptor);

  // Backbone parameters from ml_package.
  const auto voxels_num =
    this->declare_parameter<std::vector<std::int64_t>>("backbone.voxels_num", descriptor);
  const auto point_cloud_range = to_float_vector(
    this->declare_parameter<std::vector<double>>("backbone.point_cloud_range", descriptor));
  const auto voxel_size = to_float_vector(
    this->declare_parameter<std::vector<double>>("backbone.voxel_size", descriptor));

  if (point_cloud_range.size() != 6) {
    throw std::runtime_error("The size of point_cloud_range != 6");
  }
  if (voxel_size.size() != 3) {
    throw std::runtime_error("The size of voxel_size != 3");
  }

  // Segmentation head selection and params.
  const bool use_seg3d_head = this->declare_parameter<bool>("segmentation3d.use_head", descriptor);
  std::optional<tensorrt_common::TrtCommonConfig> seg3d_head_trt_config;
  std::vector<std::string> class_names_seg3d;
  std::vector<std::int64_t> palette;
  float filter_class_probability_threshold = 0.0F;
  std::vector<std::string> filter_classes;
  std::string filter_output_format;
  std::string source_reconstruction = "none";
  if (use_seg3d_head) {
    const std::string seg3d_head_onnx_path =
      this->declare_parameter<std::string>("segmentation3d.onnx_path", descriptor);
    const std::string seg3d_head_engine_path =
      this->declare_parameter<std::string>("segmentation3d.engine_path", descriptor);
    class_names_seg3d =
      this->declare_parameter<std::vector<std::string>>("segmentation3d.class_names", descriptor);
    palette =
      this->declare_parameter<std::vector<std::int64_t>>("segmentation3d.palette", descriptor);
    filter_class_probability_threshold = static_cast<float>(this->declare_parameter<double>(
      "segmentation3d.filter.class_probability_threshold", descriptor));
    filter_classes = this->declare_parameter<std::vector<std::string>>(
      "segmentation3d.filter.classes", descriptor);
    filter_output_format =
      this->declare_parameter<std::string>("segmentation3d.filter.output_format", descriptor);
    source_reconstruction =
      this->declare_parameter<std::string>("segmentation3d.source_reconstruction", descriptor);
    seg3d_head_trt_config.emplace(
      seg3d_head_onnx_path, trt_precision, seg3d_head_engine_path, 1ULL << 30U);
  }

  PTv3Config config(
    plugins_path, cloud_capacity, voxels_num, point_cloud_range, voxel_size, class_names_seg3d,
    palette, filter_class_probability_threshold, filter_classes, filter_output_format,
    source_reconstruction, use_seg3d_head);

  const auto backbone_trt_config = tensorrt_common::TrtCommonConfig(
    backbone_onnx_path, trt_precision, backbone_engine_path, 1ULL << 33U);

  model_ptr_ = std::make_unique<PTv3TRT>(backbone_trt_config, seg3d_head_trt_config, config);

  pointcloud_sub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(&PTv3Node::cloud_callback, this, std::placeholders::_1));

  if (use_seg3d_head) {
    segmented_pointcloud_pub_ =
      std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
        *this, "~/output/pointcloud/segmentation");
    visualization_pointcloud_pub_ =
      std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
        *this, "~/output/pointcloud/visualization");
    filtered_pointcloud_pub_ =
      std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
        *this, "~/output/pointcloud/filtered");

    model_ptr_->set_publish_segmented_pointcloud(
      std::bind(&PTv3Node::publish_segmented_pointcloud, this, std::placeholders::_1));
    model_ptr_->set_publish_visualization_pointcloud(
      std::bind(&PTv3Node::publish_visualization_pointcloud, this, std::placeholders::_1));
    model_ptr_->set_publish_filtered_pointcloud(
      std::bind(&PTv3Node::publish_filtered_pointcloud, this, std::placeholders::_1));
  }

  published_time_pub_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);

  // Initialize debug helpers.
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

void PTv3Node::publish_segmented_pointcloud(
  std::unique_ptr<const cuda_blackboard::CudaPointCloud2> msg_ptr)
{
  if (segmented_pointcloud_pub_) {
    segmented_pointcloud_pub_->publish(std::move(msg_ptr));
  }
}

void PTv3Node::publish_visualization_pointcloud(
  std::unique_ptr<const cuda_blackboard::CudaPointCloud2> msg_ptr)
{
  if (visualization_pointcloud_pub_) {
    visualization_pointcloud_pub_->publish(std::move(msg_ptr));
  }
}

void PTv3Node::publish_filtered_pointcloud(
  std::unique_ptr<const cuda_blackboard::CudaPointCloud2> msg_ptr)
{
  if (filtered_pointcloud_pub_) {
    filtered_pointcloud_pub_->publish(std::move(msg_ptr));
  }
}

void PTv3Node::cloud_callback(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr)
{
  const auto segmented_sub_count =
    segmented_pointcloud_pub_ ? (segmented_pointcloud_pub_->get_subscription_count() +
                                 segmented_pointcloud_pub_->get_intra_process_subscription_count())
                              : 0u;
  const auto visualization_sub_count =
    visualization_pointcloud_pub_
      ? (visualization_pointcloud_pub_->get_subscription_count() +
         visualization_pointcloud_pub_->get_intra_process_subscription_count())
      : 0u;
  const auto filtered_sub_count =
    filtered_pointcloud_pub_ ? (filtered_pointcloud_pub_->get_subscription_count() +
                                filtered_pointcloud_pub_->get_intra_process_subscription_count())
                             : 0u;

  if (segmented_sub_count + visualization_sub_count + filtered_sub_count == 0u) {
    return;
  }

  if (stop_watch_ptr_) {
    stop_watch_ptr_->toc("processing/total", true);
  }

  std::unordered_map<std::string, double> proc_timing;

  const bool is_success = model_ptr_->infer(
    msg_ptr, segmented_sub_count > 0u, visualization_sub_count > 0u, filtered_sub_count > 0u,
    proc_timing);

  if (!is_success) {
    return;
  }

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
