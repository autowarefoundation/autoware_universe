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

#include "autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/cuda_random_downsample_filter_node.hpp"

#include "autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/cuda_random_downsample_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/pointcloud_preprocessor/utility/memory.hpp"

namespace autoware::cuda_pointcloud_preprocessor
{
CudaRandomDownsampleFilterNode::CudaRandomDownsampleFilterNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_random_downsample_filter", node_options)
{
  // Set initial parameters
  size_t sample_num = static_cast<size_t>(declare_parameter<int64_t>("sample_num"));
  const bool output_point_xyzircaedt = declare_parameter<bool>("output_point_xyzircaedt", false);
  int64_t max_mem_pool_size_in_byte = declare_parameter<int64_t>("max_mem_pool_size_in_byte", 1e9);
  if (max_mem_pool_size_in_byte < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid pool size was specified. The value should be positive");
    return;
  }

  sub_ =
    std::make_shared<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(
        &CudaRandomDownsampleFilterNode::cudaPointcloudCallback, this, std::placeholders::_1));

  pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");

  cuda_random_downsample_filter_ = std::make_unique<CudaRandomDownsampleFilter>(
    sample_num, max_mem_pool_size_in_byte, output_point_xyzircaedt);
}

void CudaRandomDownsampleFilterNode::cudaPointcloudCallback(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg)
{
  // Validate point_step matches one of the supported formats
  constexpr size_t point_xyzirc_size = sizeof(OutputPointType);
  constexpr size_t point_xyzircaedt_size = sizeof(InputPointType);
  const bool is_point_xyzirc = (msg->point_step == point_xyzirc_size);
  const bool is_point_xyzircaedt = (msg->point_step == point_xyzircaedt_size);

  if (!is_point_xyzirc && !is_point_xyzircaedt) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Point cloud point_step mismatch! Expected PointXYZIRC (%zu bytes) or "
      "PointXYZIRCAEDT (%zu bytes), but got %u bytes. Point cloud will be skipped.",
      point_xyzirc_size, point_xyzircaedt_size, msg->point_step);
    return;
  }

  // Validate data pointer
  if (!msg->data) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Point cloud data pointer is null! Point cloud will be skipped.");
    return;
  }

  // Process the point cloud with error handling
  try {
    auto output_pointcloud_ptr = cuda_random_downsample_filter_->filter(msg);
    if (output_pointcloud_ptr) {
      pub_->publish(std::move(output_pointcloud_ptr));
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Filter returned null output. Point cloud not published.");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Exception in filter processing: %s. Point cloud will be skipped.", e.what());
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Unknown exception in filter processing. Point cloud will be skipped.");
  }
}
}  // namespace autoware::cuda_pointcloud_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor::CudaRandomDownsampleFilterNode)
