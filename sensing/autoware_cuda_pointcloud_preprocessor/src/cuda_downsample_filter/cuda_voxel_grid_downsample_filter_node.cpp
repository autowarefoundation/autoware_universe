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

#include "autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/cuda_voxel_grid_downsample_filter_node.hpp"

#include "autoware/pointcloud_preprocessor/utility/memory.hpp"

namespace autoware::cuda_pointcloud_preprocessor
{
CudaVoxelGridDownsampleFilterNode::CudaVoxelGridDownsampleFilterNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_voxel_grid_downsample_filter", node_options)
{
  // set initial parameters
  float voxel_size_x = declare_parameter<float>("voxel_size_x");
  float voxel_size_y = declare_parameter<float>("voxel_size_y");
  float voxel_size_z = declare_parameter<float>("voxel_size_z");
  int64_t max_mem_pool_size_in_byte = declare_parameter<int64_t>(
    "max_mem_pool_size_in_byte",
    1e9);  // 1GB in default
  if (max_mem_pool_size_in_byte < 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid pool size was specified. The value should be positive");
    return;
  }

  sub_ =
    std::make_shared<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(
        &CudaVoxelGridDownsampleFilterNode::cudaPointcloudCallback, this, std::placeholders::_1));

  pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");

  cuda_voxel_grid_downsample_filter_ = std::make_unique<CudaVoxelGridDownsampleFilter>(
    voxel_size_x, voxel_size_y, voxel_size_z, max_mem_pool_size_in_byte);
}

void CudaVoxelGridDownsampleFilterNode::cudaPointcloudCallback(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg)
{
  // The following only checks compatibility with xyzi
  // (i.e., just check the first four elements of the point field are x, y, z, and intensity
  // and don't care the rest of the fields)
  if (!pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzi(msg->fields)) {
    // This filter assumes float for intensity data type, though the filter supports
    // other data types for the intensity field, so here just outputs a WARN message.
    RCLCPP_WARN(
      this->get_logger(),
      "Input pointcloud data layout is not compatible with PointXYZI. "
      "The output result may not be correct");
  }

  auto output_pointcloud_ptr = cuda_voxel_grid_downsample_filter_->filter(msg);
  pub_->publish(std::move(output_pointcloud_ptr));
}
}  // namespace autoware::cuda_pointcloud_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor::CudaVoxelGridDownsampleFilterNode)
