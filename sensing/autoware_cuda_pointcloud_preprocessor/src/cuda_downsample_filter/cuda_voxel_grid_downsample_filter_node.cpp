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

#include "autoware/cuda_downsample_filter/cuda_voxel_grid_downsample_filter_node.hpp"
#include "autoware/cuda_pointcloud_preprocessor/memory.hpp"

////////////////////////////////////////////////////////////////////////////////
// DEBUG
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cfloat>
////////////////////////////////////////////////////////////////////////////////

namespace autoware::cuda_downsample_filter
{
CudaVoxelGridDownsampleFilterNode::CudaVoxelGridDownsampleFilterNode(
  const rclcpp::NodeOptions & node_options)
    : Node ("cuda_voxel_grid_downsample_filter", node_options)
{
  // set initial parameters
  float voxel_size_x = declare_parameter<float>("voxel_size_x");
  float voxel_size_y = declare_parameter<float>("voxel_size_y");
  float voxel_size_z = declare_parameter<float>("voxel_size_z");

  sub_ =
      std::make_shared<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
          *this, "~/input/pointcloud",
          std::bind(&CudaVoxelGridDownsampleFilterNode::cudaPointcloudCallback, this,
                    std::placeholders::_1));

  pub_ =
      std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
          *this, "~/output/pointcloud");

  cuda_voxel_grid_downsample_filter_ = std::make_unique<CudaVoxelGridDownsampleFilter>(
      voxel_size_x, voxel_size_y, voxel_size_z);

}

void CudaVoxelGridDownsampleFilterNode::cudaPointcloudCallback(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg)
{
  // The following only checks compatibility with xyzi
  // (i.e., just check the first four elements of the point field are x, y, z, and intensity
  // and don't care the rest of the fields)
  if (!cuda_pointcloud_preprocessor::is_data_layout_compatible_with_point_xyzi(msg->fields)) {
    RCLCPP_ERROR(
        this->get_logger(), "Input pointcloud data layout is not compatible with PointXYZIRCAEDT");
  }

  ////////////////////////////////////////////////////////////////////////////////
  { // XXX: DEBUG
    auto ros_pcl = std::make_unique<sensor_msgs::msg::PointCloud2>();
    ros_pcl->header = msg->header;
    ros_pcl->height = msg->height;
    ros_pcl->width = msg->width;
    ros_pcl->fields = msg->fields;
    ros_pcl->is_bigendian = msg->is_bigendian;
    ros_pcl->point_step = msg->point_step;
    ros_pcl->row_step = msg->row_step;
    ros_pcl->is_dense = msg->is_dense;

    auto data_size = msg->height * msg->width * msg->point_step * sizeof(uint8_t);

    ros_pcl->data = std::vector<uint8_t>(data_size);
    cudaMemcpy(ros_pcl->data.data(), msg->data.get(), data_size, cudaMemcpyDeviceToHost);


    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*ros_pcl, *in_cloud);

    auto mins = CudaVoxelGridDownsampleFilter::ThreeDim<float>{FLT_MAX, FLT_MAX, FLT_MAX};
    auto maxs = CudaVoxelGridDownsampleFilter::ThreeDim<float>{FLT_MIN, FLT_MIN, FLT_MIN};
    // for (int i = 0; i < 2; i++) {
    //   std::cerr << "host: idx: " << i << ", value: " << in_cloud->points[i].x << std::endl;
    // }
    auto start = std::chrono::high_resolution_clock::now();
    for (const auto& p : in_cloud->points) {
      mins.x = p.x < mins.x ? p.x : mins.x;
      mins.y = p.y < mins.y ? p.y : mins.y;
      mins.z = p.z < mins.z ? p.z : mins.z;
      maxs.x = p.x > maxs.x ? p.x : maxs.x;
      maxs.y = p.y > maxs.y ? p.y : maxs.y;
      maxs.z = p.z > maxs.z ? p.z : maxs.z;
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end - start).count();

    // std::cerr << "   min: " << mins << std::endl
    //           << "   max: " << maxs << std::endl
    //           << "    @ " << duration << "[ms]" << std::endl;
  }
  ////////////////////////////////////////////////////////////////////////////////

  auto output_pointcloud_ptr = cuda_voxel_grid_downsample_filter_->filter(msg);
  pub_->publish(std::move(output_pointcloud_ptr));
  return;
}
}  // namespace autoware::cuda_downsample_filter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
    autoware::cuda_downsample_filter::CudaVoxelGridDownsampleFilterNode)
