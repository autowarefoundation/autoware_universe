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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__DAG_UTILS_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__DAG_UTILS_HPP_

#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <memory>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

/**
 * @brief Convert shared_ptr to unique_ptr for publishing
 *
 * This follows the standard pattern in CUDA filters:
 * - Internal processing uses shared_ptr for flexibility
 * - Publishing requires unique_ptr for ownership transfer
 *
 * This function creates a NEW unique_ptr and copies the pointcloud data.
 * This follows the same pattern as cuda_polar_voxel_outlier_filter.cu:913-937
 *
 * @param shared_cloud Shared pointer to CudaPointCloud2
 * @return Unique pointer to CudaPointCloud2 (suitable for publishing)
 */
inline std::unique_ptr<cuda_blackboard::CudaPointCloud2> convert_shared_to_unique_for_publishing(
  const std::shared_ptr<cuda_blackboard::CudaPointCloud2> & shared_cloud)
{
  if (!shared_cloud || shared_cloud->width == 0) {
    return std::make_unique<cuda_blackboard::CudaPointCloud2>();
  }

  // Create new unique_ptr with same metadata
  auto unique_cloud = std::make_unique<cuda_blackboard::CudaPointCloud2>();

  // Copy metadata
  unique_cloud->header = shared_cloud->header;
  unique_cloud->fields = shared_cloud->fields;
  unique_cloud->is_bigendian = shared_cloud->is_bigendian;
  unique_cloud->point_step = shared_cloud->point_step;
  unique_cloud->is_dense = shared_cloud->is_dense;
  unique_cloud->height = shared_cloud->height;
  unique_cloud->width = shared_cloud->width;
  unique_cloud->row_step = shared_cloud->row_step;

  // Allocate new data buffer and copy (follows pattern from cuda_polar_voxel_outlier_filter.cu:928)
  unique_cloud->data = cuda_blackboard::make_unique<std::uint8_t[]>(unique_cloud->row_step);

  // Copy data Device-to-Device
  if (shared_cloud->data && unique_cloud->data) {
    cudaMemcpy(
      unique_cloud->data.get(), shared_cloud->data.get(), unique_cloud->row_step,
      cudaMemcpyDeviceToDevice);
  }

  return unique_cloud;
}

}  // namespace autoware::cuda_pointcloud_preprocessor::dag

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__DAG_UTILS_HPP_
