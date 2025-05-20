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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_VOXEL_GRID_DOWNSAMPLE_FILTER_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_VOXEL_GRID_DOWNSAMPLE_FILTER_HPP_

#include "autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/thrust_custom_allocator.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_utils/cuda_check_error.hpp"

#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_blackboard/cuda_unique_ptr.hpp>

#include <cuda_runtime.h>

#include <cstdint>
#include <iostream>
#include <memory>

namespace autoware::cuda_pointcloud_preprocessor
{

class CudaVoxelGridDownsampleFilter
{
public:
  template <typename T>
  struct ThreeDim
  {
    T x;
    T y;
    T z;

    friend std::ostream & operator<<(std::ostream & os, const ThreeDim & t)
    {
      os << t.x << ", " << t.y << ", " << t.z;
      return os;
    }

    T & operator[](size_t i)
    {  // Define [] operator to make iterative access easy
      static T * members[] = {&x, &y, &z};
      return *members[i];
    }
  };

  struct OptionalField
  {
    bool is_valid;
    size_t offset;
  };

  struct VoxelInfo
  {
    size_t num_input_points;
    size_t input_point_step;
    size_t input_xyzi_offset[4];
    OptionalField input_return_type_offset;
    OptionalField input_channel_offset;
    size_t output_offsets[6];
    ThreeDim<float> voxel_size;
    ThreeDim<int> min_coord;
    ThreeDim<int> max_coord;
  };

  struct Centroid
  {
    float x;
    float y;
    float z;
    float i;
    unsigned int
      count;  // use unsigned int instead of size_t because of lack support of cuda's atomicAdd
  };

  explicit CudaVoxelGridDownsampleFilter(
    const float voxel_size_x, const float voxel_size_y, const float voxel_size_z,
    const int64_t max_mem_pool_size_in_byte);
  ~CudaVoxelGridDownsampleFilter() = default;

  std::unique_ptr<cuda_blackboard::CudaPointCloud2> filter(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points);

private:
  template <typename T>
  T * allocateBufferFromPool(size_t num_elements);

  template <typename T>
  void returnBufferToPool(T * buffer);

  void getVoxelMinMaxCoordinate(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & points, float * buffer_dev);
  size_t searchValidVoxel(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & points,
    size_t * voxel_index_buffer_dev, size_t * point_index_buffer_dev,
    decltype(OutputPointType::return_type) * return_type_field_dev,
    decltype(OutputPointType::channel) * channel_field_dev);
  void getCentroid(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points,
    const size_t num_valid_voxel, const size_t * voxel_index_dev, const size_t * point_index_dev,
    size_t * index_map_dev, Centroid * buffer_dev,
    std::unique_ptr<cuda_blackboard::CudaPointCloud2> & output_points,
    decltype(OutputPointType::return_type) * return_type_field_dev,
    decltype(OutputPointType::channel) * channel_field_dev);

  VoxelInfo voxel_info_{};

  cudaStream_t stream_{};
  cudaMemPool_t mem_pool_{};

  std::unique_ptr<ThrustCustomAllocator> thrust_custom_allocator_;
};
}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_VOXEL_GRID_DOWNSAMPLE_FILTER_HPP_
