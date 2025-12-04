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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD2COSTMAP_FILTER__CUDA_POINTCLOUD2COSTMAP_FILTER_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD2COSTMAP_FILTER__CUDA_POINTCLOUD2COSTMAP_FILTER_HPP_

#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/types.hpp"
#include "autoware/cuda_utils/cuda_check_error.hpp"

#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_blackboard/cuda_unique_ptr.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <cuda_runtime.h>
#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/fill.h>
#include <thrust/reduce.h>
#include <thrust/sort.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{

// CUDA kernel parameters struct (device-side)
struct CostmapGridParams
{
  float grid_length_x;
  float grid_length_y;
  float grid_resolution;
  float grid_position_x;
  float grid_position_y;
  float maximum_height_thres;
  float minimum_height_thres;
  float grid_min_value;
  float grid_max_value;
  int32_t grid_size_x;
  int32_t grid_size_y;
};

// Host-side costmap parameters struct
struct CostmapParameters
{
  double grid_length_x;
  double grid_length_y;
  double grid_resolution;
  double grid_position_x;
  double grid_position_y;
  double maximum_height_thres;
  double minimum_height_thres;
  double grid_min_value;
  double grid_max_value;
};

struct GridCellData
{
  float z_min;
  float z_max;
  int32_t point_count;
};

class CudaPointcloud2CostmapFilter
{
public:
  explicit CudaPointcloud2CostmapFilter(
    const CostmapParameters & costmap_parameters, int64_t max_mem_pool_size_in_byte = 1e9);
  ~CudaPointcloud2CostmapFilter() = default;

  /// \brief Generate costmap from pointcloud
  /// \param[in] input_points: Input pointcloud in costmap frame
  /// \param[out] costmap_matrix: Output costmap as grid_map::Matrix
  void generateCostmap(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points,
    grid_map::Matrix & costmap_matrix);

private:
  template <typename T>
  T * allocateBufferFromPool(size_t num_elements);

  template <typename T>
  void returnBufferToPool(T * buffer);

  CostmapParameters costmap_parameters_;
  int32_t grid_size_x_;
  int32_t grid_size_y_;

  cudaStream_t stream_{};
  cudaMemPool_t mem_pool_{};

  static constexpr int threads_per_block_ = 512;
};

// CUDA kernel launch functions
void assignPointsToGridCellsLaunch(
  const InputPointType * input_points, int32_t * grid_indices, int num_points,
  const CostmapGridParams * params, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream);

void calculateCostmapFromGridCellsLaunch(
  const InputPointType * input_points, const int32_t * grid_indices, float * costmap_data,
  int num_points, const CostmapGridParams * params, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream);

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD2COSTMAP_FILTER__CUDA_POINTCLOUD2COSTMAP_FILTER_HPP_
