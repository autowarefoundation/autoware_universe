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

#include "autoware/cuda_pointcloud_preprocessor/common_kernels.hpp"
#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud2costmap_filter/cuda_pointcloud2costmap_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_utils/cuda_check_error.hpp"

#include <thrust/device_ptr.h>
#include <thrust/fill.h>

#include <cmath>
#include <cstdint>
#include <memory>

namespace autoware::cuda_pointcloud_preprocessor
{

// CUDA kernels for pointcloud to costmap conversion
__global__ void assignPointsToGridCellsKernel(
  const InputPointType * __restrict__ input_points, int32_t * __restrict__ grid_indices,
  int num_points, const CostmapGridParams * __restrict__ params)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    const InputPointType & point = input_points[idx];
    const float x = point.x;
    const float y = point.y;

    // Check if point is valid
    if (!isfinite(x) || !isfinite(y)) {
      grid_indices[idx] = -1;  // Invalid index
      return;
    }

    // Calculate grid index following the same logic as CPU version
    // From points_to_costmap.cpp: fetchGridIndexFromPoint
    const float origin_x_offset = params->grid_length_x / 2.0f - params->grid_position_x;
    const float origin_y_offset = params->grid_length_y / 2.0f - params->grid_position_y;

    // Coordinate conversion for making index. Set bottom left to the origin of coordinate (0, 0)
    float mapped_x = (params->grid_length_x - origin_x_offset - x) / params->grid_resolution;
    float mapped_y = (params->grid_length_y - origin_y_offset - y) / params->grid_resolution;

    int32_t mapped_x_ind = static_cast<int32_t>(ceilf(mapped_x));
    int32_t mapped_y_ind = static_cast<int32_t>(ceilf(mapped_y));

    // Check if index is valid
    if (
      mapped_x_ind >= 0 && mapped_x_ind < params->grid_size_x && mapped_y_ind >= 0 &&
      mapped_y_ind < params->grid_size_y) {
      // Store as linear index: x * grid_size_y + y
      grid_indices[idx] = mapped_x_ind * params->grid_size_y + mapped_y_ind;
    } else {
      grid_indices[idx] = -1;  // Invalid index
    }
  }
}

__global__ void calculateCostmapFromGridCellsKernel(
  const InputPointType * __restrict__ input_points, const int32_t * __restrict__ grid_indices,
  float * __restrict__ costmap_data, int num_points, const CostmapGridParams * __restrict__ params)
{
  // First, initialize all costmap cells to min_value
  int grid_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int total_grid_cells = params->grid_size_x * params->grid_size_y;
  if (grid_idx < total_grid_cells) {
    costmap_data[grid_idx] = params->grid_min_value;
  }

  __syncthreads();

  // Then, for each point, check if it should contribute to costmap
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx < num_points) {
    const int32_t grid_cell_idx = grid_indices[point_idx];
    if (grid_cell_idx < 0 || grid_cell_idx >= total_grid_cells) {
      return;  // Invalid grid cell
    }

    const InputPointType & point = input_points[point_idx];
    const float z = point.z;

    // Check if z is within height thresholds
    if (z <= params->maximum_height_thres && z >= params->minimum_height_thres) {
      // Use atomic operation to set costmap value to max_value
      // This ensures thread safety when multiple points map to the same cell
      atomicExch(&costmap_data[grid_cell_idx], params->grid_max_value);
    }
  }
}

// Kernel launch functions
void assignPointsToGridCellsLaunch(
  const InputPointType * input_points, int32_t * grid_indices, int num_points,
  const CostmapGridParams * params, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream)
{
  assignPointsToGridCellsKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, grid_indices, num_points, params);
}

void calculateCostmapFromGridCellsLaunch(
  const InputPointType * input_points, const int32_t * grid_indices, float * costmap_data,
  int num_points, const CostmapGridParams * params, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream)
{
  // Calculate blocks for grid cells (for initialization)
  int total_grid_cells = params->grid_size_x * params->grid_size_y;
  int blocks_for_grid = (total_grid_cells + threads_per_block - 1) / threads_per_block;

  // Use max of blocks_for_grid and blocks_per_grid to handle both initialization and point
  // processing
  int total_blocks = (blocks_for_grid > blocks_per_grid) ? blocks_for_grid : blocks_per_grid;

  calculateCostmapFromGridCellsKernel<<<total_blocks, threads_per_block, 0, stream>>>(
    input_points, grid_indices, costmap_data, num_points, params);
}

CudaPointcloud2CostmapFilter::CudaPointcloud2CostmapFilter(
  const CostmapParameters & costmap_parameters, int64_t max_mem_pool_size_in_byte)
: costmap_parameters_(costmap_parameters)
{
  // Calculate grid size
  grid_size_x_ = static_cast<int32_t>(
    std::ceil(costmap_parameters.grid_length_x / costmap_parameters.grid_resolution));
  grid_size_y_ = static_cast<int32_t>(
    std::ceil(costmap_parameters.grid_length_y / costmap_parameters.grid_resolution));

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

  // Create memory pool to make repeated allocation efficient
  {
    int current_device_id = 0;
    CHECK_CUDA_ERROR(cudaGetDevice(&current_device_id));
    cudaMemPoolProps pool_props = {};
    pool_props.allocType = cudaMemAllocationTypePinned;
    pool_props.location.id = current_device_id;
    pool_props.location.type = cudaMemLocationTypeDevice;
    CHECK_CUDA_ERROR(cudaMemPoolCreate(&mem_pool_, &pool_props));

    // Configure the memory pool reusing allocation
    uint64_t pool_release_threshold = max_mem_pool_size_in_byte;
    CHECK_CUDA_ERROR(cudaMemPoolSetAttribute(
      mem_pool_, cudaMemPoolAttrReleaseThreshold, static_cast<void *>(&pool_release_threshold)));
  }
}

template <typename T>
T * CudaPointcloud2CostmapFilter::allocateBufferFromPool(size_t num_elements)
{
  T * buffer{};
  CHECK_CUDA_ERROR(cudaMallocFromPoolAsync(&buffer, num_elements * sizeof(T), mem_pool_, stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(buffer, 0, num_elements * sizeof(T), stream_));
  return buffer;
}

template <typename T>
void CudaPointcloud2CostmapFilter::returnBufferToPool(T * buffer)
{
  CHECK_CUDA_ERROR(cudaFreeAsync(buffer, stream_));
}

void CudaPointcloud2CostmapFilter::generateCostmap(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points,
  grid_map::Matrix & costmap_matrix)
{
  const size_t num_points = input_points->width * input_points->height;

  if (num_points == 0) {
    // Initialize costmap with min_value
    costmap_matrix =
      grid_map::Matrix::Constant(grid_size_x_, grid_size_y_, costmap_parameters_.grid_min_value);
    return;
  }

  // Validate that data pointer is not null
  if (!input_points->data) {
    throw std::runtime_error("Input pointcloud data pointer is null.");
  }

  // Check input format
  const bool is_point_xyzirc = (input_points->point_step == sizeof(OutputPointType));
  const bool is_point_xyzircaedt = (input_points->point_step == sizeof(InputPointType));

  if (!is_point_xyzirc && !is_point_xyzircaedt) {
    throw std::runtime_error(
      "Input pointcloud point_step does not match expected format. "
      "Expected PointXYZIRC (" +
      std::to_string(sizeof(OutputPointType)) + " bytes) or PointXYZIRCAEDT (" +
      std::to_string(sizeof(InputPointType)) + " bytes), but got " +
      std::to_string(input_points->point_step) + " bytes.");
  }

  // Allocate device buffers
  InputPointType * device_input_points = allocateBufferFromPool<InputPointType>(num_points);
  int32_t * device_grid_indices = allocateBufferFromPool<int32_t>(num_points);
  float * device_costmap_data = allocateBufferFromPool<float>(grid_size_x_ * grid_size_y_);
  CostmapGridParams * device_params = allocateBufferFromPool<CostmapGridParams>(1);

  // Copy and convert input points to device
  if (is_point_xyzirc) {
    // Convert PointXYZIRC to PointXYZIRCAEDT format
    OutputPointType * device_temp_points = allocateBufferFromPool<OutputPointType>(num_points);
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      device_temp_points, input_points->data.get(), num_points * sizeof(OutputPointType),
      cudaMemcpyDeviceToDevice, stream_));

    // Convert to InputPointType format
    const int blocks_per_grid_convert = (num_points + threads_per_block_ - 1) / threads_per_block_;
    convertPointXYZIRCToInputPointTypeLaunch(
      device_temp_points, device_input_points, static_cast<int>(num_points), threads_per_block_,
      blocks_per_grid_convert, stream_);

    // Free temporary buffer
    returnBufferToPool(device_temp_points);
  } else {
    // Direct copy for PointXYZIRCAEDT format
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      device_input_points, input_points->data.get(), num_points * sizeof(InputPointType),
      cudaMemcpyDeviceToDevice, stream_));
  }

  // Prepare costmap parameters for device
  CostmapGridParams host_params;
  host_params.grid_length_x = static_cast<float>(costmap_parameters_.grid_length_x);
  host_params.grid_length_y = static_cast<float>(costmap_parameters_.grid_length_y);
  host_params.grid_resolution = static_cast<float>(costmap_parameters_.grid_resolution);
  host_params.grid_position_x = static_cast<float>(costmap_parameters_.grid_position_x);
  host_params.grid_position_y = static_cast<float>(costmap_parameters_.grid_position_y);
  host_params.maximum_height_thres = static_cast<float>(costmap_parameters_.maximum_height_thres);
  host_params.minimum_height_thres = static_cast<float>(costmap_parameters_.minimum_height_thres);
  host_params.grid_min_value = static_cast<float>(costmap_parameters_.grid_min_value);
  host_params.grid_max_value = static_cast<float>(costmap_parameters_.grid_max_value);
  host_params.grid_size_x = grid_size_x_;
  host_params.grid_size_y = grid_size_y_;

  // Copy parameters to device
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    device_params, &host_params, sizeof(CostmapGridParams), cudaMemcpyHostToDevice, stream_));

  // Initialize grid indices to -1 (invalid)
  thrust::fill(
    thrust::cuda::par_nosync.on(stream_), thrust::device_ptr<int32_t>(device_grid_indices),
    thrust::device_ptr<int32_t>(device_grid_indices + num_points), -1);

  // Assign points to grid cells
  const int blocks_per_grid = (num_points + threads_per_block_ - 1) / threads_per_block_;
  assignPointsToGridCellsLaunch(
    device_input_points, device_grid_indices, static_cast<int>(num_points), device_params,
    threads_per_block_, blocks_per_grid, stream_);

  // Calculate costmap from grid cells
  calculateCostmapFromGridCellsLaunch(
    device_input_points, device_grid_indices, device_costmap_data, static_cast<int>(num_points),
    device_params, threads_per_block_, blocks_per_grid, stream_);

  // Synchronize stream
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  // Copy costmap data back to host
  std::vector<float> host_costmap_data(grid_size_x_ * grid_size_y_);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    host_costmap_data.data(), device_costmap_data, grid_size_x_ * grid_size_y_ * sizeof(float),
    cudaMemcpyDeviceToHost, stream_));

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  // Convert to grid_map::Matrix
  // grid_map::Matrix uses (row, col) indexing where row is x and col is y
  // (matching the CPU implementation: gridmap_data(x_ind, y_ind))
  costmap_matrix = grid_map::Matrix(grid_size_x_, grid_size_y_);
  for (int x = 0; x < grid_size_x_; ++x) {
    for (int y = 0; y < grid_size_y_; ++y) {
      // Linear index: x * grid_size_y + y
      costmap_matrix(x, y) = host_costmap_data[x * grid_size_y_ + y];
    }
  }

  // Return buffers to pool
  returnBufferToPool(device_input_points);
  returnBufferToPool(device_grid_indices);
  returnBufferToPool(device_costmap_data);
  returnBufferToPool(device_params);

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

// Explicit template instantiations
template InputPointType * CudaPointcloud2CostmapFilter::allocateBufferFromPool<InputPointType>(
  size_t);
template OutputPointType * CudaPointcloud2CostmapFilter::allocateBufferFromPool<OutputPointType>(
  size_t);
template int32_t * CudaPointcloud2CostmapFilter::allocateBufferFromPool<int32_t>(size_t);
template float * CudaPointcloud2CostmapFilter::allocateBufferFromPool<float>(size_t);
template CostmapGridParams *
CudaPointcloud2CostmapFilter::allocateBufferFromPool<CostmapGridParams>(size_t);

template void CudaPointcloud2CostmapFilter::returnBufferToPool<InputPointType>(InputPointType *);
template void CudaPointcloud2CostmapFilter::returnBufferToPool<OutputPointType>(OutputPointType *);
template void CudaPointcloud2CostmapFilter::returnBufferToPool<int32_t>(int32_t *);
template void CudaPointcloud2CostmapFilter::returnBufferToPool<float>(float *);
template void CudaPointcloud2CostmapFilter::returnBufferToPool<CostmapGridParams>(
  CostmapGridParams *);

}  // namespace autoware::cuda_pointcloud_preprocessor
