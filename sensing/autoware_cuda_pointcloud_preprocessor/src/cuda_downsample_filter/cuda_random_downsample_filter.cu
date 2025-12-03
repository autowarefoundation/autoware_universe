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
#include "autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/cuda_random_downsample_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_utils/cuda_check_error.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/sequence.h>
#include <thrust/shuffle.h>

#include <cstdint>
#include <memory>
#include <random>
#include <string>

namespace autoware::cuda_pointcloud_preprocessor
{
namespace
{
__global__ void randomSampleKernel(
  const InputPointType * __restrict__ input_points, OutputPointType * __restrict__ output_points,
  const std::uint32_t * __restrict__ selected_indices, size_t num_samples)
{
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_samples) {
    return;
  }

  const std::uint32_t input_idx = selected_indices[idx];
  const InputPointType & input_point = input_points[input_idx];
  OutputPointType & output_point = output_points[idx];

  output_point.x = input_point.x;
  output_point.y = input_point.y;
  output_point.z = input_point.z;
  output_point.intensity = input_point.intensity;
  output_point.return_type = input_point.return_type;
  output_point.channel = input_point.channel;
}

__global__ void randomSampleInputKernel(
  const InputPointType * __restrict__ input_points, InputPointType * __restrict__ output_points,
  const std::uint32_t * __restrict__ selected_indices, size_t num_samples)
{
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_samples) {
    return;
  }

  const std::uint32_t input_idx = selected_indices[idx];
  // Copy all fields including extended ones
  output_points[idx] = input_points[input_idx];
}

}  // namespace

CudaRandomDownsampleFilter::CudaRandomDownsampleFilter(
  size_t sample_num, int64_t max_mem_pool_size_in_byte, bool output_point_xyzircaedt)
: sample_num_(sample_num), output_point_xyzircaedt_(output_point_xyzircaedt)
{
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

  // Create memory pool
  {
    int current_device_id = 0;
    CHECK_CUDA_ERROR(cudaGetDevice(&current_device_id));
    cudaMemPoolProps pool_props = {};
    pool_props.allocType = cudaMemAllocationTypePinned;
    pool_props.location.id = current_device_id;
    pool_props.location.type = cudaMemLocationTypeDevice;
    CHECK_CUDA_ERROR(cudaMemPoolCreate(&mem_pool_, &pool_props));

    uint64_t pool_release_threshold = max_mem_pool_size_in_byte;
    CHECK_CUDA_ERROR(cudaMemPoolSetAttribute(
      mem_pool_, cudaMemPoolAttrReleaseThreshold, static_cast<void *>(&pool_release_threshold)));
  }
}

template <typename T>
T * CudaRandomDownsampleFilter::allocateBufferFromPool(size_t num_elements)
{
  T * buffer{};
  CHECK_CUDA_ERROR(cudaMallocFromPoolAsync(&buffer, num_elements * sizeof(T), mem_pool_, stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(buffer, 0, num_elements * sizeof(T), stream_));
  return buffer;
}

template <typename T>
void CudaRandomDownsampleFilter::returnBufferToPool(T * buffer)
{
  CHECK_CUDA_ERROR(cudaFreeAsync(buffer, stream_));
}

std::unique_ptr<cuda_blackboard::CudaPointCloud2> CudaRandomDownsampleFilter::filter(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points)
{
  const size_t num_input_points = input_points->width * input_points->height;

  if (num_input_points == 0) {
    auto output = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    output->header = input_points->header;
    output->width = 0;
    output->height = 1;
    output->fields = input_points->fields;
    output->is_bigendian = input_points->is_bigendian;
    output->point_step = input_points->point_step;
    output->row_step = 0;
    output->is_dense = input_points->is_dense;
    output->data.reset();
    return output;
  }

  // Determine actual number of samples (cannot exceed input size)
  const size_t num_samples = std::min(sample_num_, num_input_points);

  if (num_samples == 0) {
    auto output = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    output->header = input_points->header;
    output->width = 0;
    output->height = 1;
    output->fields = input_points->fields;
    output->is_bigendian = input_points->is_bigendian;
    output->point_step = input_points->point_step;
    output->row_step = 0;
    output->is_dense = input_points->is_dense;
    output->data.reset();
    return output;
  }

  // Validate that data pointer is not null
  if (!input_points->data) {
    throw std::runtime_error("Input pointcloud data pointer is null.");
  }

  // Check input format and handle conversion if needed
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

  // Generate random seed
  std::random_device rd;
  std::mt19937 gen(rd());
  uint32_t seed = static_cast<uint32_t>(gen());

  // Allocate device buffers
  InputPointType * device_input_points = allocateBufferFromPool<InputPointType>(num_input_points);
  // Allocate full index array for shuffling
  std::uint32_t * device_all_indices = allocateBufferFromPool<std::uint32_t>(num_input_points);
  std::uint32_t * device_selected_indices = allocateBufferFromPool<std::uint32_t>(num_samples);

  // Copy and convert input points to device
  // Note: cuda_blackboard::CudaPointCloud2::data is already on device memory when received
  // from CudaBlackboardSubscriber, so we use DeviceToDevice copy
  if (is_point_xyzirc) {
    // Convert PointXYZIRC to PointXYZIRCAEDT format
    OutputPointType * device_temp_points =
      allocateBufferFromPool<OutputPointType>(num_input_points);
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      device_temp_points, input_points->data.get(), num_input_points * sizeof(OutputPointType),
      cudaMemcpyDeviceToDevice, stream_));

    // Convert to InputPointType format
    const int blocks_per_grid_convert =
      (num_input_points + threads_per_block_ - 1) / threads_per_block_;
    convertPointXYZIRCToInputPointTypeLaunch(
      device_temp_points, device_input_points, static_cast<int>(num_input_points),
      threads_per_block_, blocks_per_grid_convert, stream_);

    // Free temporary buffer
    returnBufferToPool(device_temp_points);
  } else {
    // Direct copy for PointXYZIRCAEDT format
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      device_input_points, input_points->data.get(), num_input_points * sizeof(InputPointType),
      cudaMemcpyDeviceToDevice, stream_));
  }

  // Generate sequence of indices [0, 1, 2, ..., num_input_points-1]
  thrust::sequence(
    thrust::cuda::par_nosync.on(stream_), thrust::device_ptr<std::uint32_t>(device_all_indices),
    thrust::device_ptr<std::uint32_t>(device_all_indices + num_input_points), 0);

  // Shuffle the indices using random number generator
  thrust::default_random_engine rng(seed);
  thrust::shuffle(
    thrust::cuda::par_nosync.on(stream_), thrust::device_ptr<std::uint32_t>(device_all_indices),
    thrust::device_ptr<std::uint32_t>(device_all_indices + num_input_points), rng);

  // Copy first num_samples indices to selected_indices
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    device_selected_indices, device_all_indices, num_samples * sizeof(std::uint32_t),
    cudaMemcpyDeviceToDevice, stream_));

  // Allocate output buffer based on output format
  void * device_output_points = nullptr;
  if (output_point_xyzircaedt_) {
    device_output_points = allocateBufferFromPool<InputPointType>(num_samples);
  } else {
    device_output_points = allocateBufferFromPool<OutputPointType>(num_samples);
  }

  // Sample points using random indices
  const int blocks_per_grid_sample = (num_samples + threads_per_block_ - 1) / threads_per_block_;
  if (output_point_xyzircaedt_) {
    randomSampleInputKernel<<<blocks_per_grid_sample, threads_per_block_, 0, stream_>>>(
      device_input_points, static_cast<InputPointType *>(device_output_points),
      device_selected_indices, num_samples);
  } else {
    randomSampleKernel<<<blocks_per_grid_sample, threads_per_block_, 0, stream_>>>(
      device_input_points, static_cast<OutputPointType *>(device_output_points),
      device_selected_indices, num_samples);
  }

  // Allocate output point cloud
  auto output = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  const size_t output_point_size =
    output_point_xyzircaedt_ ? sizeof(InputPointType) : sizeof(OutputPointType);
  output->data = cuda_blackboard::make_unique<std::uint8_t[]>(num_samples * output_point_size);

  // Copy output points from device
  // Note: cuda_blackboard::make_unique creates device memory, so we use DeviceToDevice copy
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    output->data.get(), device_output_points, num_samples * output_point_size,
    cudaMemcpyDeviceToDevice, stream_));

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  // Set output metadata
  output->header = input_points->header;
  output->width = num_samples;
  output->height = 1;
  output->point_step = output_point_size;
  output->row_step = num_samples * output_point_size;
  output->is_dense = true;
  output->is_bigendian = input_points->is_bigendian;

  // Set point fields
  auto make_point_field = [](
                            const std::string name, const uint32_t offset, const uint8_t datatype,
                            const uint32_t count) -> sensor_msgs::msg::PointField {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = count;
    return field;
  };

  output->fields.clear();
  output->fields.push_back(make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  output->fields.push_back(make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  output->fields.push_back(make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  output->fields.push_back(
    make_point_field("intensity", 12, sensor_msgs::msg::PointField::UINT8, 1));
  output->fields.push_back(
    make_point_field("return_type", 13, sensor_msgs::msg::PointField::UINT8, 1));
  output->fields.push_back(
    make_point_field("channel", 14, sensor_msgs::msg::PointField::UINT16, 1));

  if (output_point_xyzircaedt_) {
    // Add extended fields for PointXYZIRCAEDT
    output->fields.push_back(
      make_point_field("azimuth", 16, sensor_msgs::msg::PointField::FLOAT32, 1));
    output->fields.push_back(
      make_point_field("elevation", 20, sensor_msgs::msg::PointField::FLOAT32, 1));
    output->fields.push_back(
      make_point_field("distance", 24, sensor_msgs::msg::PointField::FLOAT32, 1));
    output->fields.push_back(
      make_point_field("time_stamp", 28, sensor_msgs::msg::PointField::UINT32, 1));
  }

  // Return buffers to pool
  returnBufferToPool(device_input_points);
  returnBufferToPool(device_all_indices);
  returnBufferToPool(device_selected_indices);
  if (output_point_xyzircaedt_) {
    returnBufferToPool(static_cast<InputPointType *>(device_output_points));
  } else {
    returnBufferToPool(static_cast<OutputPointType *>(device_output_points));
  }

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  return output;
}

}  // namespace autoware::cuda_pointcloud_preprocessor
