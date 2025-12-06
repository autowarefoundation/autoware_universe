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
#include "autoware/cuda_pointcloud_preprocessor/cuda_cropbox_filter/cuda_crop_box_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/types.hpp"
#include "autoware/cuda_utils/cuda_check_error.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/fill.h>
#include <thrust/scan.h>

#include <cstdint>
#include <memory>
#include <string>

namespace autoware::cuda_pointcloud_preprocessor
{

CudaCropBoxFilter::CudaCropBoxFilter(
  const CropBoxParameters crop_box_parameters, int64_t max_mem_pool_size_in_byte,
  bool output_point_xyzircaedt)
: crop_box_parameters_(crop_box_parameters), output_point_xyzircaedt_(output_point_xyzircaedt)
{
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
T * CudaCropBoxFilter::allocateBufferFromPool(size_t num_elements)
{
  T * buffer{};
  CHECK_CUDA_ERROR(cudaMallocFromPoolAsync(&buffer, num_elements * sizeof(T), mem_pool_, stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(buffer, 0, num_elements * sizeof(T), stream_));
  return buffer;
}

template <typename T>
void CudaCropBoxFilter::returnBufferToPool(T * buffer)
{
  CHECK_CUDA_ERROR(cudaFreeAsync(buffer, stream_));
}

std::unique_ptr<cuda_blackboard::CudaPointCloud2> CudaCropBoxFilter::filter(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points)
{
  const size_t num_points = input_points->width * input_points->height;

  if (num_points == 0) {
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

  // Allocate device buffers
  InputPointType * device_input_points = allocateBufferFromPool<InputPointType>(num_points);
  std::uint32_t * device_crop_mask = allocateBufferFromPool<std::uint32_t>(num_points);
  std::uint8_t * device_nan_mask = allocateBufferFromPool<std::uint8_t>(num_points);
  std::uint32_t * device_indices = allocateBufferFromPool<std::uint32_t>(num_points);
  CropBoxParameters * device_crop_box_params = allocateBufferFromPool<CropBoxParameters>(1);

  // Copy and convert input points to device
  // Note: cuda_blackboard::CudaPointCloud2::data is already on device memory when received
  // from CudaBlackboardSubscriber, so we use DeviceToDevice copy
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

  // Copy crop box parameters to device
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    device_crop_box_params, &crop_box_parameters_, sizeof(CropBoxParameters),
    cudaMemcpyHostToDevice, stream_));

  // Initialize masks
  thrust::fill(
    thrust::cuda::par_nosync.on(stream_), thrust::device_ptr<std::uint32_t>(device_crop_mask),
    thrust::device_ptr<std::uint32_t>(device_crop_mask + num_points), 0U);
  thrust::fill(
    thrust::cuda::par_nosync.on(stream_), thrust::device_ptr<std::uint8_t>(device_nan_mask),
    thrust::device_ptr<std::uint8_t>(device_nan_mask + num_points), 0);

  // Launch crop box kernel
  const int blocks_per_grid = (num_points + threads_per_block_ - 1) / threads_per_block_;
  cropBoxLaunch(
    device_input_points, device_crop_mask, device_nan_mask, static_cast<int>(num_points),
    device_crop_box_params, 1, threads_per_block_, blocks_per_grid, stream_);

  // Compute indices for output points using inclusive scan
  thrust::inclusive_scan(
    thrust::cuda::par_nosync.on(stream_), thrust::device_ptr<std::uint32_t>(device_crop_mask),
    thrust::device_ptr<std::uint32_t>(device_crop_mask + num_points),
    thrust::device_ptr<std::uint32_t>(device_indices));

  // Get number of output points
  int num_output_points = 0;
  if (num_points > 0) {
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      &num_output_points, device_indices + num_points - 1, sizeof(int), cudaMemcpyDeviceToHost,
      stream_));
  }

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  // Allocate output point cloud
  auto output = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  if (num_output_points > 0) {
    if (output_point_xyzircaedt_) {
      // Output PointXYZIRCAEDT format
      output->data =
        cuda_blackboard::make_unique<std::uint8_t[]>(num_output_points * sizeof(InputPointType));

      // Extract points directly using the already-allocated device buffers
      const int blocks_per_grid_extract =
        (num_points + threads_per_block_ - 1) / threads_per_block_;
      extractInputPointsLaunch(
        device_input_points, device_crop_mask, device_indices, static_cast<int>(num_points),
        reinterpret_cast<InputPointType *>(output->data.get()), threads_per_block_,
        blocks_per_grid_extract, stream_);
    } else {
      // Output PointXYZIRC format (default)
      output->data =
        cuda_blackboard::make_unique<std::uint8_t[]>(num_output_points * sizeof(OutputPointType));

      // Extract points directly using the already-allocated device buffers
      const int blocks_per_grid_extract =
        (num_points + threads_per_block_ - 1) / threads_per_block_;
      extractPointsLaunch(
        device_input_points, device_crop_mask, device_indices, static_cast<int>(num_points),
        reinterpret_cast<OutputPointType *>(output->data.get()), threads_per_block_,
        blocks_per_grid_extract, stream_);
    }
  } else {
    output->data.reset();
  }

  // Set output metadata
  output->header = input_points->header;
  output->width = num_output_points;
  output->height = 1;
  output->point_step = output_point_xyzircaedt_ ? sizeof(InputPointType) : sizeof(OutputPointType);
  output->row_step = num_output_points * output->point_step;
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
  returnBufferToPool(device_crop_mask);
  returnBufferToPool(device_nan_mask);
  returnBufferToPool(device_indices);
  returnBufferToPool(device_crop_box_params);

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  return output;
}

}  // namespace autoware::cuda_pointcloud_preprocessor
