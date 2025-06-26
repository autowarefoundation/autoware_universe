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

#include "autoware/cuda_pointcloud_preprocessor/common_kernels.hpp"
#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.hpp"
#include "autoware/cuda_pointcloud_preprocessor/organize_kernels.hpp"
#include "autoware/cuda_pointcloud_preprocessor/outlier_kernels.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/undistort_kernels.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cub/cub.cuh>

#include <cuda_runtime.h>
#include <tf2/utils.h>
#include <thrust/count.h>
#include <thrust/execution_policy.h>
#include <thrust/reduce.h>
#include <thrust/scan.h>

namespace autoware::cuda_pointcloud_preprocessor
{

CudaPointcloudPreprocessor::CudaPointcloudPreprocessor()
{
  sensor_msgs::msg::PointField x_field, y_field, z_field, intensity_field, return_type_field,
    channel_field, azimuth_field, elevation_field, distance_field, time_stamp_field;
  x_field.name = "x";
  x_field.offset = 0;
  x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  x_field.count = 1;

  y_field.name = "y";
  y_field.offset = 4;
  y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  y_field.count = 1;

  z_field.name = "z";
  z_field.offset = 8;
  z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  z_field.count = 1;

  intensity_field.name = "intensity";
  intensity_field.offset = 12;
  intensity_field.datatype = sensor_msgs::msg::PointField::UINT8;
  intensity_field.count = 1;

  return_type_field.name = "return_type";
  return_type_field.offset = 13;
  return_type_field.datatype = sensor_msgs::msg::PointField::UINT8;
  return_type_field.count = 1;

  channel_field.name = "channel";
  channel_field.offset = 14;
  channel_field.datatype = sensor_msgs::msg::PointField::UINT16;
  channel_field.count = 1;

  static_assert(sizeof(OutputPointType) == 16, "OutputPointType size is not 16 bytes");
  static_assert(offsetof(OutputPointType, x) == 0);
  static_assert(offsetof(OutputPointType, y) == 4);
  static_assert(offsetof(OutputPointType, z) == 8);
  static_assert(offsetof(OutputPointType, intensity) == 12);
  static_assert(offsetof(OutputPointType, return_type) == 13);
  static_assert(offsetof(OutputPointType, channel) == 14);

  point_fields_.push_back(x_field);
  point_fields_.push_back(y_field);
  point_fields_.push_back(z_field);
  point_fields_.push_back(intensity_field);
  point_fields_.push_back(return_type_field);
  point_fields_.push_back(channel_field);

  cudaStreamCreate(&stream_);

  int num_sm;
  cudaDeviceGetAttribute(&num_sm, cudaDevAttrMultiProcessorCount, 0);
  max_blocks_per_grid_ = 4 * num_sm;  // used for strided loops

  cudaMemPoolProps pool_props;
  memset(&pool_props, 0, sizeof(cudaMemPoolProps));
  pool_props.allocType = cudaMemAllocationTypePinned;
  pool_props.handleTypes = cudaMemHandleTypePosixFileDescriptor;

  pool_props.location.type = cudaMemLocationTypeDevice;
  cudaGetDevice(&(pool_props.location.id));

  cudaMemPoolCreate(&device_memory_pool_, &pool_props);
  MemoryPoolAllocator<TwistStruct2D> allocator_2d(device_memory_pool_);
  MemoryPoolAllocator<TwistStruct3D> allocator_3d(device_memory_pool_);
  MemoryPoolAllocator<std::int32_t> allocator_int32(device_memory_pool_);
  MemoryPoolAllocator<std::uint32_t> allocator_uint32(device_memory_pool_);
  MemoryPoolAllocator<std::uint8_t> allocator_uint8(device_memory_pool_);
  MemoryPoolAllocator<InputPointType> allocator_points(device_memory_pool_);

  device_twist_2d_structs_ =
    thrust::device_vector<TwistStruct2D, MemoryPoolAllocator<TwistStruct2D>>(allocator_2d);
  device_twist_3d_structs_ =
    thrust::device_vector<TwistStruct3D, MemoryPoolAllocator<TwistStruct3D>>(allocator_3d);

  num_rings_ = 1;
  max_points_per_ring_ = 1;
  num_organized_points_ = num_rings_ * max_points_per_ring_;
  device_ring_index_ =
    thrust::device_vector<std::int32_t, MemoryPoolAllocator<std::int32_t>>(allocator_int32);
  device_ring_index_.resize(num_rings_);

  device_indexes_tensor_ =
    thrust::device_vector<std::uint32_t, MemoryPoolAllocator<std::uint32_t>>(allocator_uint32);
  device_sorted_indexes_tensor_ =
    thrust::device_vector<std::uint32_t, MemoryPoolAllocator<std::uint32_t>>(allocator_uint32);

  device_indexes_tensor_.resize(num_organized_points_);
  device_sorted_indexes_tensor_.resize(num_organized_points_);

  device_segment_offsets_ =
    thrust::device_vector<std::int32_t, MemoryPoolAllocator<std::int32_t>>(allocator_int32);
  device_segment_offsets_.resize(num_rings_ + 1);
  device_segment_offsets_[0] = 0;
  device_segment_offsets_[1] = 1;

  device_max_ring_ =
    thrust::device_vector<std::int32_t, MemoryPoolAllocator<std::int32_t>>(allocator_int32);
  device_max_ring_.resize(1);

  device_max_points_per_ring_ =
    thrust::device_vector<std::int32_t, MemoryPoolAllocator<std::int32_t>>(allocator_int32);
  device_max_points_per_ring_.resize(1);

  device_input_points_ =
    thrust::device_vector<InputPointType, MemoryPoolAllocator<InputPointType>>(allocator_points);
  device_organized_points_ =
    thrust::device_vector<InputPointType, MemoryPoolAllocator<InputPointType>>(allocator_points);
  device_organized_points_.resize(num_organized_points_);

  cudaMemsetAsync(
    thrust::raw_pointer_cast(device_max_ring_.data()), 0, sizeof(std::int32_t), stream_);
  cudaMemsetAsync(
    thrust::raw_pointer_cast(device_max_points_per_ring_.data()), 0, sizeof(std::int32_t), stream_);
  cudaMemsetAsync(
    thrust::raw_pointer_cast(device_indexes_tensor_.data()), 0x255, sizeof(std::uint32_t), stream_);

  sort_workspace_bytes_ = querySortWorkspace(
    num_rings_ * max_points_per_ring_, num_rings_,
    thrust::raw_pointer_cast(device_segment_offsets_.data()),
    thrust::raw_pointer_cast(device_indexes_tensor_.data()),
    thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()));

  device_sort_workspace_ =
    thrust::device_vector<std::uint8_t, MemoryPoolAllocator<std::uint8_t>>(allocator_uint8);

  device_transformed_points_.resize(num_organized_points_);
  device_crop_mask_.resize(num_organized_points_);
  device_ring_outlier_mask_.resize(num_organized_points_);
  device_indices_.resize(num_organized_points_);

  preallocateOutput();
}

void CudaPointcloudPreprocessor::setCropBoxParameters(
  const std::vector<CropBoxParameters> & crop_box_parameters)
{
  host_crop_box_structs_ = crop_box_parameters;
  device_crop_box_structs_ = host_crop_box_structs_;
}

void CudaPointcloudPreprocessor::setRingOutlierFilterParameters(
  const RingOutlierFilterParameters & ring_outlier_parameters)
{
  ring_outlier_parameters_ = ring_outlier_parameters;
}

void CudaPointcloudPreprocessor::setUndistortionType(const UndistortionType & undistortion_type)
{
  if (undistortion_type == UndistortionType::Invalid) {
    throw std::runtime_error("Invalid undistortion type");
  }

  undistortion_type_ = undistortion_type;
}

void CudaPointcloudPreprocessor::preallocateOutput()
{
  output_pointcloud_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  output_pointcloud_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
    num_rings_ * max_points_per_ring_ * sizeof(OutputPointType));
}

void CudaPointcloudPreprocessor::organizePointcloud()
{
  cudaMemsetAsync(
    thrust::raw_pointer_cast(device_ring_index_.data()), 0, num_rings_ * sizeof(std::int32_t),
    stream_);
  cudaMemsetAsync(
    thrust::raw_pointer_cast(device_indexes_tensor_.data()), num_raw_points_,
    num_organized_points_ * sizeof(std::uint32_t), stream_);

  if (num_raw_points_ == 0) {
    output_pointcloud_ptr_->data.reset();
    return;
  }

  const int raw_points_blocks_per_grid =
    (num_raw_points_ + threads_per_block_ - 1) / threads_per_block_;

  organizeLaunch(
    thrust::raw_pointer_cast(device_input_points_.data()),
    thrust::raw_pointer_cast(device_indexes_tensor_.data()),
    thrust::raw_pointer_cast(device_ring_index_.data()), num_rings_,
    thrust::raw_pointer_cast(device_max_ring_.data()), max_points_per_ring_,
    thrust::raw_pointer_cast(device_max_points_per_ring_.data()), num_raw_points_,
    threads_per_block_, raw_points_blocks_per_grid, stream_);

  std::int32_t max_ring_value;
  std::int32_t max_points_per_ring;

  cudaMemcpyAsync(
    &max_ring_value, thrust::raw_pointer_cast(device_max_ring_.data()), sizeof(std::int32_t),
    cudaMemcpyDeviceToHost, stream_);
  cudaMemcpyAsync(
    &max_points_per_ring, thrust::raw_pointer_cast(device_max_points_per_ring_.data()),
    sizeof(std::int32_t), cudaMemcpyDeviceToHost, stream_);
  cudaStreamSynchronize(stream_);

  if (max_ring_value >= num_rings_ || max_points_per_ring > max_points_per_ring_) {
    num_rings_ = max_ring_value + 1;
    max_points_per_ring_ = std::max((max_points_per_ring + 511) / 512 * 512, 512);
    num_organized_points_ = num_rings_ * max_points_per_ring_;

    device_ring_index_.resize(num_rings_);
    thrust::fill(device_ring_index_.begin(), device_ring_index_.end(), 0);
    device_indexes_tensor_.resize(num_organized_points_);
    thrust::fill(device_indexes_tensor_.begin(), device_indexes_tensor_.end(), num_raw_points_);
    device_sorted_indexes_tensor_.resize(num_organized_points_);
    thrust::fill(
      device_sorted_indexes_tensor_.begin(), device_sorted_indexes_tensor_.end(), num_raw_points_);
    device_segment_offsets_.resize(num_rings_ + 1);
    device_organized_points_.resize(num_organized_points_);
    thrust::fill(
      device_organized_points_.begin(), device_organized_points_.end(), InputPointType{});

    device_transformed_points_.resize(num_organized_points_);
    thrust::fill(
      device_transformed_points_.begin(), device_transformed_points_.end(), InputPointType{});
    device_crop_mask_.resize(num_organized_points_);
    thrust::fill(device_crop_mask_.begin(), device_crop_mask_.end(), 0);
    device_nan_mask_.resize(num_organized_points_);
    thrust::fill(device_nan_mask_.begin(), device_nan_mask_.end(), 0);
    device_mismatch_mask_.resize(num_organized_points_);
    thrust::fill(device_mismatch_mask_.begin(), device_mismatch_mask_.end(), 0);
    device_ring_outlier_mask_.resize(num_organized_points_);
    thrust::fill(device_ring_outlier_mask_.begin(), device_ring_outlier_mask_.end(), 0);
    device_indices_.resize(num_organized_points_);
    thrust::fill(device_indices_.begin(), device_indices_.end(), 0);

    preallocateOutput();

    std::vector<std::int32_t> segment_offsets_host(num_rings_ + 1);
    for (std::size_t i = 0; i < num_rings_ + 1; i++) {
      segment_offsets_host[i] = i * max_points_per_ring_;
    }

    cudaMemcpyAsync(
      thrust::raw_pointer_cast(device_segment_offsets_.data()), segment_offsets_host.data(),
      (num_rings_ + 1) * sizeof(std::int32_t), cudaMemcpyHostToDevice, stream_);

    cudaMemsetAsync(
      thrust::raw_pointer_cast(device_ring_index_.data()), 0, num_rings_ * sizeof(std::int32_t),
      stream_);
    cudaMemsetAsync(
      thrust::raw_pointer_cast(device_indexes_tensor_.data()), 0xFF,
      num_organized_points_ * sizeof(std::int32_t), stream_);

    sort_workspace_bytes_ = querySortWorkspace(
      num_organized_points_, num_rings_, thrust::raw_pointer_cast(device_segment_offsets_.data()),
      thrust::raw_pointer_cast(device_indexes_tensor_.data()),
      thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()));
    device_sort_workspace_.resize(sort_workspace_bytes_);

    organizeLaunch(
      thrust::raw_pointer_cast(device_input_points_.data()),
      thrust::raw_pointer_cast(device_indexes_tensor_.data()),
      thrust::raw_pointer_cast(device_ring_index_.data()), num_rings_,
      thrust::raw_pointer_cast(device_max_ring_.data()), max_points_per_ring_,
      thrust::raw_pointer_cast(device_max_points_per_ring_.data()), num_raw_points_,
      threads_per_block_, raw_points_blocks_per_grid, stream_);
  }

  if (num_organized_points_ == num_rings_) {
    cudaMemcpyAsync(
      thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()),
      thrust::raw_pointer_cast(device_indexes_tensor_.data()),
      num_organized_points_ * sizeof(std::uint32_t), cudaMemcpyDeviceToDevice, stream_);
  } else {
    cub::DeviceSegmentedRadixSort::SortKeys(
      reinterpret_cast<void *>(thrust::raw_pointer_cast(device_sort_workspace_.data())),
      sort_workspace_bytes_, thrust::raw_pointer_cast(device_indexes_tensor_.data()),
      thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()), num_organized_points_,
      num_rings_, thrust::raw_pointer_cast(device_segment_offsets_.data()),
      thrust::raw_pointer_cast(device_segment_offsets_.data()) + 1, 0, sizeof(std::uint32_t) * 8,
      stream_);
  }

  // reuse device_indexes_tensor_ to store valid point location
  thrust::fill(device_indexes_tensor_.begin(), device_indexes_tensor_.end(), 0);

  const int organized_points_blocks_per_grid =
    (num_organized_points_ + threads_per_block_ - 1) / threads_per_block_;

  gatherLaunch(
    thrust::raw_pointer_cast(device_input_points_.data()),
    thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()),
    thrust::raw_pointer_cast(device_organized_points_.data()), num_rings_, max_points_per_ring_,
    thrust::raw_pointer_cast(device_indexes_tensor_.data()), num_raw_points_, threads_per_block_,
    organized_points_blocks_per_grid, stream_);
}

std::unique_ptr<cuda_blackboard::CudaPointCloud2> CudaPointcloudPreprocessor::process(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
  const geometry_msgs::msg::TransformStamped & transform_msg,
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
  const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue,
  const std::uint32_t first_point_rel_stamp_nsec)
{
  auto frame_id = input_pointcloud_msg.header.frame_id;
  num_raw_points_ = input_pointcloud_msg.width * input_pointcloud_msg.height;
  num_organized_points_ = num_rings_ * max_points_per_ring_;

  if (num_raw_points_ == 0) {
    output_pointcloud_ptr_->row_step = 0;
    output_pointcloud_ptr_->width = 0;
    output_pointcloud_ptr_->height = 1;

    output_pointcloud_ptr_->fields = point_fields_;
    output_pointcloud_ptr_->is_dense = true;
    output_pointcloud_ptr_->is_bigendian = input_pointcloud_msg.is_bigendian;
    output_pointcloud_ptr_->point_step = sizeof(OutputPointType);
    output_pointcloud_ptr_->header.stamp = input_pointcloud_msg.header.stamp;

    return std::move(output_pointcloud_ptr_);
  }

  if (num_raw_points_ > device_input_points_.size()) {
    std::size_t new_capacity = (num_raw_points_ + 1024) / 1024 * 1024;
    device_input_points_.resize(new_capacity);
  }

  // Reset all contents in the device vector
  thrust::fill(device_input_points_.begin(), device_input_points_.end(), InputPointType{});
  thrust::fill(device_organized_points_.begin(), device_organized_points_.end(), InputPointType{});

  cudaMemcpyAsync(
    thrust::raw_pointer_cast(device_input_points_.data()), input_pointcloud_msg.data.data(),
    num_raw_points_ * sizeof(InputPointType), cudaMemcpyHostToDevice, stream_);

  cudaStreamSynchronize(stream_);

  organizePointcloud();

  // Reset all contents in the device vector
  thrust::fill(
    device_transformed_points_.begin(), device_transformed_points_.end(), InputPointType{});
  thrust::fill(device_ring_outlier_mask_.begin(), device_ring_outlier_mask_.end(), 0);
  thrust::fill(device_mismatch_mask_.begin(), device_mismatch_mask_.end(), 0);
  thrust::fill(device_nan_mask_.begin(), device_nan_mask_.end(), 0);
  thrust::fill(device_crop_mask_.begin(), device_crop_mask_.end(), 0);

  tf2::Quaternion rotation_quaternion(
    transform_msg.transform.rotation.x, transform_msg.transform.rotation.y,
    transform_msg.transform.rotation.z, transform_msg.transform.rotation.w);
  tf2::Matrix3x3 rotation_matrix;
  rotation_matrix.setRotation(rotation_quaternion);

  TransformStruct transform_struct;
  transform_struct.x = static_cast<float>(transform_msg.transform.translation.x);
  transform_struct.y = static_cast<float>(transform_msg.transform.translation.y);
  transform_struct.z = static_cast<float>(transform_msg.transform.translation.z);
  transform_struct.m11 = static_cast<float>(rotation_matrix[0][0]);
  transform_struct.m12 = static_cast<float>(rotation_matrix[0][1]);
  transform_struct.m13 = static_cast<float>(rotation_matrix[0][2]);
  transform_struct.m21 = static_cast<float>(rotation_matrix[1][0]);
  transform_struct.m22 = static_cast<float>(rotation_matrix[1][1]);
  transform_struct.m23 = static_cast<float>(rotation_matrix[1][2]);
  transform_struct.m31 = static_cast<float>(rotation_matrix[2][0]);
  transform_struct.m32 = static_cast<float>(rotation_matrix[2][1]);
  transform_struct.m33 = static_cast<float>(rotation_matrix[2][2]);

  // Twist preprocessing
  std::uint64_t pointcloud_stamp_nsec = 1'000'000'000 * input_pointcloud_msg.header.stamp.sec +
                                        input_pointcloud_msg.header.stamp.nanosec;

  if (undistortion_type_ == UndistortionType::Undistortion3D) {
    setupTwist3DStructs(
      twist_queue, angular_velocity_queue, pointcloud_stamp_nsec, first_point_rel_stamp_nsec,
      device_twist_3d_structs_, stream_);
  } else if (undistortion_type_ == UndistortionType::Undistortion2D) {
    setupTwist2DStructs(
      twist_queue, angular_velocity_queue, pointcloud_stamp_nsec, first_point_rel_stamp_nsec,
      device_twist_2d_structs_, stream_);
  } else {
    throw std::runtime_error("Invalid undistortion type");
  }

  // Obtain raw pointers for the kernels
  TwistStruct2D * device_twist_2d_structs =
    thrust::raw_pointer_cast(device_twist_2d_structs_.data());
  TwistStruct3D * device_twist_3d_structs =
    thrust::raw_pointer_cast(device_twist_3d_structs_.data());
  InputPointType * device_transformed_points =
    thrust::raw_pointer_cast(device_transformed_points_.data());
  std::uint32_t * device_crop_mask = thrust::raw_pointer_cast(device_crop_mask_.data());
  std::uint8_t * device_nan_mask = thrust::raw_pointer_cast(device_nan_mask_.data());
  std::uint8_t * device_mismatch_mask = thrust::raw_pointer_cast(device_mismatch_mask_.data());
  std::uint32_t * device_ring_outlier_mask =
    thrust::raw_pointer_cast(device_ring_outlier_mask_.data());
  std::uint32_t * device_indices = thrust::raw_pointer_cast(device_indices_.data());
  std::uint32_t * device_is_valid_point = thrust::raw_pointer_cast(device_indexes_tensor_.data());

  const int blocks_per_grid = (num_organized_points_ + threads_per_block_ - 1) / threads_per_block_;

  transformPointsLaunch(
    thrust::raw_pointer_cast(device_organized_points_.data()), device_transformed_points,
    num_organized_points_, transform_struct, threads_per_block_, blocks_per_grid, stream_);

  // Crop box filter
  int crop_box_blocks_per_grid = std::min(blocks_per_grid, max_blocks_per_grid_);
  if (host_crop_box_structs_.size() > 0) {
    cropBoxLaunch(
      device_transformed_points, device_crop_mask, device_nan_mask, num_organized_points_,
      thrust::raw_pointer_cast(device_crop_box_structs_.data()), host_crop_box_structs_.size(),
      crop_box_blocks_per_grid, threads_per_block_, stream_);
  } else {
    thrust::fill(thrust::device, device_crop_mask, device_crop_mask + num_organized_points_, 1);
  }

  // Undistortion
  if (
    undistortion_type_ == UndistortionType::Undistortion3D && device_twist_3d_structs_.size() > 0) {
    undistort3DLaunch(
      device_transformed_points, num_organized_points_, device_twist_3d_structs,
      device_twist_3d_structs_.size(), device_mismatch_mask, threads_per_block_, blocks_per_grid,
      stream_);
  } else if (
    undistortion_type_ == UndistortionType::Undistortion2D && device_twist_2d_structs_.size() > 0) {
    undistort2DLaunch(
      device_transformed_points, num_organized_points_, device_twist_2d_structs,
      device_twist_2d_structs_.size(), device_mismatch_mask, threads_per_block_, blocks_per_grid,
      stream_);
  }

  // Ring outlier
  ringOutlierFilterLaunch(
    device_transformed_points, device_ring_outlier_mask, num_rings_, max_points_per_ring_,
    ring_outlier_parameters_.distance_ratio,
    ring_outlier_parameters_.object_length_threshold *
      ring_outlier_parameters_.object_length_threshold,
    threads_per_block_, blocks_per_grid, stream_);

  combineMasksLaunch(
    device_crop_mask, device_ring_outlier_mask, num_organized_points_, device_ring_outlier_mask,
    threads_per_block_, blocks_per_grid, stream_);

  // Mask out invalid points in the array
  combineMasksLaunch(
    device_is_valid_point, device_ring_outlier_mask, num_organized_points_,
    device_ring_outlier_mask, threads_per_block_, blocks_per_grid, stream_);

  thrust::inclusive_scan(
    thrust::device, device_ring_outlier_mask, device_ring_outlier_mask + num_organized_points_,
    device_indices);

  std::uint32_t num_output_points;
  cudaMemcpyAsync(
    &num_output_points, device_indices + num_organized_points_ - 1, sizeof(std::uint32_t),
    cudaMemcpyDeviceToHost, stream_);

  cudaStreamSynchronize(stream_);

  // Get information and extract points after filters
  std::uint32_t num_crop_box_passed_points =
    thrust::count(thrust::device, device_crop_mask_.begin(), device_crop_mask_.end(), 1);

  std::uint32_t num_nan_points =
    thrust::count(thrust::device, device_nan_mask_.begin(), device_nan_mask_.end(), 1);

  std::uint32_t mismatch_count =
    thrust::count(thrust::device, device_mismatch_mask_.begin(), device_mismatch_mask_.end(), 1);

  stats_.num_nan_points = static_cast<int>(num_nan_points);
  stats_.num_crop_box_passed_points = static_cast<int>(num_crop_box_passed_points);
  stats_.mismatch_count = static_cast<int>(mismatch_count);

  if (num_output_points > 0) {
    extractPointsLaunch(
      device_transformed_points, device_ring_outlier_mask, device_indices, num_organized_points_,
      reinterpret_cast<OutputPointType *>(output_pointcloud_ptr_->data.get()), threads_per_block_,
      blocks_per_grid, stream_);
  }

  cudaStreamSynchronize(stream_);

  // Copy the transformed points back
  output_pointcloud_ptr_->row_step = num_output_points * sizeof(OutputPointType);
  output_pointcloud_ptr_->width = num_output_points;
  output_pointcloud_ptr_->height = 1;

  output_pointcloud_ptr_->fields = point_fields_;
  output_pointcloud_ptr_->is_dense = true;
  output_pointcloud_ptr_->is_bigendian = input_pointcloud_msg.is_bigendian;
  output_pointcloud_ptr_->point_step = sizeof(OutputPointType);
  output_pointcloud_ptr_->header.stamp = input_pointcloud_msg.header.stamp;

  return std::move(output_pointcloud_ptr_);
}

}  // namespace autoware::cuda_pointcloud_preprocessor
