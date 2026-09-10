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
#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <cub/cub.cuh>

#include <sensor_msgs/msg/point_field.hpp>

#include <cuda_runtime.h>
#include <tf2/utils.h>
#include <thrust/iterator/transform_iterator.h>

#include <algorithm>
#include <cstdint>
#include <stdexcept>

namespace autoware::cuda_pointcloud_preprocessor
{

template <typename T>
__global__ void fillKernel(T * output, T value, std::size_t count)
{
  for (std::size_t idx = blockIdx.x * blockDim.x + threadIdx.x; idx < count;
       idx += blockDim.x * gridDim.x) {
    output[idx] = value;
  }
}

template <typename T>
void fillDeviceVectorPrefix(
  thrust::device_vector<T> & vector, std::size_t count, T value, int threads_per_block,
  int blocks_per_grid, cudaStream_t stream)
{
  if (count == 0) {
    return;
  }
  fillKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    thrust::raw_pointer_cast(vector.data()), value, count);
  CHECK_CUDA_ERROR(cudaGetLastError());
}

template <typename T>
void fillDeviceVector(
  thrust::device_vector<T> & vector, T value, int threads_per_block, int blocks_per_grid,
  cudaStream_t stream)
{
  fillDeviceVectorPrefix(vector, vector.size(), value, threads_per_block, blocks_per_grid, stream);
}

// Counting is done with thrust's transform_iterator rather than
// cub::TransformInputIterator, which CCCL 3.0 (CUDA 13) removed.
template <typename T>
struct EqualsValue
{
  T value;

  __host__ __device__ std::uint32_t operator()(const T & input) const
  {
    return input == value ? 1U : 0U;
  }
};

template <typename T>
std::size_t queryCountWorkspace(
  const T * input, std::uint32_t * output, std::size_t count, T value, cudaStream_t stream)
{
  void * workspace = nullptr;
  std::size_t workspace_bytes = 0;
  auto transform_iterator = thrust::make_transform_iterator(input, EqualsValue<T>{value});
  CHECK_CUDA_ERROR(
    cub::DeviceReduce::Sum(workspace, workspace_bytes, transform_iterator, output, count, stream));
  return workspace_bytes;
}

template <typename T>
void countEqualAsync(
  void * workspace, std::size_t workspace_bytes, const T * input, std::uint32_t * output,
  std::size_t count, T value, cudaStream_t stream)
{
  auto transform_iterator = thrust::make_transform_iterator(input, EqualsValue<T>{value});
  CHECK_CUDA_ERROR(
    cub::DeviceReduce::Sum(workspace, workspace_bytes, transform_iterator, output, count, stream));
}

namespace
{

PreprocessorCapacity validate_capacity(const PreprocessorCapacity & capacity)
{
  if (
    capacity.max_input_point_count == 0 || capacity.max_ring_count <= 0 ||
    capacity.max_points_per_ring <= 0 || capacity.max_twist_struct_count == 0) {
    throw std::runtime_error("CudaPointcloudPreprocessor capacities must be positive");
  }
  return capacity;
}

}  // namespace

CudaPointcloudPreprocessor::CudaPointcloudPreprocessor(const PreprocessorCapacity & capacity)
: capacity_(validate_capacity(capacity)),
  num_rings_(capacity_.max_ring_count),
  max_points_per_ring_(capacity_.max_points_per_ring),
  num_organized_points_(static_cast<std::size_t>(num_rings_) * max_points_per_ring_),
  stream_(initialize_stream())
{
  using sensor_msgs::msg::PointField;

  auto make_point_field = [](
                            const std::string & name, std::size_t offset,
                            sensor_msgs::msg::PointField::_datatype_type datatype,
                            std::size_t count) {
    PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = count;
    return field;
  };

  point_fields_ = {
    make_point_field("x", 0, PointField::FLOAT32, 1),
    make_point_field("y", 4, PointField::FLOAT32, 1),
    make_point_field("z", 8, PointField::FLOAT32, 1),
    make_point_field("intensity", 12, PointField::UINT8, 1),
    make_point_field("return_type", 13, PointField::UINT8, 1),
    make_point_field("channel", 14, PointField::UINT16, 1),
  };

  int num_sm{};
  CHECK_CUDA_ERROR(cudaDeviceGetAttribute(&num_sm, cudaDevAttrMultiProcessorCount, 0));
  max_blocks_per_grid_ = 4 * num_sm;  // used for strided loops

  initializeBuffers();
}
cudaStream_t CudaPointcloudPreprocessor::initialize_stream()
{
  cudaStream_t stream{};
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream));
  return stream;
}

void CudaPointcloudPreprocessor::setCropBoxParameters(
  const std::vector<CropBoxParameters> & crop_box_parameters)
{
  device_crop_box_structs_ = crop_box_parameters;
}

void CudaPointcloudPreprocessor::setRingOutlierFilterParameters(
  const RingOutlierFilterParameters & ring_outlier_parameters)
{
  ring_outlier_parameters_ = ring_outlier_parameters;
}

void CudaPointcloudPreprocessor::setRingOutlierFilterActive(const bool enable_filter)
{
  enable_ring_outlier_filter_ = enable_filter;
}

void CudaPointcloudPreprocessor::setUndistortionType(const UndistortionType & undistortion_type)
{
  if (undistortion_type == UndistortionType::Invalid) {
    throw std::runtime_error("Invalid undistortion type");
  }

  undistortion_type_ = undistortion_type;
}

void CudaPointcloudPreprocessor::initializeBuffers()
{
  device_input_points_.resize(capacity_.max_input_point_count);
  device_ring_index_.resize(num_rings_);
  device_indexes_tensor_.resize(num_organized_points_);
  device_sorted_indexes_tensor_.resize(num_organized_points_);
  device_segment_offsets_.resize(num_rings_ + 1);
  device_max_ring_.resize(1);
  device_max_points_per_ring_.resize(1);
  device_organized_points_.resize(num_organized_points_);
  device_transformed_points_.resize(num_organized_points_);
  device_crop_mask_.resize(num_organized_points_);
  device_nan_mask_.resize(num_organized_points_);
  device_mismatch_mask_.resize(num_organized_points_);
  device_ring_outlier_mask_.resize(num_organized_points_);
  device_indices_.resize(num_organized_points_);
  device_twist_2d_structs_.resize(capacity_.max_twist_struct_count);
  device_twist_3d_structs_.resize(capacity_.max_twist_struct_count);
  device_processing_stats_.resize(processing_stat_count);

  std::vector<std::int32_t> segment_offsets_host(num_rings_ + 1);
  for (int i = 0; i < num_rings_ + 1; i++) {
    segment_offsets_host[i] = i * max_points_per_ring_;
  }
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    thrust::raw_pointer_cast(device_segment_offsets_.data()), segment_offsets_host.data(),
    segment_offsets_host.size() * sizeof(std::int32_t), cudaMemcpyHostToDevice, stream_));

  fillDeviceVector(
    device_max_ring_, std::int32_t{}, threads_per_block_, max_blocks_per_grid_, stream_);
  fillDeviceVector(
    device_max_points_per_ring_, std::int32_t{}, threads_per_block_, max_blocks_per_grid_, stream_);
  fillDeviceVector(
    device_indexes_tensor_, UINT32_MAX, threads_per_block_, max_blocks_per_grid_, stream_);

  const auto sort_workspace_bytes = querySortWorkspace(
    num_organized_points_, num_rings_, thrust::raw_pointer_cast(device_segment_offsets_.data()),
    thrust::raw_pointer_cast(device_indexes_tensor_.data()),
    thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()), stream_);
  std::size_t scan_workspace_bytes{};
  CHECK_CUDA_ERROR(
    cub::DeviceScan::InclusiveSum(
      nullptr, scan_workspace_bytes, thrust::raw_pointer_cast(device_ring_outlier_mask_.data()),
      thrust::raw_pointer_cast(device_indices_.data()), num_organized_points_, stream_));
  const auto reduce_workspace_bytes = std::max(
    {queryCountWorkspace(
       thrust::raw_pointer_cast(device_crop_mask_.data()),
       thrust::raw_pointer_cast(device_processing_stats_.data()), num_organized_points_, 1U,
       stream_),
     queryCountWorkspace(
       thrust::raw_pointer_cast(device_nan_mask_.data()),
       thrust::raw_pointer_cast(device_processing_stats_.data()), num_organized_points_,
       static_cast<std::uint8_t>(1), stream_),
     queryCountWorkspace(
       thrust::raw_pointer_cast(device_mismatch_mask_.data()),
       thrust::raw_pointer_cast(device_processing_stats_.data()), num_organized_points_,
       static_cast<std::uint8_t>(1), stream_)});
  workspace_bytes_ = std::max({sort_workspace_bytes, scan_workspace_bytes, reduce_workspace_bytes});
  device_scratch_workspace_.resize(workspace_bytes_);

  preallocateOutput();
}

void CudaPointcloudPreprocessor::preallocateOutput()
{
  output_pointcloud_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  output_pointcloud_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
    num_rings_ * max_points_per_ring_ * sizeof(OutputPointType));
}

void CudaPointcloudPreprocessor::organizePointcloud()
{
  fillDeviceVectorPrefix(
    device_ring_index_, num_rings_, std::int32_t{}, threads_per_block_, max_blocks_per_grid_,
    stream_);
  fillDeviceVectorPrefix(
    device_indexes_tensor_, num_organized_points_, static_cast<std::uint32_t>(num_raw_points_),
    threads_per_block_, max_blocks_per_grid_, stream_);
  fillDeviceVector(
    device_max_ring_, std::int32_t{}, threads_per_block_, max_blocks_per_grid_, stream_);
  fillDeviceVector(
    device_max_points_per_ring_, std::int32_t{}, threads_per_block_, max_blocks_per_grid_, stream_);

  if (num_raw_points_ == 0) {
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

  CHECK_CUDA_ERROR(
    cub::DeviceSegmentedRadixSort::SortKeys(
      reinterpret_cast<void *>(
        thrust::raw_pointer_cast(device_scratch_workspace_.data())),  // NOLINT
      workspace_bytes_, thrust::raw_pointer_cast(device_indexes_tensor_.data()),
      thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()), num_organized_points_,
      num_rings_, thrust::raw_pointer_cast(device_segment_offsets_.data()),
      thrust::raw_pointer_cast(device_segment_offsets_.data()) + 1, 0, sizeof(std::uint32_t) * 8,
      stream_));

  // reuse device_indexes_tensor_ to store valid point location
  fillDeviceVector(device_indexes_tensor_, 0U, threads_per_block_, max_blocks_per_grid_, stream_);

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
  const auto input_point_count =
    static_cast<std::size_t>(input_pointcloud_msg.width) * input_pointcloud_msg.height;
  num_raw_points_ = std::min(input_point_count, capacity_.max_input_point_count);

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

  // Reset all contents in the device vector
  fillDeviceVector(
    device_input_points_, InputPointType{}, threads_per_block_, max_blocks_per_grid_, stream_);
  fillDeviceVector(
    device_organized_points_, InputPointType{}, threads_per_block_, max_blocks_per_grid_, stream_);

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    thrust::raw_pointer_cast(device_input_points_.data()), input_pointcloud_msg.data.data(),
    num_raw_points_ * sizeof(InputPointType), cudaMemcpyHostToDevice, stream_));

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  organizePointcloud();

  // Reset all contents in the device vector
  fillDeviceVector(
    device_transformed_points_, InputPointType{}, threads_per_block_, max_blocks_per_grid_,
    stream_);
  fillDeviceVector(
    device_ring_outlier_mask_, 0U, threads_per_block_, max_blocks_per_grid_, stream_);
  fillDeviceVector(
    device_mismatch_mask_, static_cast<std::uint8_t>(0), threads_per_block_, max_blocks_per_grid_,
    stream_);
  fillDeviceVector(
    device_nan_mask_, static_cast<std::uint8_t>(0), threads_per_block_, max_blocks_per_grid_,
    stream_);
  fillDeviceVector(device_crop_mask_, 0U, threads_per_block_, max_blocks_per_grid_, stream_);
  fillDeviceVector(device_processing_stats_, 0U, threads_per_block_, max_blocks_per_grid_, stream_);

  tf2::Quaternion rotation_quaternion(
    transform_msg.transform.rotation.x, transform_msg.transform.rotation.y,
    transform_msg.transform.rotation.z, transform_msg.transform.rotation.w);
  tf2::Matrix3x3 rotation_matrix;
  rotation_matrix.setRotation(rotation_quaternion);

  TransformStruct transform_struct{};
  transform_struct.x = static_cast<float>(transform_msg.transform.translation.x);
  transform_struct.y = static_cast<float>(transform_msg.transform.translation.y);
  transform_struct.z = static_cast<float>(transform_msg.transform.translation.z);
  transform_struct.m11 = static_cast<float>(rotation_matrix.getRow(0).getX());
  transform_struct.m12 = static_cast<float>(rotation_matrix.getRow(0).getY());
  transform_struct.m13 = static_cast<float>(rotation_matrix.getRow(0).getZ());
  transform_struct.m21 = static_cast<float>(rotation_matrix.getRow(1).getX());
  transform_struct.m22 = static_cast<float>(rotation_matrix.getRow(1).getY());
  transform_struct.m23 = static_cast<float>(rotation_matrix.getRow(1).getZ());
  transform_struct.m31 = static_cast<float>(rotation_matrix.getRow(2).getX());
  transform_struct.m32 = static_cast<float>(rotation_matrix.getRow(2).getY());
  transform_struct.m33 = static_cast<float>(rotation_matrix.getRow(2).getZ());

  // Twist preprocessing
  std::uint64_t pointcloud_stamp_nsec =
    static_cast<std::uint64_t>(1'000'000'000) * input_pointcloud_msg.header.stamp.sec +
    input_pointcloud_msg.header.stamp.nanosec;

  std::size_t active_twist_2d_struct_count{};
  std::size_t active_twist_3d_struct_count{};
  if (undistortion_type_ == UndistortionType::Undistortion3D) {
    active_twist_3d_struct_count = setupTwist3DStructs(
      twist_queue, angular_velocity_queue, pointcloud_stamp_nsec, first_point_rel_stamp_nsec,
      device_twist_3d_structs_, stream_);
  } else if (undistortion_type_ == UndistortionType::Undistortion2D) {
    active_twist_2d_struct_count = setupTwist2DStructs(
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
  if (device_crop_box_structs_.size() > 0) {
    cropBoxLaunch(
      device_transformed_points, device_crop_mask, device_nan_mask, num_organized_points_,
      thrust::raw_pointer_cast(device_crop_box_structs_.data()),
      static_cast<int>(device_crop_box_structs_.size()), crop_box_blocks_per_grid,
      threads_per_block_, stream_);
  } else {
    fillDeviceVectorPrefix(
      device_crop_mask_, num_organized_points_, 1U, threads_per_block_, max_blocks_per_grid_,
      stream_);
  }

  // Undistortion
  if (undistortion_type_ == UndistortionType::Undistortion3D && active_twist_3d_struct_count > 0) {
    undistort3DLaunch(
      device_transformed_points, num_organized_points_, device_twist_3d_structs,
      static_cast<int>(active_twist_3d_struct_count), device_mismatch_mask, threads_per_block_,
      blocks_per_grid, stream_);
  } else if (
    undistortion_type_ == UndistortionType::Undistortion2D && active_twist_2d_struct_count > 0) {
    undistort2DLaunch(
      device_transformed_points, num_organized_points_, device_twist_2d_structs,
      static_cast<int>(active_twist_2d_struct_count), device_mismatch_mask, threads_per_block_,
      blocks_per_grid, stream_);
  }

  // Ring outlier
  if (enable_ring_outlier_filter_) {
    ringOutlierFilterLaunch(
      device_transformed_points, device_ring_outlier_mask, num_rings_, max_points_per_ring_,
      ring_outlier_parameters_.distance_ratio,
      ring_outlier_parameters_.object_length_threshold *
        ring_outlier_parameters_.object_length_threshold,
      threads_per_block_, blocks_per_grid, stream_);
  } else {
    fillDeviceVectorPrefix(
      device_ring_outlier_mask_, num_organized_points_, 1U, threads_per_block_,
      max_blocks_per_grid_, stream_);
  }

  combineMasksLaunch(
    device_crop_mask, device_ring_outlier_mask, num_organized_points_, device_ring_outlier_mask,
    threads_per_block_, blocks_per_grid, stream_);

  // Mask out invalid points in the array
  combineMasksLaunch(
    device_is_valid_point, device_ring_outlier_mask, num_organized_points_,
    device_ring_outlier_mask, threads_per_block_, blocks_per_grid, stream_);

  CHECK_CUDA_ERROR(
    cub::DeviceScan::InclusiveSum(
      reinterpret_cast<void *>(thrust::raw_pointer_cast(device_scratch_workspace_.data())),
      workspace_bytes_, device_ring_outlier_mask, device_indices, num_organized_points_, stream_));

  int num_output_points{};
  std::int32_t max_ring_value{};
  std::int32_t max_points_per_ring_value{};
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &num_output_points, device_indices + num_organized_points_ - 1, sizeof(int),
    cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &max_ring_value, thrust::raw_pointer_cast(device_max_ring_.data()), sizeof(std::int32_t),
    cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &max_points_per_ring_value, thrust::raw_pointer_cast(device_max_points_per_ring_.data()),
    sizeof(std::int32_t), cudaMemcpyDeviceToHost, stream_));

  countEqualAsync(
    reinterpret_cast<void *>(thrust::raw_pointer_cast(device_scratch_workspace_.data())),
    workspace_bytes_, device_crop_mask,
    thrust::raw_pointer_cast(device_processing_stats_.data()) + crop_box_passed_stat_index,
    num_organized_points_, 1U, stream_);
  countEqualAsync(
    reinterpret_cast<void *>(thrust::raw_pointer_cast(device_scratch_workspace_.data())),
    workspace_bytes_, device_nan_mask,
    thrust::raw_pointer_cast(device_processing_stats_.data()) + nan_stat_index,
    num_organized_points_, static_cast<std::uint8_t>(1), stream_);
  countEqualAsync(
    reinterpret_cast<void *>(thrust::raw_pointer_cast(device_scratch_workspace_.data())),
    workspace_bytes_, device_mismatch_mask,
    thrust::raw_pointer_cast(device_processing_stats_.data()) + mismatch_stat_index,
    num_organized_points_, static_cast<std::uint8_t>(1), stream_);
  std::uint32_t processing_stats[processing_stat_count]{};
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    processing_stats, thrust::raw_pointer_cast(device_processing_stats_.data()),
    sizeof(processing_stats), cudaMemcpyDeviceToHost, stream_));

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  stats_.ring_overflow =
    max_ring_value >= num_rings_ || max_points_per_ring_value >= max_points_per_ring_;
  stats_.num_crop_box_passed_points =
    static_cast<int>(processing_stats[crop_box_passed_stat_index]);
  stats_.num_nan_points = static_cast<int>(processing_stats[nan_stat_index]);
  stats_.mismatch_count = static_cast<int>(processing_stats[mismatch_stat_index]);

  if (num_output_points > 0) {
    extractPointsLaunch(
      device_transformed_points, device_ring_outlier_mask, device_indices, num_organized_points_,
      reinterpret_cast<OutputPointType *>(output_pointcloud_ptr_->data.get()), threads_per_block_,
      blocks_per_grid, stream_);
  }

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

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
