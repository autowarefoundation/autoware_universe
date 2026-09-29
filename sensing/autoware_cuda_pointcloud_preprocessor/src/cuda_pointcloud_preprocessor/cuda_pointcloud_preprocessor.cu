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
#include <numeric>
#include <stdexcept>
#include <vector>

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
  if (capacity.max_input_point_count == 0 || capacity.max_twist_struct_count == 0) {
    throw std::runtime_error("CudaPointcloudPreprocessor capacities must be positive");
  }
  return capacity;
}

}  // namespace

CudaPointcloudPreprocessor::CudaPointcloudPreprocessor(const PreprocessorCapacity & capacity)
: capacity_(validate_capacity(capacity)), stream_(initialize_stream())
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
  const std::size_t max_points = capacity_.max_input_point_count;

  device_input_points_.resize(max_points);
  device_ring_keys_.resize(max_points);
  device_sorted_ring_keys_.resize(max_points);
  device_point_indices_.resize(max_points);
  device_sorted_point_indices_.resize(max_points);
  device_transformed_points_.resize(max_points);
  device_crop_mask_.resize(max_points);
  device_nan_mask_.resize(max_points);
  device_mismatch_mask_.resize(max_points);
  device_ring_outlier_mask_.resize(max_points);
  device_indices_.resize(max_points);
  device_twist_2d_structs_.resize(capacity_.max_twist_struct_count);
  device_twist_3d_structs_.resize(capacity_.max_twist_struct_count);
  device_processing_stats_.resize(processing_stat_count);

  // The sort's values are the point indices 0..N-1; SortPairs leaves its inputs untouched, so
  // this is uploaded once.
  std::vector<std::uint32_t> point_indices_host(max_points);
  std::iota(point_indices_host.begin(), point_indices_host.end(), 0U);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    thrust::raw_pointer_cast(device_point_indices_.data()), point_indices_host.data(),
    point_indices_host.size() * sizeof(std::uint32_t), cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  // Workspaces are queried for the capacity; every smaller frame needs no more.
  std::size_t sort_workspace_bytes{};
  CHECK_CUDA_ERROR(
    cub::DeviceRadixSort::SortPairs(
      nullptr, sort_workspace_bytes, thrust::raw_pointer_cast(device_ring_keys_.data()),
      thrust::raw_pointer_cast(device_sorted_ring_keys_.data()),
      thrust::raw_pointer_cast(device_point_indices_.data()),
      thrust::raw_pointer_cast(device_sorted_point_indices_.data()), max_points, 0,
      sizeof(std::uint16_t) * 8, stream_));
  std::size_t scan_workspace_bytes{};
  CHECK_CUDA_ERROR(
    cub::DeviceScan::InclusiveSum(
      nullptr, scan_workspace_bytes, thrust::raw_pointer_cast(device_ring_outlier_mask_.data()),
      thrust::raw_pointer_cast(device_indices_.data()), max_points, stream_));
  const auto reduce_workspace_bytes = std::max(
    {queryCountWorkspace(
       thrust::raw_pointer_cast(device_crop_mask_.data()),
       thrust::raw_pointer_cast(device_processing_stats_.data()), max_points, 1U, stream_),
     queryCountWorkspace(
       thrust::raw_pointer_cast(device_nan_mask_.data()),
       thrust::raw_pointer_cast(device_processing_stats_.data()), max_points,
       static_cast<std::uint8_t>(1), stream_),
     queryCountWorkspace(
       thrust::raw_pointer_cast(device_mismatch_mask_.data()),
       thrust::raw_pointer_cast(device_processing_stats_.data()), max_points,
       static_cast<std::uint8_t>(1), stream_)});
  workspace_bytes_ = std::max({sort_workspace_bytes, scan_workspace_bytes, reduce_workspace_bytes});
  device_scratch_workspace_.resize(workspace_bytes_);

  preallocateOutput();
}

void CudaPointcloudPreprocessor::preallocateOutput()
{
  output_pointcloud_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  output_pointcloud_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
    capacity_.max_input_point_count * sizeof(OutputPointType));
}

// Stable sort of the point indices by ring: rings become contiguous and each keeps its input
// (scan) order, which is what the ring outlier filter walks along. Nothing is dropped and no
// per-ring capacity exists; the sorted indices drive the gather in transformPointsLaunch.
void CudaPointcloudPreprocessor::sortPointsByRing()
{
  const int blocks_per_grid = (num_raw_points_ + threads_per_block_ - 1) / threads_per_block_;
  ringKeysLaunch(
    thrust::raw_pointer_cast(device_input_points_.data()),
    thrust::raw_pointer_cast(device_ring_keys_.data()), num_raw_points_, threads_per_block_,
    blocks_per_grid, stream_);
  CHECK_CUDA_ERROR(
    cub::DeviceRadixSort::SortPairs(
      reinterpret_cast<void *>(thrust::raw_pointer_cast(device_scratch_workspace_.data())),
      workspace_bytes_, thrust::raw_pointer_cast(device_ring_keys_.data()),
      thrust::raw_pointer_cast(device_sorted_ring_keys_.data()),
      thrust::raw_pointer_cast(device_point_indices_.data()),
      thrust::raw_pointer_cast(device_sorted_point_indices_.data()), num_raw_points_, 0,
      sizeof(std::uint16_t) * 8, stream_));
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

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    thrust::raw_pointer_cast(device_input_points_.data()), input_pointcloud_msg.data.data(),
    num_raw_points_ * sizeof(InputPointType), cudaMemcpyHostToDevice, stream_));

  sortPointsByRing();

  // Reset the masks the kernels below only write selectively (the transformed points and the
  // ring outlier mask are written for every point), over this frame's points.
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    thrust::raw_pointer_cast(device_mismatch_mask_.data()), 0,
    num_raw_points_ * sizeof(std::uint8_t), stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    thrust::raw_pointer_cast(device_nan_mask_.data()), 0, num_raw_points_ * sizeof(std::uint8_t),
    stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    thrust::raw_pointer_cast(device_crop_mask_.data()), 0, num_raw_points_ * sizeof(std::uint32_t),
    stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    thrust::raw_pointer_cast(device_processing_stats_.data()), 0,
    processing_stat_count * sizeof(std::uint32_t), stream_));

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
  const int num_points = static_cast<int>(num_raw_points_);

  const int blocks_per_grid = (num_points + threads_per_block_ - 1) / threads_per_block_;

  maxRingSizeLaunch(
    thrust::raw_pointer_cast(device_sorted_ring_keys_.data()), num_points,
    thrust::raw_pointer_cast(device_processing_stats_.data()) + max_points_per_ring_stat_index,
    threads_per_block_, blocks_per_grid, stream_);

  transformPointsLaunch(
    thrust::raw_pointer_cast(device_input_points_.data()),
    thrust::raw_pointer_cast(device_sorted_point_indices_.data()), device_transformed_points,
    num_points, transform_struct, threads_per_block_, blocks_per_grid, stream_);

  // Crop box filter
  int crop_box_blocks_per_grid = std::min(blocks_per_grid, max_blocks_per_grid_);
  if (device_crop_box_structs_.size() > 0) {
    cropBoxLaunch(
      device_transformed_points, device_crop_mask, device_nan_mask, num_points,
      thrust::raw_pointer_cast(device_crop_box_structs_.data()),
      static_cast<int>(device_crop_box_structs_.size()), crop_box_blocks_per_grid,
      threads_per_block_, stream_);
  } else {
    fillDeviceVectorPrefix(
      device_crop_mask_, num_raw_points_, 1U, threads_per_block_, max_blocks_per_grid_, stream_);
  }

  // Undistortion
  if (undistortion_type_ == UndistortionType::Undistortion3D && active_twist_3d_struct_count > 0) {
    undistort3DLaunch(
      device_transformed_points, num_points, device_twist_3d_structs,
      static_cast<int>(active_twist_3d_struct_count), device_mismatch_mask, threads_per_block_,
      blocks_per_grid, stream_);
  } else if (
    undistortion_type_ == UndistortionType::Undistortion2D && active_twist_2d_struct_count > 0) {
    undistort2DLaunch(
      device_transformed_points, num_points, device_twist_2d_structs,
      static_cast<int>(active_twist_2d_struct_count), device_mismatch_mask, threads_per_block_,
      blocks_per_grid, stream_);
  }

  // Ring outlier
  if (enable_ring_outlier_filter_) {
    ringOutlierFilterLaunch(
      device_transformed_points, device_ring_outlier_mask, num_points,
      ring_outlier_parameters_.distance_ratio,
      ring_outlier_parameters_.object_length_threshold *
        ring_outlier_parameters_.object_length_threshold,
      threads_per_block_, blocks_per_grid, stream_);
  } else {
    fillDeviceVectorPrefix(
      device_ring_outlier_mask_, num_raw_points_, 1U, threads_per_block_, max_blocks_per_grid_,
      stream_);
  }

  combineMasksLaunch(
    device_crop_mask, device_ring_outlier_mask, num_points, device_ring_outlier_mask,
    threads_per_block_, blocks_per_grid, stream_);

  CHECK_CUDA_ERROR(
    cub::DeviceScan::InclusiveSum(
      reinterpret_cast<void *>(thrust::raw_pointer_cast(device_scratch_workspace_.data())),
      workspace_bytes_, device_ring_outlier_mask, device_indices, num_points, stream_));

  int num_output_points{};
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &num_output_points, device_indices + num_points - 1, sizeof(int), cudaMemcpyDeviceToHost,
    stream_));

  countEqualAsync(
    reinterpret_cast<void *>(thrust::raw_pointer_cast(device_scratch_workspace_.data())),
    workspace_bytes_, device_crop_mask,
    thrust::raw_pointer_cast(device_processing_stats_.data()) + crop_box_passed_stat_index,
    num_points, 1U, stream_);
  countEqualAsync(
    reinterpret_cast<void *>(thrust::raw_pointer_cast(device_scratch_workspace_.data())),
    workspace_bytes_, device_nan_mask,
    thrust::raw_pointer_cast(device_processing_stats_.data()) + nan_stat_index, num_points,
    static_cast<std::uint8_t>(1), stream_);
  countEqualAsync(
    reinterpret_cast<void *>(thrust::raw_pointer_cast(device_scratch_workspace_.data())),
    workspace_bytes_, device_mismatch_mask,
    thrust::raw_pointer_cast(device_processing_stats_.data()) + mismatch_stat_index, num_points,
    static_cast<std::uint8_t>(1), stream_);
  std::uint32_t processing_stats[processing_stat_count]{};
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    processing_stats, thrust::raw_pointer_cast(device_processing_stats_.data()),
    sizeof(processing_stats), cudaMemcpyDeviceToHost, stream_));

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  stats_.num_crop_box_passed_points =
    static_cast<int>(processing_stats[crop_box_passed_stat_index]);
  stats_.num_nan_points = static_cast<int>(processing_stats[nan_stat_index]);
  stats_.mismatch_count = static_cast<int>(processing_stats[mismatch_stat_index]);
  stats_.max_points_per_ring = static_cast<int>(processing_stats[max_points_per_ring_stat_index]);

  if (num_output_points > 0) {
    extractPointsLaunch(
      device_transformed_points, device_ring_outlier_mask, device_indices, num_points,
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
