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
#include <autoware/cuda_utils/thrust_utils.hpp>
#include <cub/cub.cuh>

#include <sensor_msgs/msg/point_field.hpp>

#include <cuda_runtime.h>
#include <tf2/utils.h>
#include <thrust/execution_policy.h>
#include <thrust/iterator/transform_iterator.h>
#include <thrust/reduce.h>
#include <thrust/scan.h>

#include <cstdint>

namespace autoware::cuda_pointcloud_preprocessor
{

namespace thrust_stream = cuda_utils::thrust_stream;

namespace
{
template <typename T>
struct EqualsTo
{
  using result_type = int;
  T value;
  __device__ int operator()(const T & x) const { return x == value ? 1 : 0; }
};

// Count, on `stream`, how many of the first `n` elements of `mask` equal `value`,
// leaving the result in the device scalar `out_dev`. Using CUB DeviceReduce keeps
// the result on the device: thrust::count would instead copy its return value to
// the host and synchronize the *default* stream on every call (a process-wide
// barrier). CUB scratch is carved from the pre-allocated thrust workspace, so no
// per-call cudaMalloc/cudaFree is incurred either.
template <typename T>
void cubCountEqual(
  const T * mask, std::size_t n, T value, int * out_dev,
  autoware::cuda_utils::ThrustWorkspace::Allocator alloc, cudaStream_t stream)
{
  thrust::transform_iterator<EqualsTo<T>, const T *> it(mask, EqualsTo<T>{value});
  std::size_t temp_bytes = 0;
  CHECK_CUDA_ERROR(cub::DeviceReduce::Sum(nullptr, temp_bytes, it, out_dev, n, stream));
  char * temp = alloc.allocate(static_cast<std::ptrdiff_t>(temp_bytes));
  CHECK_CUDA_ERROR(cub::DeviceReduce::Sum(temp, temp_bytes, it, out_dev, n, stream));
  alloc.deallocate(temp, temp_bytes);
}

// Ensure a device_vector can hold at least n elements, growing monotonically with
// 2x headroom. An enlarging thrust::device_vector::resize() value-initializes the
// new region on the *default* stream and synchronizes -- a process-wide barrier --
// so never shrink and over-allocate, making regrowth geometrically rare. The
// vector's size() therefore becomes a high-water mark: callers must drive kernels
// with the logical element count, not size().
template <typename Vec>
void grow_to(Vec & vec, std::size_t n)
{
  if (vec.size() < n) vec.resize(n * 2);
}
}  // namespace

CudaPointcloudPreprocessor::CudaPointcloudPreprocessor() : stream_(initialize_stream())
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

  num_rings_ = 1;
  max_points_per_ring_ = 1;
  num_organized_points_ = num_rings_ * max_points_per_ring_;
  device_ring_index_.resize(num_rings_);

  device_indexes_tensor_.resize(num_organized_points_);
  device_sorted_indexes_tensor_.resize(num_organized_points_);

  device_segment_offsets_.resize(num_rings_ + 1);
  device_segment_offsets_[0] = 0;
  device_segment_offsets_[1] = 1;

  device_max_ring_.resize(1);
  device_max_points_per_ring_.resize(1);

  device_organized_points_.resize(num_organized_points_);

  thrust_stream::fill(device_max_ring_, 0, stream_);
  thrust_stream::fill(device_max_points_per_ring_, 0, stream_);
  thrust_stream::fill(device_indexes_tensor_, UINT32_MAX, stream_);

  sort_workspace_bytes_ = querySortWorkspace(
    num_rings_ * max_points_per_ring_, num_rings_,
    thrust::raw_pointer_cast(device_segment_offsets_.data()),
    thrust::raw_pointer_cast(device_indexes_tensor_.data()),
    thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()), stream_);

  device_transformed_points_.resize(num_organized_points_);
  device_crop_mask_.resize(num_organized_points_);
  device_nan_mask_.resize(num_organized_points_);
  device_mismatch_mask_.resize(num_organized_points_);
  device_ring_outlier_mask_.resize(num_organized_points_);
  device_indices_.resize(num_organized_points_);
  device_diag_counts_.resize(3);

  preallocateOutput();
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

void CudaPointcloudPreprocessor::preallocateOutput()
{
  output_pointcloud_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  output_pointcloud_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
    num_rings_ * max_points_per_ring_ * sizeof(OutputPointType));
}

void CudaPointcloudPreprocessor::organizePointcloud()
{
  thrust_stream::fill_n(device_ring_index_, num_rings_, 0, stream_);
  thrust_stream::fill_n(
    device_indexes_tensor_, num_organized_points_, static_cast<std::uint32_t>(num_raw_points_),
    stream_);

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

  std::int32_t max_ring_value{};
  std::int32_t max_points_per_ring{};

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &max_ring_value, thrust::raw_pointer_cast(device_max_ring_.data()), sizeof(std::int32_t),
    cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &max_points_per_ring, thrust::raw_pointer_cast(device_max_points_per_ring_.data()),
    sizeof(std::int32_t), cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  if (max_ring_value >= num_rings_ || max_points_per_ring > max_points_per_ring_) {
    num_rings_ = max_ring_value + 1;
    max_points_per_ring_ = std::max((max_points_per_ring + 511) / 512 * 512, 512);
    num_organized_points_ = num_rings_ * max_points_per_ring_;

    // Grow these buffers monotonically with headroom (grow_to) so the
    // value-initializing resize -- which thrust runs on the default stream and
    // synchronizes -- only fires while climbing to the high-water mark, not on
    // every ring-geometry change. The fills below initialize the live region on
    // the node stream; kernels are driven by num_organized_points_/num_rings_,
    // never by size().
    grow_to(device_ring_index_, num_rings_);
    thrust_stream::fill_n(device_ring_index_, num_rings_, 0, stream_);
    grow_to(device_indexes_tensor_, num_organized_points_);
    thrust_stream::fill_n<uint32_t>(
      device_indexes_tensor_, num_organized_points_, num_raw_points_, stream_);
    grow_to(device_sorted_indexes_tensor_, num_organized_points_);
    thrust_stream::fill_n<uint32_t>(
      device_sorted_indexes_tensor_, num_organized_points_, num_raw_points_, stream_);
    grow_to(device_segment_offsets_, num_rings_ + 1);
    grow_to(device_organized_points_, num_organized_points_);
    thrust_stream::fill_n(
      device_organized_points_, num_organized_points_, InputPointType{}, stream_);

    grow_to(device_transformed_points_, num_organized_points_);
    thrust_stream::fill_n(
      device_transformed_points_, num_organized_points_, InputPointType{}, stream_);
    grow_to(device_crop_mask_, num_organized_points_);
    thrust_stream::fill_n(device_crop_mask_, num_organized_points_, 0U, stream_);
    grow_to(device_nan_mask_, num_organized_points_);
    thrust_stream::fill_n<uint8_t>(device_nan_mask_, num_organized_points_, 0, stream_);
    grow_to(device_mismatch_mask_, num_organized_points_);
    thrust_stream::fill_n<uint8_t>(device_mismatch_mask_, num_organized_points_, 0, stream_);
    grow_to(device_ring_outlier_mask_, num_organized_points_);
    thrust_stream::fill_n(device_ring_outlier_mask_, num_organized_points_, 0U, stream_);
    grow_to(device_indices_, num_organized_points_);
    thrust_stream::fill_n(device_indices_, num_organized_points_, 0U, stream_);

    preallocateOutput();

    std::vector<std::int32_t> segment_offsets_host(num_rings_ + 1);
    for (int i = 0; i < num_rings_ + 1; i++) {
      segment_offsets_host[i] = i * max_points_per_ring_;
    }

    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      thrust::raw_pointer_cast(device_segment_offsets_.data()), segment_offsets_host.data(),
      (num_rings_ + 1) * sizeof(std::int32_t), cudaMemcpyHostToDevice, stream_));

    CHECK_CUDA_ERROR(cudaMemsetAsync(
      thrust::raw_pointer_cast(device_ring_index_.data()), 0, num_rings_ * sizeof(std::int32_t),
      stream_));
    CHECK_CUDA_ERROR(cudaMemsetAsync(
      thrust::raw_pointer_cast(device_indexes_tensor_.data()), 0xFF,
      num_organized_points_ * sizeof(std::int32_t), stream_));

    sort_workspace_bytes_ = querySortWorkspace(
      num_organized_points_, num_rings_, thrust::raw_pointer_cast(device_segment_offsets_.data()),
      thrust::raw_pointer_cast(device_indexes_tensor_.data()),
      thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()), stream_);
    grow_to(device_sort_workspace_, sort_workspace_bytes_);

    organizeLaunch(
      thrust::raw_pointer_cast(device_input_points_.data()),
      thrust::raw_pointer_cast(device_indexes_tensor_.data()),
      thrust::raw_pointer_cast(device_ring_index_.data()), num_rings_,
      thrust::raw_pointer_cast(device_max_ring_.data()), max_points_per_ring_,
      thrust::raw_pointer_cast(device_max_points_per_ring_.data()), num_raw_points_,
      threads_per_block_, raw_points_blocks_per_grid, stream_);
  }

  if (num_organized_points_ == num_rings_) {
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()),
      thrust::raw_pointer_cast(device_indexes_tensor_.data()),
      num_organized_points_ * sizeof(std::uint32_t), cudaMemcpyDeviceToDevice, stream_));
  } else {
    cub::DeviceSegmentedRadixSort::SortKeys(
      reinterpret_cast<void *>(thrust::raw_pointer_cast(device_sort_workspace_.data())),  // NOLINT
      sort_workspace_bytes_, thrust::raw_pointer_cast(device_indexes_tensor_.data()),
      thrust::raw_pointer_cast(device_sorted_indexes_tensor_.data()), num_organized_points_,
      num_rings_, thrust::raw_pointer_cast(device_segment_offsets_.data()),
      thrust::raw_pointer_cast(device_segment_offsets_.data()) + 1, 0, sizeof(std::uint32_t) * 8,
      stream_);
  }

  // reuse device_indexes_tensor_ to store valid point location
  thrust_stream::fill(device_indexes_tensor_, 0U, stream_);

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
  num_raw_points_ = static_cast<int>(input_pointcloud_msg.width * input_pointcloud_msg.height);
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

  // Reset contents in the device vectors. device_organized_points_ may be
  // over-allocated (grow_to), so reset only its live [0, num_organized_points_)
  // region on the node stream.
  thrust_stream::fill(device_input_points_, InputPointType{}, stream_);
  thrust_stream::fill_n(device_organized_points_, num_organized_points_, InputPointType{}, stream_);

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    thrust::raw_pointer_cast(device_input_points_.data()), input_pointcloud_msg.data.data(),
    num_raw_points_ * sizeof(InputPointType), cudaMemcpyHostToDevice, stream_));

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  organizePointcloud();

  // Reset all contents in the device vector
  thrust_stream::fill(device_transformed_points_, InputPointType{}, stream_);
  thrust_stream::fill<uint32_t>(device_ring_outlier_mask_, 0, stream_);
  thrust_stream::fill<uint8_t>(device_mismatch_mask_, 0, stream_);
  thrust_stream::fill<uint8_t>(device_nan_mask_, 0, stream_);
  thrust_stream::fill<uint32_t>(device_crop_mask_, 0, stream_);

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

  // Valid twist count for this callback. device_twist_*_structs_.size() is only a
  // monotonic high-water mark (see setupTwist*Structs), so the kernels below must
  // use these counts rather than .size().
  std::size_t num_twist_2d_structs = 0;
  std::size_t num_twist_3d_structs = 0;
  if (undistortion_type_ == UndistortionType::Undistortion3D) {
    num_twist_3d_structs = setupTwist3DStructs(
      twist_queue, angular_velocity_queue, pointcloud_stamp_nsec, first_point_rel_stamp_nsec,
      device_twist_3d_structs_, stream_);
  } else if (undistortion_type_ == UndistortionType::Undistortion2D) {
    num_twist_2d_structs = setupTwist2DStructs(
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
    thrust_stream::fill_n(device_crop_mask_, num_organized_points_, 1U, stream_);
  }

  // Undistortion
  if (undistortion_type_ == UndistortionType::Undistortion3D && num_twist_3d_structs > 0) {
    undistort3DLaunch(
      device_transformed_points, num_organized_points_, device_twist_3d_structs,
      static_cast<int>(num_twist_3d_structs), device_mismatch_mask, threads_per_block_,
      blocks_per_grid, stream_);
  } else if (undistortion_type_ == UndistortionType::Undistortion2D && num_twist_2d_structs > 0) {
    undistort2DLaunch(
      device_transformed_points, num_organized_points_, device_twist_2d_structs,
      static_cast<int>(num_twist_2d_structs), device_mismatch_mask, threads_per_block_,
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
    thrust::fill(
      cuda_utils::thrust_on_stream(stream_), device_ring_outlier_mask,
      device_ring_outlier_mask + num_organized_points_, 1);
  }

  combineMasksLaunch(
    device_crop_mask, device_ring_outlier_mask, num_organized_points_, device_ring_outlier_mask,
    threads_per_block_, blocks_per_grid, stream_);

  // Mask out invalid points in the array
  combineMasksLaunch(
    device_is_valid_point, device_ring_outlier_mask, num_organized_points_,
    device_ring_outlier_mask, threads_per_block_, blocks_per_grid, stream_);

  // Back thrust temporaries with the pre-allocated workspace (no per-call
  // cudaMalloc/cudaFree) while keeping execution on the node's own stream.
  const auto workspace_policy = thrust::cuda::par(thrust_workspace_.allocator()).on(stream_);

  thrust::inclusive_scan(
    workspace_policy, device_ring_outlier_mask, device_ring_outlier_mask + num_organized_points_,
    device_indices);

  // Diagnostic counts: reduce on the node stream with CUB into device scalars
  // instead of thrust::count, whose host-value return synchronizes the default
  // stream on every call. The results are read back at the single synchronize
  // below, alongside num_output_points.
  // num_organized_points_ is the logical element count; the mask buffers may be
  // over-allocated (grow_to), so count over [0, num_organized_points_), not size().
  int * device_diag_counts = thrust::raw_pointer_cast(device_diag_counts_.data());
  cubCountEqual(
    device_crop_mask, num_organized_points_, static_cast<std::uint32_t>(1), device_diag_counts + 0,
    thrust_workspace_.allocator(), stream_);
  cubCountEqual(
    device_nan_mask, num_organized_points_, static_cast<std::uint8_t>(1), device_diag_counts + 1,
    thrust_workspace_.allocator(), stream_);
  cubCountEqual(
    device_mismatch_mask, num_organized_points_, static_cast<std::uint8_t>(1),
    device_diag_counts + 2, thrust_workspace_.allocator(), stream_);

  // The extract kernel writes only the points the scan selected, so launching it
  // unconditionally is a no-op when nothing passed. Doing so removes the host's
  // dependency on num_output_points before the launch, letting every
  // device->host result be gathered at one synchronize after all GPU work is
  // queued (rather than syncing mid-pipeline to read the count back first).
  extractPointsLaunch(
    device_transformed_points, device_ring_outlier_mask, device_indices, num_organized_points_,
    reinterpret_cast<OutputPointType *>(output_pointcloud_ptr_->data.get()), threads_per_block_,
    blocks_per_grid, stream_);

  int num_output_points{};
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &num_output_points, device_indices + num_organized_points_ - 1, sizeof(int),
    cudaMemcpyDeviceToHost, stream_));
  int diag_counts_host[3] = {0, 0, 0};
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    diag_counts_host, device_diag_counts, sizeof(diag_counts_host), cudaMemcpyDeviceToHost,
    stream_));

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  stats_.num_crop_box_passed_points = diag_counts_host[0];
  stats_.num_nan_points = diag_counts_host[1];
  stats_.mismatch_count = diag_counts_host[2];

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
