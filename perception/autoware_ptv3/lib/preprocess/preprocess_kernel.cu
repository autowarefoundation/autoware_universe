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

#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/preprocess/preprocess_kernel.hpp"
#include "autoware/ptv3/utils.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <cub/cub.cuh>

#include <thrust/adjacent_difference.h>
#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>
#include <thrust/scan.h>
#include <thrust/scatter.h>
#include <thrust/sequence.h>
#include <thrust/unique.h>

#include <limits>

namespace autoware::ptv3
{

PreprocessCuda::PreprocessCuda(const PTv3Config & config, cudaStream_t stream)
: config_(config), stream_(stream)
{
  points_d_ = autoware::cuda_utils::make_unique<float[]>(
    config_.cloud_capacity_ * config_.num_point_feature_size_);
  cropped_points_d_ = autoware::cuda_utils::make_unique<float[]>(
    config_.cloud_capacity_ * config_.num_point_feature_size_);
  crop_mask_d_ = autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.cloud_capacity_);
  crop_indices_d_ = autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.cloud_capacity_);

  auto policy = thrust::cuda::par.on(stream_);

  if (config_.use_64bit_hash_) {
    hashes64_d_ = autoware::cuda_utils::make_unique<std::uint64_t[]>(config_.cloud_capacity_);
    sorted_hashes64_d_ =
      autoware::cuda_utils::make_unique<std::uint64_t[]>(config_.cloud_capacity_);
    hash_indexes64_d_ = autoware::cuda_utils::make_unique<std::uint64_t[]>(config_.cloud_capacity_);
    sorted_hash_indexes64_d_ =
      autoware::cuda_utils::make_unique<std::uint64_t[]>(config_.cloud_capacity_);
    unique_mask64_d_ = autoware::cuda_utils::make_unique<std::uint64_t[]>(config_.cloud_capacity_);
    unique_indices64_d_ =
      autoware::cuda_utils::make_unique<std::uint64_t[]>(config_.cloud_capacity_);

    thrust::device_ptr<std::uint64_t> idx_ptr(hash_indexes64_d_.get());

    thrust::sequence(policy, idx_ptr, idx_ptr + config_.cloud_capacity_ + 1, 0);
  } else {
    hashes32_d_ = autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.cloud_capacity_);
    sorted_hashes32_d_ =
      autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.cloud_capacity_);
    hash_indexes32_d_ = autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.cloud_capacity_);
    sorted_hash_indexes32_d_ =
      autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.cloud_capacity_);
    unique_mask32_d_ = autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.cloud_capacity_);
    unique_indices32_d_ =
      autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.cloud_capacity_);

    thrust::device_ptr<std::uint32_t> idx_ptr(hash_indexes32_d_.get());

    thrust::sequence(policy, idx_ptr, idx_ptr + config_.cloud_capacity_ + 1, 0);
  }

  std::uint64_t * uint64_nullptr = nullptr;

  cub::DeviceRadixSort::SortPairs(
    nullptr, sort_workspace_size_, uint64_nullptr, uint64_nullptr, uint64_nullptr, uint64_nullptr,
    config_.cloud_capacity_, 0, 64, nullptr);

  sort_workspace_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(sort_workspace_size_);

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

__global__ void points2FeaturesKernel(
  const InputPointType * __restrict__ input_points, std::size_t points_size,
  float4 * __restrict__ output_points)
{
  const auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= points_size) {
    return;
  }

  const InputPointType & input_point = input_points[idx];
  float4 & output_point = output_points[idx];
  output_point.x = input_point.x;
  output_point.y = input_point.y;
  output_point.z = input_point.z;
  output_point.w = static_cast<float>(input_point.intensity) / 255.f;
}

__global__ void cropKernel(
  float4 * __restrict__ points, std::uint32_t * __restrict__ mask, int num_points, float min_x,
  float min_y, float min_z, float max_x, float max_y, float max_z)
{
  auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }
  const float & x = points[idx].x;
  const float & y = points[idx].y;
  const float & z = points[idx].z;

  mask[idx] = x >= min_x && x <= max_x && y >= min_y && y <= max_y && z >= min_z && z <= max_z;
}

template <typename scalar_t, typename mask_t>
__global__ void extractIndicesKernel(
  scalar_t * __restrict__ input_data, mask_t * __restrict__ masks, mask_t * __restrict__ indices,
  scalar_t * __restrict__ output_data, int num_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points && masks[idx] == 1) {
    output_data[indices[idx] - 1] = input_data[idx];
  }
}

template <typename scalar_t, typename mask_t>
__global__ void extractIndicesKernel(
  scalar_t * __restrict__ input_data, mask_t * __restrict__ masks, mask_t * __restrict__ indices1,
  mask_t * __restrict__ indices2, scalar_t * __restrict__ output_data, int num_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points && masks[idx] == 1) {
    output_data[indices1[idx] - 1] = input_data[indices2[idx]];
  }
}

__global__ void voxelizationHash64Kernel(
  const float4 * __restrict__ points, std::uint64_t * __restrict__ hashes, int num_points,
  float voxel_size_x, float voxel_size_y, float voxel_size_z, std::int32_t min_x,
  std::int32_t min_y, std::int32_t min_z)
{
  // FNV64-1A
  auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  const float4 & point = points[idx];
  const auto x = static_cast<std::int32_t>(std::floor(point.x / voxel_size_x));
  const auto y = static_cast<std::int32_t>(std::floor(point.y / voxel_size_y));
  const auto z = static_cast<std::int32_t>(std::floor(point.z / voxel_size_z));

  std::uint64_t hash = 14695981039346656037ULL;
  hash *= 1099511628211ULL;
  hash ^= static_cast<std::uint64_t>(x - min_x);
  hash *= 1099511628211ULL;
  hash ^= static_cast<std::uint64_t>(y - min_y);
  hash *= 1099511628211ULL;
  hash ^= static_cast<std::uint64_t>(z - min_z);

  hashes[idx] = hash;
}

__global__ void voxelizationHash32Kernel(
  const float4 * __restrict__ points, std::uint32_t * __restrict__ hashes, int num_points,
  float voxel_size_x, float voxel_size_y, float voxel_size_z, float min_x, float min_y, float min_z,
  std::uint32_t grid_x_size, std::uint32_t grid_xy_size)
{
  auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  const float4 & point = points[idx];
  const std::uint32_t x =
    static_cast<std::uint32_t>(std::max<float>((point.x - min_x) / voxel_size_x, 0.f));
  const std::uint32_t y =
    static_cast<std::uint32_t>(std::max<float>((point.y - min_y) / voxel_size_y, 0.f));
  const std::uint32_t z =
    static_cast<std::uint32_t>(std::max<float>((point.z - min_z) / voxel_size_z, 0.f));
  hashes[idx] = z * grid_xy_size + y * grid_x_size + x;
}

__global__ void computeGridCoordsAndSerializationKernel(
  const float4 * __restrict__ points, longlong3 * __restrict__ coords,
  std::int64_t * __restrict__ hashes, int num_points, float voxel_size_x, float voxel_size_y,
  float voxel_size_z, std::int32_t min_x, std::int32_t min_y, std::int32_t min_z, int depth)
{
  static_assert(sizeof(longlong3) == sizeof(std::uint64_t) * 3, "longlong3 must be 24 bytes");
  auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  const float4 & point = points[idx];
  const std::int64_t x = static_cast<std::int32_t>(std::floor(point.x / voxel_size_x) - min_x);
  const std::int64_t y = static_cast<std::int32_t>(std::floor(point.y / voxel_size_y) - min_y);
  const std::int64_t z = static_cast<std::int32_t>(std::floor(point.z / voxel_size_z) - min_z);

  coords[idx] = make_longlong3(x, y, z);

  std::int64_t key1 = 0;
  std::int64_t key2 = 0;

  for (int i = 0; i < depth; ++i) {
    std::int64_t mask = 1 << i;
    key1 |= ((x & mask) << (2 * i + 2));
    key1 |= ((y & mask) << (2 * i + 1));
    key1 |= ((z & mask) << (2 * i + 0));

    key2 |= ((y & mask) << (2 * i + 2));
    key2 |= ((x & mask) << (2 * i + 1));
    key2 |= ((z & mask) << (2 * i + 0));
  }

  hashes[idx] = key1;
  hashes[idx + num_points] = key2;
}

std::size_t PreprocessCuda::generateFeatures(
  const InputPointType * input_data, unsigned int num_points, float * voxel_features,
  std::int64_t * voxel_coords, std::int64_t * voxel_hashes)
{
  auto policy = thrust::cuda::par.on(stream_);

  const auto num_blocks = divup(num_points, config_.threads_per_block_);
  points2FeaturesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
    input_data, num_points, reinterpret_cast<float4 *>(points_d_.get()));

  cropKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
    reinterpret_cast<float4 *>(points_d_.get()), crop_mask_d_.get(), num_points,
    config_.min_x_range_, config_.min_y_range_, config_.min_z_range_, config_.max_x_range_,
    config_.max_y_range_, config_.max_z_range_);

  thrust::inclusive_scan(
    policy, crop_mask_d_.get(), crop_mask_d_.get() + num_points, crop_indices_d_.get());

  std::uint32_t num_cropped_points;

  cudaMemcpyAsync(
    &num_cropped_points, crop_indices_d_.get() + num_points - 1, sizeof(std::uint32_t),
    cudaMemcpyDeviceToHost, stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  if (num_cropped_points == 0) {
    return 0;
  }

  extractIndicesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
    reinterpret_cast<float4 *>(points_d_.get()), crop_mask_d_.get(), crop_indices_d_.get(),
    reinterpret_cast<float4 *>(cropped_points_d_.get()), num_points);

  auto min_op = [] __host__ __device__(const float4 & a, const float4 & b) {
    return make_float4(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z), fminf(a.w, b.w));
  };

  auto max_op = [] __host__ __device__(const float4 & a, const float4 & b) {
    return make_float4(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z), fmaxf(a.w, b.w));
  };

  float4 min_value = make_float4(
    std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
    std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  min_value = thrust::reduce(
    policy, reinterpret_cast<float4 *>(cropped_points_d_.get()),
    reinterpret_cast<float4 *>(cropped_points_d_.get()) + num_cropped_points, min_value, min_op);

  float4 max_value = make_float4(
    -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
    -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
  max_value = thrust::reduce(
    policy, reinterpret_cast<float4 *>(cropped_points_d_.get()),
    reinterpret_cast<float4 *>(cropped_points_d_.get()) + num_cropped_points, max_value, max_op);

  std::int32_t min_x = static_cast<std::int32_t>(std::floor(min_value.x / config_.voxel_x_size_));
  std::int32_t min_y = static_cast<std::int32_t>(std::floor(min_value.y / config_.voxel_y_size_));
  std::int32_t min_z = static_cast<std::int32_t>(std::floor(min_value.z / config_.voxel_z_size_));

  const auto num_cropped_blocks = divup(num_cropped_points, config_.threads_per_block_);

  std::uint64_t num_unique_points;

  if (config_.use_64bit_hash_) {
    voxelizationHash64Kernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
      reinterpret_cast<float4 *>(cropped_points_d_.get()), hashes64_d_.get(), num_cropped_points,
      config_.voxel_x_size_, config_.voxel_y_size_, config_.voxel_z_size_, min_x, min_y, min_z);

    cub::DeviceRadixSort::SortPairs(
      reinterpret_cast<void *>(sort_workspace_d_.get()), sort_workspace_size_, hashes64_d_.get(),
      sorted_hashes64_d_.get(), hash_indexes64_d_.get(), sorted_hash_indexes64_d_.get(),
      num_cropped_points, 0, 64, stream_);

    auto not_equal = [] __device__(const std::uint64_t a, const std::uint64_t b) { return a != b; };

    thrust::adjacent_difference(
      policy, sorted_hashes64_d_.get(), sorted_hashes64_d_.get() + num_cropped_points,
      unique_mask64_d_.get(), not_equal);

    std::uint64_t one = 1;
    cudaMemcpyAsync(
      unique_mask64_d_.get(), &one, sizeof(std::uint64_t), cudaMemcpyHostToDevice, stream_);

    thrust::inclusive_scan(
      policy, unique_mask64_d_.get(), unique_mask64_d_.get() + num_cropped_points,
      unique_indices64_d_.get());

    cudaMemcpyAsync(
      &num_unique_points, unique_indices64_d_.get() + num_cropped_points - 1, sizeof(std::int64_t),
      cudaMemcpyDeviceToHost, stream_);

    extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
      reinterpret_cast<float4 *>(cropped_points_d_.get()), unique_mask64_d_.get(),
      unique_indices64_d_.get(), sorted_hash_indexes64_d_.get(),
      reinterpret_cast<float4 *>(voxel_features), num_cropped_points);

  } else {
    voxelizationHash32Kernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
      reinterpret_cast<float4 *>(cropped_points_d_.get()), hashes32_d_.get(), num_cropped_points,
      config_.voxel_x_size_, config_.voxel_y_size_, config_.voxel_z_size_, config_.min_x_range_,
      config_.min_y_range_, config_.min_z_range_, static_cast<std::uint32_t>(config_.grid_x_size_),
      static_cast<std::uint32_t>(config_.grid_x_size_ * config_.grid_y_size_));

    cub::DeviceRadixSort::SortPairs(
      reinterpret_cast<void *>(sort_workspace_d_.get()), sort_workspace_size_, hashes32_d_.get(),
      sorted_hashes32_d_.get(), hash_indexes32_d_.get(), sorted_hash_indexes32_d_.get(),
      num_cropped_points, 0, 32, stream_);

    auto not_equal = [] __device__(const std::uint32_t a, const std::uint32_t b) { return a != b; };

    thrust::adjacent_difference(
      policy, sorted_hashes32_d_.get(), sorted_hashes32_d_.get() + num_cropped_points,
      unique_mask32_d_.get(), not_equal);

    std::uint32_t one = 1;
    cudaMemcpyAsync(
      unique_mask32_d_.get(), &one, sizeof(std::uint32_t), cudaMemcpyHostToDevice, stream_);

    thrust::inclusive_scan(
      policy, unique_mask32_d_.get(), unique_mask32_d_.get() + num_cropped_points,
      unique_indices32_d_.get());

    std::uint32_t num_unique_points32;
    cudaMemcpyAsync(
      &num_unique_points32, unique_indices32_d_.get() + num_cropped_points - 1,
      sizeof(std::uint32_t), cudaMemcpyDeviceToHost, stream_);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    num_unique_points = static_cast<std::uint64_t>(num_unique_points32);

    extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
      reinterpret_cast<float4 *>(cropped_points_d_.get()), unique_mask32_d_.get(),
      unique_indices32_d_.get(), sorted_hash_indexes32_d_.get(),
      reinterpret_cast<float4 *>(voxel_features), num_cropped_points);
  }

  computeGridCoordsAndSerializationKernel<<<
    num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
    reinterpret_cast<float4 *>(voxel_features), reinterpret_cast<longlong3 *>(voxel_coords),
    voxel_hashes, num_unique_points, config_.voxel_x_size_, config_.voxel_y_size_,
    config_.voxel_z_size_, min_x, min_y, min_z, config_.serialization_depth_);

  return num_unique_points;
}

}  // namespace autoware::ptv3
