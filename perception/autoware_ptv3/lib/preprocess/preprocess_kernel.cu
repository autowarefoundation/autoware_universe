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
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <cub/cub.cuh>

#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>
#include <thrust/sequence.h>

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace autoware::ptv3
{
namespace
{
struct NotEqual
{
  template <typename T>
  __host__ __device__ bool operator()(const T & a, const T & b) const
  {
    return a != b;
  }
};
}  // namespace

PreprocessCuda::PreprocessCuda(const PTv3Config & config, cudaStream_t stream)
: config_(config), stream_(stream)
{
  points_d_ = autoware::cuda_utils::make_unique<float[]>(
    config_.cloud_capacity_ * config_.num_point_feature_size_);
  cropped_points_d_ = autoware::cuda_utils::make_unique<float[]>(
    config_.cloud_capacity_ * config_.num_point_feature_size_);
  cropped_input_points_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(
    config_.cloud_capacity_ * sizeof(CloudPointTypeXYZIRCAEDT));
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

    thrust::sequence(policy, idx_ptr, idx_ptr + config_.cloud_capacity_, 0);
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

    thrust::sequence(policy, idx_ptr, idx_ptr + config_.cloud_capacity_, 0);
  }

  std::uint64_t * uint64_nullptr = nullptr;

  std::size_t sort_pair_workspace_size = 0;
  std::size_t inclusive_sum_workspace_size = 0;
  std::size_t adjacent_difference_workspace_size = 0;

  CHECK_CUDA_ERROR(
    cub::DeviceRadixSort::SortPairs(
      nullptr, sort_pair_workspace_size, uint64_nullptr, uint64_nullptr, uint64_nullptr,
      uint64_nullptr, config_.cloud_capacity_, 0, 64, nullptr));
  CHECK_CUDA_ERROR(
    cub::DeviceScan::InclusiveSum(
      nullptr, inclusive_sum_workspace_size, uint64_nullptr, uint64_nullptr,
      config_.cloud_capacity_));
  CHECK_CUDA_ERROR(
    cub::DeviceAdjacentDifference::SubtractLeftCopy(
      nullptr, adjacent_difference_workspace_size, uint64_nullptr, uint64_nullptr,
      config_.cloud_capacity_, NotEqual{}));

  generate_feature_workspace_size_ = std::max(
    {sort_pair_workspace_size, inclusive_sum_workspace_size, adjacent_difference_workspace_size});
  generate_feature_workspace_d_ =
    autoware::cuda_utils::make_unique<std::uint8_t[]>(generate_feature_workspace_size_);

  num_cropped_points_ = autoware::cuda_utils::make_unique_host<std::uint32_t>();
  num_unique_points32_ = autoware::cuda_utils::make_unique_host<std::uint32_t>();
  num_unique_points64_ = autoware::cuda_utils::make_unique_host<std::uint64_t>();

  pooling_keys_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);
  pooling_sorted_keys_d_ =
    autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);
  pooling_indices_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);
  pooling_sorted_indices_d_ =
    autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);
  pooling_run_flags_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);
  pooling_run_ids_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);

  std::size_t pooling_sort_workspace_size = 0;
  std::size_t pooling_scan_workspace_size = 0;
  std::int64_t * int64_nullptr = nullptr;
  cub::DeviceRadixSort::SortPairs(
    nullptr, pooling_sort_workspace_size, int64_nullptr, int64_nullptr, int64_nullptr,
    int64_nullptr, config_.max_num_voxels_, 0, 63, nullptr);
  cub::DeviceScan::InclusiveSum(
    nullptr, pooling_scan_workspace_size, int64_nullptr, int64_nullptr, config_.max_num_voxels_,
    nullptr);
  pooling_workspace_size_ = std::max(pooling_sort_workspace_size, pooling_scan_workspace_size);
  pooling_workspace_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(pooling_workspace_size_);

  CHECK_CUDA_ERROR(
    cudaEventCreateWithFlags(&num_cropped_points_copy_event_, cudaEventDisableTiming));
  CHECK_CUDA_ERROR(
    cudaEventCreateWithFlags(&num_unique_points32_copy_event_, cudaEventDisableTiming));
  CHECK_CUDA_ERROR(
    cudaEventCreateWithFlags(&num_unique_points64_copy_event_, cudaEventDisableTiming));

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

PreprocessCuda::~PreprocessCuda()
{
  if (num_cropped_points_copy_event_) {
    cudaEventDestroy(num_cropped_points_copy_event_);
  }
  if (num_unique_points32_copy_event_) {
    cudaEventDestroy(num_unique_points32_copy_event_);
  }
  if (num_unique_points64_copy_event_) {
    cudaEventDestroy(num_unique_points64_copy_event_);
  }
}

template <typename PointT>
__global__ void points2FeaturesKernel(
  const PointT * __restrict__ input_points, std::size_t points_size,
  float4 * __restrict__ output_points)
{
  const auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= points_size) {
    return;
  }

  const PointT & input_point = input_points[idx];
  float4 & output_point = output_points[idx];
  output_point.x = input_point.x;
  output_point.y = input_point.y;
  output_point.z = input_point.z;
  output_point.w = static_cast<float>(input_point.intensity) / 255.f;
}

template <>
__global__ void points2FeaturesKernel<CloudPointTypeXYZI>(
  const CloudPointTypeXYZI * __restrict__ input_points, std::size_t points_size,
  float4 * __restrict__ output_points)
{
  const auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= points_size) {
    return;
  }

  const auto & input_point = input_points[idx];
  auto & output_point = output_points[idx];
  output_point.x = input_point.x;
  output_point.y = input_point.y;
  output_point.z = input_point.z;
  output_point.w = input_point.intensity;
}

template <>
__global__ void points2FeaturesKernel<CloudPointTypeXYZIRADRT>(
  const CloudPointTypeXYZIRADRT * __restrict__ input_points, std::size_t points_size,
  float4 * __restrict__ output_points)
{
  const auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= points_size) {
    return;
  }

  const auto & input_point = input_points[idx];
  auto & output_point = output_points[idx];
  output_point.x = input_point.x;
  output_point.y = input_point.y;
  output_point.z = input_point.z;
  output_point.w = input_point.intensity;
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
  const scalar_t * __restrict__ input_data, mask_t * __restrict__ masks,
  mask_t * __restrict__ indices, scalar_t * __restrict__ output_data, int num_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points && masks[idx] == 1) {
    output_data[indices[idx] - 1] = input_data[idx];
  }
}

template <typename scalar_t, typename mask_t>
__global__ void extractIndicesKernel(
  const scalar_t * __restrict__ input_data, mask_t * __restrict__ masks,
  mask_t * __restrict__ indices1, mask_t * __restrict__ indices2,
  scalar_t * __restrict__ output_data, int num_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points && masks[idx] == 1) {
    output_data[indices1[idx] - 1] = input_data[indices2[idx]];
  }
}

template <typename mask_t>
__global__ void scatterInverseMapKernel(
  const mask_t * __restrict__ unique_indices, const mask_t * __restrict__ sorted_hash_indexes,
  std::int64_t * __restrict__ inverse_map, int num_points)
{
  const auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  inverse_map[sorted_hash_indexes[idx]] = static_cast<std::int64_t>(unique_indices[idx] - 1);
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
  const float4 * __restrict__ points, int3 * __restrict__ coords,
  std::int64_t * __restrict__ hashes, int num_points, float voxel_size_x, float voxel_size_y,
  float voxel_size_z, std::int32_t min_x, std::int32_t min_y, std::int32_t min_z, int depth)
{
  static_assert(sizeof(int3) == sizeof(std::int32_t) * 3, "int3 must be 12 bytes");
  auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  const float4 & point = points[idx];
  const auto x = static_cast<std::int32_t>(std::floor(point.x / voxel_size_x) - min_x);
  const auto y = static_cast<std::int32_t>(std::floor(point.y / voxel_size_y) - min_y);
  const auto z = static_cast<std::int32_t>(std::floor(point.z / voxel_size_z) - min_z);

  coords[idx] = make_int3(x, y, z);

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

constexpr std::int64_t kInvalidPoolingKey = std::numeric_limits<std::int64_t>::max();

__global__ void setInitialStageCountKernel(
  std::int64_t * __restrict__ stage_counts, std::int64_t num_voxels)
{
  *stage_counts = num_voxels;
}

__global__ void preparePoolingSortInputKernel(
  const std::int64_t * __restrict__ serialized_code, const std::int64_t * __restrict__ stage_counts,
  std::int64_t * __restrict__ keys, std::int64_t * __restrict__ indices, std::int32_t stage_index,
  std::int32_t pooling_depth, std::int64_t capacity)
{
  const auto idx = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (idx >= capacity) {
    return;
  }

  const auto input_count = stage_counts[stage_index];
  keys[idx] = idx < input_count ? serialized_code[idx] >> (pooling_depth * 3) : kInvalidPoolingKey;
  indices[idx] = idx;
}

__global__ void markPoolingRunsKernel(
  const std::int64_t * __restrict__ sorted_keys, std::int64_t * __restrict__ run_flags,
  std::int64_t capacity)
{
  const auto idx = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (idx >= capacity) {
    return;
  }

  const auto key = sorted_keys[idx];
  run_flags[idx] = key != kInvalidPoolingKey && (idx == 0 || key != sorted_keys[idx - 1]) ? 1 : 0;
}

__global__ void fillPoolingStageKernel(
  const std::int32_t * __restrict__ grid_coord_in,
  const std::int64_t * __restrict__ serialized_code_in,
  const std::int64_t * __restrict__ sorted_keys, const std::int64_t * __restrict__ sorted_indices,
  const std::int64_t * __restrict__ run_flags, const std::int64_t * __restrict__ run_ids,
  std::int64_t * __restrict__ indices_out, std::int64_t * __restrict__ indptr_out,
  std::int64_t * __restrict__ head_indices_out, std::int64_t * __restrict__ cluster_out,
  std::int32_t * __restrict__ grid_coord_out, std::int64_t * __restrict__ serialized_code_out,
  std::int64_t * __restrict__ stage_counts, std::int32_t stage_index, std::int32_t pooling_depth,
  std::int32_t num_orders, std::int64_t capacity)
{
  const auto idx = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (idx >= capacity) {
    return;
  }

  // The order-major (per-serialization-order) tensors are stored densely so they can be bound
  // directly to the engine inputs of shape [num_orders, count]: the input is strided by the
  // stage's input count (the original serialized_code is laid out [2, num_voxels]) and the output
  // by the stage's output count. Using `capacity` as the stride here would both misread the
  // dense input and produce a non-dense output that TensorRT cannot consume.
  const auto input_count = stage_counts[stage_index];
  const auto next_count = run_ids[capacity - 1];
  if (idx == 0) {
    stage_counts[stage_index + 1] = next_count;
    indptr_out[next_count] = input_count;
  }

  if (sorted_keys[idx] == kInvalidPoolingKey) {
    return;
  }

  const auto input_index = sorted_indices[idx];
  const auto segment_index = run_ids[idx] - 1;
  indices_out[idx] = input_index;
  cluster_out[input_index] = segment_index;

  if (run_flags[idx] == 0) {
    return;
  }

  indptr_out[segment_index] = idx;
  head_indices_out[segment_index] = input_index;
  for (std::int32_t coord_index = 0; coord_index < 3; ++coord_index) {
    grid_coord_out[segment_index * 3 + coord_index] =
      grid_coord_in[input_index * 3 + coord_index] >> pooling_depth;
  }
  for (std::int32_t order_index = 0; order_index < num_orders; ++order_index) {
    serialized_code_out[order_index * next_count + segment_index] =
      serialized_code_in[order_index * input_count + input_index] >> (pooling_depth * 3);
  }
}

__global__ void prepareOrderSortInputKernel(
  const std::int64_t * __restrict__ serialized_code, const std::int64_t * __restrict__ stage_counts,
  std::int64_t * __restrict__ keys, std::int64_t * __restrict__ indices, std::int32_t stage_index,
  std::int32_t order_index, std::int64_t capacity)
{
  const auto idx = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (idx >= capacity) {
    return;
  }

  const auto input_count = stage_counts[stage_index];
  // serialized_code is stored densely as [num_orders, input_count] (see fillPoolingStageKernel).
  keys[idx] =
    idx < input_count ? serialized_code[order_index * input_count + idx] : kInvalidPoolingKey;
  indices[idx] = idx;
}

__global__ void fillOrderAndInverseKernel(
  const std::int64_t * __restrict__ sorted_keys, const std::int64_t * __restrict__ sorted_indices,
  const std::int64_t * __restrict__ stage_counts, std::int64_t * __restrict__ order_out,
  std::int64_t * __restrict__ inverse_out, std::int32_t stage_index, std::int32_t order_index,
  std::int64_t capacity)
{
  const auto count = stage_counts[stage_index];
  const auto rank = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (rank >= capacity || rank >= count || sorted_keys[rank] == kInvalidPoolingKey) {
    return;
  }

  // order/inverse are stored densely as [num_orders, count] to match the engine input layout.
  const auto input_index = sorted_indices[rank];
  order_out[order_index * count + rank] = input_index;
  inverse_out[order_index * count + input_index] = rank;
}

std::int32_t poolingDepth(const std::int64_t stride)
{
  std::int32_t depth = 0;
  for (auto value = stride; value > 1; value >>= 1) {
    ++depth;
  }
  return depth;
}

void PreprocessCuda::generateSerializedPoolingMetadata(
  const std::int32_t * grid_coord, const std::int64_t * serialized_code, std::int64_t num_voxels,
  const std::vector<SerializedPoolingDeviceStageView> & stages, std::int64_t * stage_counts)
{
  if (stages.size() != config_.pooling_strides_.size()) {
    throw std::runtime_error("Serialized pooling stage buffer count does not match config.");
  }

  const auto capacity = config_.max_num_voxels_;
  const auto num_orders = static_cast<std::int32_t>(config_.serialization_orders_.size());
  const auto num_blocks = divup(static_cast<std::size_t>(capacity), config_.threads_per_block_);

  // The sort keys are serialized (Morton/Z-order) codes interleaving 3 grid coordinates of
  // serialization_depth_ bits each, so they occupy at most 3 * serialization_depth_ bits (the MSB
  // sits at 3 * serialization_depth_ - 1). Pooling stages only right-shift the codes, which can
  // never raise the MSB, so this is a valid upper bound for every stage. The INT64_MAX padding
  // sentinel still sorts last because its low bits are all set, and the full key value is preserved
  // by the radix sort, so exact-equality sentinel checks remain valid. std::max guards a degenerate
  // single-voxel grid (serialization_depth_ == 0), since CUB requires end_bit > begin_bit.
  const int pooling_end_bit = std::max(1, 3 * config_.serialization_depth_);

  setInitialStageCountKernel<<<1, 1, 0, stream_>>>(stage_counts, num_voxels);
  CHECK_CUDA_ERROR(cudaPeekAtLastError());

  const std::int32_t * current_grid_coord = grid_coord;
  const std::int64_t * current_serialized_code = serialized_code;

  for (std::size_t stage_index = 0; stage_index < stages.size(); ++stage_index) {
    const auto & stage = stages[stage_index];
    const auto pooling_depth = poolingDepth(config_.pooling_strides_[stage_index]);

    preparePoolingSortInputKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
      current_serialized_code, stage_counts, pooling_keys_d_.get(), pooling_indices_d_.get(),
      static_cast<std::int32_t>(stage_index), pooling_depth, capacity);
    CHECK_CUDA_ERROR(cudaPeekAtLastError());

    CHECK_CUDA_ERROR(
      cub::DeviceRadixSort::SortPairs(
        pooling_workspace_d_.get(), pooling_workspace_size_, pooling_keys_d_.get(),
        pooling_sorted_keys_d_.get(), pooling_indices_d_.get(), pooling_sorted_indices_d_.get(),
        capacity, 0, pooling_end_bit, stream_));

    markPoolingRunsKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
      pooling_sorted_keys_d_.get(), pooling_run_flags_d_.get(), capacity);
    CHECK_CUDA_ERROR(cudaPeekAtLastError());

    CHECK_CUDA_ERROR(
      cub::DeviceScan::InclusiveSum(
        pooling_workspace_d_.get(), pooling_workspace_size_, pooling_run_flags_d_.get(),
        pooling_run_ids_d_.get(), capacity, stream_));

    fillPoolingStageKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
      current_grid_coord, current_serialized_code, pooling_sorted_keys_d_.get(),
      pooling_sorted_indices_d_.get(), pooling_run_flags_d_.get(), pooling_run_ids_d_.get(),
      stage.indices, stage.indptr, stage.head_indices, stage.cluster, stage.grid_coord,
      stage.serialized_code, stage_counts, static_cast<std::int32_t>(stage_index), pooling_depth,
      num_orders, capacity);
    CHECK_CUDA_ERROR(cudaPeekAtLastError());

    for (std::int32_t order_index = 0; order_index < num_orders; ++order_index) {
      prepareOrderSortInputKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
        stage.serialized_code, stage_counts, pooling_keys_d_.get(), pooling_indices_d_.get(),
        static_cast<std::int32_t>(stage_index + 1), order_index, capacity);
      CHECK_CUDA_ERROR(cudaPeekAtLastError());

      CHECK_CUDA_ERROR(
        cub::DeviceRadixSort::SortPairs(
          pooling_workspace_d_.get(), pooling_workspace_size_, pooling_keys_d_.get(),
          pooling_sorted_keys_d_.get(), pooling_indices_d_.get(), pooling_sorted_indices_d_.get(),
          capacity, 0, pooling_end_bit, stream_));

      fillOrderAndInverseKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
        pooling_sorted_keys_d_.get(), pooling_sorted_indices_d_.get(), stage_counts,
        stage.serialized_order, stage.serialized_inverse,
        static_cast<std::int32_t>(stage_index + 1), order_index, capacity);
      CHECK_CUDA_ERROR(cudaPeekAtLastError());
    }

    current_grid_coord = stage.grid_coord;
    current_serialized_code = stage.serialized_code;
  }
}

std::size_t PreprocessCuda::generateFeatures(
  const void * input_data, CloudFormat input_format, unsigned int num_points,
  float * voxel_features, std::int32_t * voxel_coords, std::int64_t * voxel_hashes,
  void * compact_points, float * reconstruction_features, void * cropped_source_points,
  std::int64_t * inverse_map, std::size_t * output_num_cropped_points)
{
  const auto num_blocks = divup(num_points, config_.threads_per_block_);
  switch (input_format) {
    case CloudFormat::XYZIRCAEDT:
      points2FeaturesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
        static_cast<const CloudPointTypeXYZIRCAEDT *>(input_data), num_points,
        reinterpret_cast<float4 *>(points_d_.get()));
      break;
    case CloudFormat::XYZIRADRT:
      points2FeaturesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
        static_cast<const CloudPointTypeXYZIRADRT *>(input_data), num_points,
        reinterpret_cast<float4 *>(points_d_.get()));
      break;
    case CloudFormat::XYZIRC:
      points2FeaturesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
        static_cast<const CloudPointTypeXYZIRC *>(input_data), num_points,
        reinterpret_cast<float4 *>(points_d_.get()));
      break;
    case CloudFormat::XYZI:
      points2FeaturesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
        static_cast<const CloudPointTypeXYZI *>(input_data), num_points,
        reinterpret_cast<float4 *>(points_d_.get()));
      break;
    default:
      throw std::runtime_error("Unsupported input point cloud format.");
  }

  // FULL reconstruction preserves original input order, so copy features before range crop.
  if (
    config_.source_reconstruction_ == SourceReconstruction::FULL &&
    reconstruction_features != nullptr) {
    cudaMemcpyAsync(
      reconstruction_features, points_d_.get(),
      num_points * config_.num_point_feature_size_ * sizeof(float), cudaMemcpyDeviceToDevice,
      stream_);
  }

  cropKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
    reinterpret_cast<float4 *>(points_d_.get()), crop_mask_d_.get(), num_points,
    config_.min_x_range_, config_.min_y_range_, config_.min_z_range_, config_.max_x_range_,
    config_.max_y_range_, config_.max_z_range_);

  CHECK_CUDA_ERROR(
    cub::DeviceScan::InclusiveSum(
      generate_feature_workspace_d_.get(), generate_feature_workspace_size_, crop_mask_d_.get(),
      crop_indices_d_.get(), num_points, stream_));

  *num_cropped_points_ = 0;

  cudaMemcpyAsync(
    num_cropped_points_.get(), crop_indices_d_.get() + num_points - 1, sizeof(std::uint32_t),
    cudaMemcpyDeviceToHost, stream_);
  CHECK_CUDA_ERROR(
    cudaEventRecord(num_cropped_points_copy_event_, stream_));  // Lazy sync. use later

  extractIndicesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
    reinterpret_cast<float4 *>(points_d_.get()), crop_mask_d_.get(), crop_indices_d_.get(),
    reinterpret_cast<float4 *>(cropped_points_d_.get()), num_points);

  // PARTIAL reconstruction publishes only in-range points, so compact features after range crop.
  if (
    config_.source_reconstruction_ == SourceReconstruction::PARTIAL &&
    reconstruction_features != nullptr) {
    extractIndicesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
      reinterpret_cast<float4 *>(points_d_.get()), crop_mask_d_.get(), crop_indices_d_.get(),
      reinterpret_cast<float4 *>(reconstruction_features), num_points);
  }

  switch (input_format) {
    case CloudFormat::XYZIRCAEDT:
      extractIndicesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
        static_cast<const CloudPointTypeXYZIRCAEDT *>(input_data), crop_mask_d_.get(),
        crop_indices_d_.get(),
        reinterpret_cast<CloudPointTypeXYZIRCAEDT *>(cropped_input_points_d_.get()), num_points);
      if (cropped_source_points != nullptr) {
        extractIndicesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
          static_cast<const CloudPointTypeXYZIRCAEDT *>(input_data), crop_mask_d_.get(),
          crop_indices_d_.get(),
          reinterpret_cast<CloudPointTypeXYZIRCAEDT *>(cropped_source_points), num_points);
      }
      break;
    case CloudFormat::XYZIRADRT:
      extractIndicesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
        static_cast<const CloudPointTypeXYZIRADRT *>(input_data), crop_mask_d_.get(),
        crop_indices_d_.get(),
        reinterpret_cast<CloudPointTypeXYZIRADRT *>(cropped_input_points_d_.get()), num_points);
      if (cropped_source_points != nullptr) {
        extractIndicesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
          static_cast<const CloudPointTypeXYZIRADRT *>(input_data), crop_mask_d_.get(),
          crop_indices_d_.get(), reinterpret_cast<CloudPointTypeXYZIRADRT *>(cropped_source_points),
          num_points);
      }
      break;
    case CloudFormat::XYZIRC:
      extractIndicesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
        static_cast<const CloudPointTypeXYZIRC *>(input_data), crop_mask_d_.get(),
        crop_indices_d_.get(),
        reinterpret_cast<CloudPointTypeXYZIRC *>(cropped_input_points_d_.get()), num_points);
      if (cropped_source_points != nullptr) {
        extractIndicesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
          static_cast<const CloudPointTypeXYZIRC *>(input_data), crop_mask_d_.get(),
          crop_indices_d_.get(), reinterpret_cast<CloudPointTypeXYZIRC *>(cropped_source_points),
          num_points);
      }
      break;
    case CloudFormat::XYZI:
      extractIndicesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
        static_cast<const CloudPointTypeXYZI *>(input_data), crop_mask_d_.get(),
        crop_indices_d_.get(),
        reinterpret_cast<CloudPointTypeXYZI *>(cropped_input_points_d_.get()), num_points);
      if (cropped_source_points != nullptr) {
        extractIndicesKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
          static_cast<const CloudPointTypeXYZI *>(input_data), crop_mask_d_.get(),
          crop_indices_d_.get(), reinterpret_cast<CloudPointTypeXYZI *>(cropped_source_points),
          num_points);
      }
      break;
    default:
      throw std::runtime_error("Unsupported input point cloud format.");
  }

  CHECK_CUDA_ERROR(cudaEventSynchronize(num_cropped_points_copy_event_));

  if (*num_cropped_points_ == 0) {
    *output_num_cropped_points = 0;
    return 0;
  }
  *output_num_cropped_points = *num_cropped_points_;

  const auto coord_min_x =
    static_cast<std::int32_t>(std::floor(config_.min_x_range_ / config_.voxel_x_size_));
  const auto coord_min_y =
    static_cast<std::int32_t>(std::floor(config_.min_y_range_ / config_.voxel_y_size_));
  const auto coord_min_z =
    static_cast<std::int32_t>(std::floor(config_.min_z_range_ / config_.voxel_z_size_));

  const auto num_cropped_blocks = divup(*num_cropped_points_, config_.threads_per_block_);

  std::uint64_t num_unique_points;

  if (config_.use_64bit_hash_) {
    voxelizationHash64Kernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
      reinterpret_cast<float4 *>(cropped_points_d_.get()), hashes64_d_.get(), *num_cropped_points_,
      config_.voxel_x_size_, config_.voxel_y_size_, config_.voxel_z_size_, coord_min_x, coord_min_y,
      coord_min_z);

    // Keys are FNV-1a hashes spread pseudo-randomly across the full 64-bit range, so there is no
    // usable upper bound below 2^64; sort all 64 bits.
    CHECK_CUDA_ERROR(
      cub::DeviceRadixSort::SortPairs(
        reinterpret_cast<void *>(generate_feature_workspace_d_.get()),
        generate_feature_workspace_size_, hashes64_d_.get(), sorted_hashes64_d_.get(),
        hash_indexes64_d_.get(), sorted_hash_indexes64_d_.get(), *num_cropped_points_, 0, 64,
        stream_));

    CHECK_CUDA_ERROR(
      cub::DeviceAdjacentDifference::SubtractLeftCopy(
        generate_feature_workspace_d_.get(), generate_feature_workspace_size_,
        sorted_hashes64_d_.get(), unique_mask64_d_.get(), *num_cropped_points_, NotEqual{},
        stream_));

    std::uint64_t one = 1;
    cudaMemcpyAsync(
      unique_mask64_d_.get(), &one, sizeof(std::uint64_t), cudaMemcpyHostToDevice, stream_);

    CHECK_CUDA_ERROR(
      cub::DeviceScan::InclusiveSum(
        generate_feature_workspace_d_.get(), generate_feature_workspace_size_,
        unique_mask64_d_.get(), unique_indices64_d_.get(), *num_cropped_points_, stream_));

    *num_unique_points64_ = 0;
    cudaMemcpyAsync(
      num_unique_points64_.get(), unique_indices64_d_.get() + *num_cropped_points_ - 1,
      sizeof(std::int64_t), cudaMemcpyDeviceToHost, stream_);
    CHECK_CUDA_ERROR(cudaEventRecord(num_unique_points64_copy_event_, stream_));

    extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
      reinterpret_cast<float4 *>(cropped_points_d_.get()), unique_mask64_d_.get(),
      unique_indices64_d_.get(), sorted_hash_indexes64_d_.get(),
      reinterpret_cast<float4 *>(voxel_features), *num_cropped_points_);
    if (inverse_map != nullptr) {
      scatterInverseMapKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
        unique_indices64_d_.get(), sorted_hash_indexes64_d_.get(), inverse_map,
        *num_cropped_points_);
    }

    switch (input_format) {
      case CloudFormat::XYZIRCAEDT:
        extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
          reinterpret_cast<CloudPointTypeXYZIRCAEDT *>(cropped_input_points_d_.get()),
          unique_mask64_d_.get(), unique_indices64_d_.get(), sorted_hash_indexes64_d_.get(),
          reinterpret_cast<CloudPointTypeXYZIRCAEDT *>(compact_points), *num_cropped_points_);
        break;
      case CloudFormat::XYZIRADRT:
        extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
          reinterpret_cast<CloudPointTypeXYZIRADRT *>(cropped_input_points_d_.get()),
          unique_mask64_d_.get(), unique_indices64_d_.get(), sorted_hash_indexes64_d_.get(),
          reinterpret_cast<CloudPointTypeXYZIRADRT *>(compact_points), *num_cropped_points_);
        break;
      case CloudFormat::XYZIRC:
        extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
          reinterpret_cast<CloudPointTypeXYZIRC *>(cropped_input_points_d_.get()),
          unique_mask64_d_.get(), unique_indices64_d_.get(), sorted_hash_indexes64_d_.get(),
          reinterpret_cast<CloudPointTypeXYZIRC *>(compact_points), *num_cropped_points_);
        break;
      case CloudFormat::XYZI:
        extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
          reinterpret_cast<CloudPointTypeXYZI *>(cropped_input_points_d_.get()),
          unique_mask64_d_.get(), unique_indices64_d_.get(), sorted_hash_indexes64_d_.get(),
          reinterpret_cast<CloudPointTypeXYZI *>(compact_points), *num_cropped_points_);
        break;
      default:
        throw std::runtime_error("Unsupported input point cloud format.");
    }

    CHECK_CUDA_ERROR(cudaEventSynchronize(num_unique_points64_copy_event_));
    num_unique_points = *num_unique_points64_;

  } else {
    voxelizationHash32Kernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
      reinterpret_cast<float4 *>(cropped_points_d_.get()), hashes32_d_.get(), *num_cropped_points_,
      config_.voxel_x_size_, config_.voxel_y_size_, config_.voxel_z_size_, config_.min_x_range_,
      config_.min_y_range_, config_.min_z_range_, static_cast<std::uint32_t>(config_.grid_x_size_),
      static_cast<std::uint32_t>(config_.grid_x_size_ * config_.grid_y_size_));

    CHECK_CUDA_ERROR(
      cub::DeviceRadixSort::SortPairs(
        reinterpret_cast<void *>(generate_feature_workspace_d_.get()),
        generate_feature_workspace_size_, hashes32_d_.get(), sorted_hashes32_d_.get(),
        hash_indexes32_d_.get(), sorted_hash_indexes32_d_.get(), *num_cropped_points_, 0, 32,
        stream_));

    CHECK_CUDA_ERROR(
      cub::DeviceAdjacentDifference::SubtractLeftCopy(
        generate_feature_workspace_d_.get(), generate_feature_workspace_size_,
        sorted_hashes32_d_.get(), unique_mask32_d_.get(), *num_cropped_points_, NotEqual{},
        stream_));

    std::uint32_t one = 1;
    cudaMemcpyAsync(
      unique_mask32_d_.get(), &one, sizeof(std::uint32_t), cudaMemcpyHostToDevice, stream_);

    CHECK_CUDA_ERROR(
      cub::DeviceScan::InclusiveSum(
        generate_feature_workspace_d_.get(), generate_feature_workspace_size_,
        unique_mask32_d_.get(), unique_indices32_d_.get(), *num_cropped_points_, stream_));

    *num_unique_points32_ = 0;
    cudaMemcpyAsync(
      num_unique_points32_.get(), unique_indices32_d_.get() + *num_cropped_points_ - 1,
      sizeof(std::uint32_t), cudaMemcpyDeviceToHost, stream_);
    CHECK_CUDA_ERROR(
      cudaEventRecord(num_unique_points32_copy_event_, stream_));  // Lazy sync. use later

    extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
      reinterpret_cast<float4 *>(cropped_points_d_.get()), unique_mask32_d_.get(),
      unique_indices32_d_.get(), sorted_hash_indexes32_d_.get(),
      reinterpret_cast<float4 *>(voxel_features), *num_cropped_points_);
    if (inverse_map != nullptr) {
      scatterInverseMapKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
        unique_indices32_d_.get(), sorted_hash_indexes32_d_.get(), inverse_map,
        *num_cropped_points_);
    }

    switch (input_format) {
      case CloudFormat::XYZIRCAEDT:
        extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
          reinterpret_cast<CloudPointTypeXYZIRCAEDT *>(cropped_input_points_d_.get()),
          unique_mask32_d_.get(), unique_indices32_d_.get(), sorted_hash_indexes32_d_.get(),
          reinterpret_cast<CloudPointTypeXYZIRCAEDT *>(compact_points), *num_cropped_points_);
        break;
      case CloudFormat::XYZIRADRT:
        extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
          reinterpret_cast<CloudPointTypeXYZIRADRT *>(cropped_input_points_d_.get()),
          unique_mask32_d_.get(), unique_indices32_d_.get(), sorted_hash_indexes32_d_.get(),
          reinterpret_cast<CloudPointTypeXYZIRADRT *>(compact_points), *num_cropped_points_);
        break;
      case CloudFormat::XYZIRC:
        extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
          reinterpret_cast<CloudPointTypeXYZIRC *>(cropped_input_points_d_.get()),
          unique_mask32_d_.get(), unique_indices32_d_.get(), sorted_hash_indexes32_d_.get(),
          reinterpret_cast<CloudPointTypeXYZIRC *>(compact_points), *num_cropped_points_);
        break;
      case CloudFormat::XYZI:
        extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
          reinterpret_cast<CloudPointTypeXYZI *>(cropped_input_points_d_.get()),
          unique_mask32_d_.get(), unique_indices32_d_.get(), sorted_hash_indexes32_d_.get(),
          reinterpret_cast<CloudPointTypeXYZI *>(compact_points), *num_cropped_points_);
        break;
      default:
        throw std::runtime_error("Unsupported input point cloud format.");
    }

    CHECK_CUDA_ERROR(cudaEventSynchronize(num_unique_points32_copy_event_));
    num_unique_points = static_cast<std::uint64_t>(*num_unique_points32_);
  }

  computeGridCoordsAndSerializationKernel<<<
    num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
    reinterpret_cast<float4 *>(voxel_features), reinterpret_cast<int3 *>(voxel_coords),
    voxel_hashes, num_unique_points, config_.voxel_x_size_, config_.voxel_y_size_,
    config_.voxel_z_size_, coord_min_x, coord_min_y, coord_min_z, config_.serialization_depth_);

  return num_unique_points;
}

}  // namespace autoware::ptv3
