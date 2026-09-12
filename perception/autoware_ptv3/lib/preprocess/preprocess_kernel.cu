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
#include <cassert>
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

  codes_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.cloud_capacity_);
  sorted_codes_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.cloud_capacity_);
  code_indices_d_ = autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.cloud_capacity_);
  sorted_code_indices_d_ =
    autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.cloud_capacity_);
  unique_mask_d_ = autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.cloud_capacity_);
  unique_indices_d_ = autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.cloud_capacity_);

  thrust::device_ptr<std::uint32_t> idx_ptr(code_indices_d_.get());
  thrust::sequence(policy, idx_ptr, idx_ptr + config_.cloud_capacity_, 0);

  // Serialized codes occupy 3 * serialization_depth_ bits, and pooling only right-shifts them, so
  // this bound holds for every stage. std::max guards serialization_depth_ == 0 (CUB requires
  // end_bit > begin_bit).
  code_sort_end_bit_ = std::max(1, 3 * config_.serialization_depth_);

  std::int64_t * int64_nullptr = nullptr;
  std::uint32_t * uint32_nullptr = nullptr;

  std::size_t sort_pair_workspace_size = 0;
  std::size_t inclusive_sum_workspace_size = 0;
  std::size_t adjacent_difference_workspace_size = 0;

  CHECK_CUDA_ERROR(
    cub::DeviceRadixSort::SortPairs(
      nullptr, sort_pair_workspace_size, int64_nullptr, int64_nullptr, uint32_nullptr,
      uint32_nullptr, config_.cloud_capacity_, 0, 64, nullptr));
  CHECK_CUDA_ERROR(
    cub::DeviceScan::InclusiveSum(
      nullptr, inclusive_sum_workspace_size, uint32_nullptr, uint32_nullptr,
      config_.cloud_capacity_));
  CHECK_CUDA_ERROR(
    cub::DeviceAdjacentDifference::SubtractLeftCopy(
      nullptr, adjacent_difference_workspace_size, int64_nullptr, uint32_nullptr,
      config_.cloud_capacity_, NotEqual{}));

  generate_feature_workspace_size_ = std::max(
    {sort_pair_workspace_size, inclusive_sum_workspace_size, adjacent_difference_workspace_size});
  generate_feature_workspace_d_ =
    autoware::cuda_utils::make_unique<std::uint8_t[]>(generate_feature_workspace_size_);

  num_cropped_points_ = autoware::cuda_utils::make_unique_host<std::uint32_t>();
  num_unique_points_ = autoware::cuda_utils::make_unique_host<std::uint32_t>();

  const auto num_orders = static_cast<std::int64_t>(config_.serialization_orders_.size());
  input_level_order_d_ =
    autoware::cuda_utils::make_unique<std::int64_t[]>(num_orders * config_.max_num_voxels_);
  order_sort_keys_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);
  order_sort_sorted_keys_d_ =
    autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);
  order_sort_indices_d_ =
    autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);
  run_flags_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);
  run_ids_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);

  std::size_t pooling_sort_workspace_size = 0;
  std::size_t pooling_scan_workspace_size = 0;
  cub::DeviceRadixSort::SortPairs(
    nullptr, pooling_sort_workspace_size, int64_nullptr, int64_nullptr, int64_nullptr,
    int64_nullptr, config_.max_num_voxels_, 0, 64, nullptr);
  cub::DeviceScan::InclusiveSum(
    nullptr, pooling_scan_workspace_size, int64_nullptr, int64_nullptr, config_.max_num_voxels_,
    nullptr);
  pooling_workspace_size_ = std::max(pooling_sort_workspace_size, pooling_scan_workspace_size);
  pooling_workspace_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(pooling_workspace_size_);

  CHECK_CUDA_ERROR(
    cudaEventCreateWithFlags(&num_cropped_points_copy_event_, cudaEventDisableTiming));
  CHECK_CUDA_ERROR(
    cudaEventCreateWithFlags(&num_unique_points_copy_event_, cudaEventDisableTiming));

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

PreprocessCuda::~PreprocessCuda()
{
  if (num_cropped_points_copy_event_) {
    cudaEventDestroy(num_cropped_points_copy_event_);
  }
  if (num_unique_points_copy_event_) {
    cudaEventDestroy(num_unique_points_copy_event_);
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

  mask[idx] = x >= min_x && x < max_x && y >= min_y && y < max_y && z >= min_z && z < max_z;
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
  scalar_t * __restrict__ output_data, int num_points, int max_num_voxels)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points && masks[idx] == 1) {
    // indices1 is a 1-based unique-voxel prefix-sum id. Scattered clouds can produce more unique
    // voxels than max_num_voxels; drop the overflow here (mirrors the host-side clip in
    // PTv3TRT::preProcess) instead of writing past the max_num_voxels-sized output buffer.
    const auto out_index = static_cast<std::int64_t>(indices1[idx]) - 1;
    if (out_index < max_num_voxels) {
      output_data[out_index] = input_data[indices2[idx]];
    }
  }
}

template <typename mask_t>
__global__ void scatterInverseMapKernel(
  const mask_t * __restrict__ unique_indices, const mask_t * __restrict__ sorted_code_indices,
  std::int64_t * __restrict__ inverse_map, int num_points)
{
  const auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  inverse_map[sorted_code_indices[idx]] = static_cast<std::int64_t>(unique_indices[idx] - 1);
}

/**
 * @brief Interleaves the three grid coordinates into a serialized (Morton / Z-order) code,
 * `depth` bits per axis.
 *
 * The bit layout must match autoware-ml's z_order_encode, which the model was trained against.
 *
 * @param x Grid coordinate on the x axis.
 * @param y Grid coordinate on the y axis.
 * @param z Grid coordinate on the z axis.
 * @param depth Number of bits consumed per axis.
 * @param transposed Swap x and y ("z-trans" order).
 * @return The serialized code.
 */
__device__ inline std::int64_t serializeCoord(
  const std::int32_t x, const std::int32_t y, const std::int32_t z, const int depth,
  const bool transposed)
{
  const std::int32_t major = transposed ? y : x;
  const std::int32_t minor = transposed ? x : y;

  std::int64_t code = 0;
  for (int i = 0; i < depth; ++i) {
    const std::int64_t mask = 1 << i;
    code |= ((major & mask) << (2 * i + 2));
    code |= ((minor & mask) << (2 * i + 1));
    code |= ((z & mask) << (2 * i + 0));
  }
  return code;
}

/**
 * @brief Maps a point to its grid coordinate. Single helper so the voxelization key and the
 * serialization codes always place a point in the same cell.
 *
 * @param point Point position; only x, y and z are read.
 * @param voxel_size_x Voxel edge length on the x axis.
 * @param voxel_size_y Voxel edge length on the y axis.
 * @param voxel_size_z Voxel edge length on the z axis.
 * @param min_x Grid origin on the x axis, in cells.
 * @param min_y Grid origin on the y axis, in cells.
 * @param min_z Grid origin on the z axis, in cells.
 * @return The grid coordinate.
 */
__device__ inline int3 gridCoord(
  const float4 & point, const float voxel_size_x, const float voxel_size_y,
  const float voxel_size_z, const std::int32_t min_x, const std::int32_t min_y,
  const std::int32_t min_z)
{
  return make_int3(
    static_cast<std::int32_t>(std::floor(point.x / voxel_size_x) - min_x),
    static_cast<std::int32_t>(std::floor(point.y / voxel_size_y) - min_y),
    static_cast<std::int32_t>(std::floor(point.z / voxel_size_z) - min_z));
}

/**
 * @brief Produces the voxelization key: the order-0 serialized code.
 *
 * The code is unique per grid cell, so sorting by it deduplicates voxels and leaves them in
 * order-0 serialization order.
 *
 * @param points Input points.
 * @param codes Output code per point.
 * @param num_points Number of points.
 * @param voxel_size_x Voxel edge length on the x axis.
 * @param voxel_size_y Voxel edge length on the y axis.
 * @param voxel_size_z Voxel edge length on the z axis.
 * @param min_x Grid origin on the x axis, in cells.
 * @param min_y Grid origin on the y axis, in cells.
 * @param min_z Grid origin on the z axis, in cells.
 * @param depth Serialization depth: bits per axis in the code.
 */
__global__ void voxelizationCodeKernel(
  const float4 * __restrict__ points, std::int64_t * __restrict__ codes, int num_points,
  float voxel_size_x, float voxel_size_y, float voxel_size_z, std::int32_t min_x,
  std::int32_t min_y, std::int32_t min_z, int depth)
{
  auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  const auto coord =
    gridCoord(points[idx], voxel_size_x, voxel_size_y, voxel_size_z, min_x, min_y, min_z);
  codes[idx] = serializeCoord(coord.x, coord.y, coord.z, depth, false);
}

__global__ void computeGridCoordsAndSerializationKernel(
  const float4 * __restrict__ points, int3 * __restrict__ coords, std::int64_t * __restrict__ codes,
  int num_points, float voxel_size_x, float voxel_size_y, float voxel_size_z, std::int32_t min_x,
  std::int32_t min_y, std::int32_t min_z, int depth)
{
  static_assert(sizeof(int3) == sizeof(std::int32_t) * 3, "int3 must be 12 bytes");
  auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  const auto coord =
    gridCoord(points[idx], voxel_size_x, voxel_size_y, voxel_size_z, min_x, min_y, min_z);
  coords[idx] = coord;

  codes[idx] = serializeCoord(coord.x, coord.y, coord.z, depth, false);
  codes[idx + num_points] = serializeCoord(coord.x, coord.y, coord.z, depth, true);
}

__global__ void setInitialStageCountKernel(
  std::int64_t * __restrict__ stage_counts_out, std::int64_t num_voxels)
{
  *stage_counts_out = num_voxels;
}

/**
 * @brief Writes the sequence 0..count-1 to `out`.
 *
 * @param out Output array.
 * @param count Number of elements to write.
 */
__global__ void fillIdentityKernel(std::int64_t * __restrict__ out, std::int64_t count)
{
  const auto idx = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (idx >= count) {
    return;
  }
  out[idx] = idx;
}

/**
 * @brief Stages the key/value pair for one of the input-level order sorts.
 *
 * @param serialized_code_in Input voxels' codes, laid out [num_orders, num_voxels]; only the
 * `order_index` row is read.
 * @param keys_out Sort keys: that row's codes.
 * @param indices_out Sort values: 0..num_voxels-1.
 * @param num_voxels Number of input voxels.
 * @param order_index Serialization order being sorted.
 */
__global__ void prepareInputLevelOrderSortKernel(
  const std::int64_t * __restrict__ serialized_code_in, std::int64_t * __restrict__ keys_out,
  std::int64_t * __restrict__ indices_out, std::int64_t num_voxels, std::int32_t order_index)
{
  const auto idx = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (idx >= num_voxels) {
    return;
  }

  keys_out[idx] = serialized_code_in[order_index * num_voxels + idx];
  indices_out[idx] = idx;
}

/**
 * @brief Marks the first child of every parent run.
 *
 * @pre The input level is ascending in order-0 code (asserted below), so the parent code
 * (code >> 3 * pooling_depth) is non-decreasing and each parent's children are already
 * contiguous; comparing against the previous element finds the run starts without sorting.
 *
 * @param serialized_code_in Input level's codes, laid out [num_orders, input_count]; only the
 * order-0 row is read.
 * @param stage_counts_in Per-level voxel counts; entry `stage_index` is the input level's count.
 * @param run_flags_out Flags: 1 at each parent run start, 0 elsewhere (including padding).
 * @param stage_index Level the input arrays describe.
 * @param pooling_depth Bits each grid coordinate is shifted right by, i.e. log2(pooling stride).
 * @param capacity Padded length of the arrays (max_num_voxels).
 */
__global__ void markPoolingRunsKernel(
  const std::int64_t * __restrict__ serialized_code_in,
  const std::int64_t * __restrict__ stage_counts_in, std::int64_t * __restrict__ run_flags_out,
  std::int32_t stage_index, std::int32_t pooling_depth, std::int64_t capacity)
{
  const auto idx = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (idx >= capacity) {
    return;
  }

  const auto input_count = stage_counts_in[stage_index];
  if (idx >= input_count) {
    run_flags_out[idx] = 0;
    return;
  }

  assert(idx == 0 || serialized_code_in[idx - 1] < serialized_code_in[idx]);

  const auto shift = pooling_depth * 3;
  const auto key = serialized_code_in[idx] >> shift;
  run_flags_out[idx] = (idx == 0 || key != (serialized_code_in[idx - 1] >> shift)) ? 1 : 0;
}

/**
 * @brief Emits the pooled level and its gather metadata from the parent-run flags.
 *
 * @pre `run_flags_in` marks the input level's parent-run starts (see markPoolingRunsKernel; zero
 * beyond `input_count`) and `run_ids_in` is its inclusive prefix sum over all `capacity` entries,
 * so `run_ids_in[capacity - 1]` is the pooled voxel count.
 *
 * @param grid_coord_in Input level's grid coordinates, laid out [input_count, 3].
 * @param serialized_code_in Input level's codes, laid out [num_orders, input_count].
 * @param run_flags_in 1 at each parent run start, 0 elsewhere.
 * @param run_ids_in Each input voxel's 1-based parent segment number.
 * @param indices_out Output gather order for the pooling layer; the identity, as the input is
 * already grouped by parent.
 * @param indptr_out Output segment boundaries, [pooled_count + 1]: first input index of each
 * segment, terminated by `input_count`.
 * @param head_indices_out Output representative input voxel per segment: its run start.
 * @param cluster_out Output parent segment index of each input voxel.
 * @param grid_coord_out Output pooled coordinates, [pooled_count, 3]: the input coordinates
 * right-shifted by `pooling_depth`.
 * @param serialized_code_out Output pooled codes, [num_orders, pooled_count]: the input codes
 * right-shifted by `3 * pooling_depth`.
 * @param order_out Output pooled serialization orders, [num_orders, pooled_count]; only the
 * order-0 row is written here, the rest by fillOrderAndInverseKernel.
 * @param inverse_out Output inverse permutations of `order_out`, same layout and coverage.
 * @param stage_counts_inout Per-level voxel counts; entry `stage_index` is the input level's
 * count and entry `stage_index + 1` is set to the pooled count.
 * @param stage_index Level the input arrays describe.
 * @param pooling_depth Bits each grid coordinate is shifted right by, i.e. log2(pooling stride).
 * @param num_orders Number of serialization orders.
 * @param capacity Padded length of the arrays (max_num_voxels).
 */
__global__ void fillPoolingStageKernel(
  const std::int32_t * __restrict__ grid_coord_in,
  const std::int64_t * __restrict__ serialized_code_in,
  const std::int64_t * __restrict__ run_flags_in, const std::int64_t * __restrict__ run_ids_in,
  std::int64_t * __restrict__ indices_out, std::int64_t * __restrict__ indptr_out,
  std::int64_t * __restrict__ head_indices_out, std::int64_t * __restrict__ cluster_out,
  std::int32_t * __restrict__ grid_coord_out, std::int64_t * __restrict__ serialized_code_out,
  std::int64_t * __restrict__ order_out, std::int64_t * __restrict__ inverse_out,
  std::int64_t * __restrict__ stage_counts_inout, std::int32_t stage_index,
  std::int32_t pooling_depth, std::int32_t num_orders, std::int64_t capacity)
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
  const auto input_count = stage_counts_inout[stage_index];
  const auto next_count = run_ids_in[capacity - 1];
  if (idx == 0) {
    stage_counts_inout[stage_index + 1] = next_count;
    indptr_out[next_count] = input_count;
  }

  if (idx >= input_count) {
    return;
  }

  indices_out[idx] = idx;
  const auto segment_index = run_ids_in[idx] - 1;
  cluster_out[idx] = segment_index;

  if (run_flags_in[idx] == 0) {
    return;
  }

  indptr_out[segment_index] = idx;
  head_indices_out[segment_index] = idx;
  for (std::int32_t coord_index = 0; coord_index < 3; ++coord_index) {
    grid_coord_out[segment_index * 3 + coord_index] =
      grid_coord_in[idx * 3 + coord_index] >> pooling_depth;
  }
  for (std::int32_t order_index = 0; order_index < num_orders; ++order_index) {
    serialized_code_out[order_index * next_count + segment_index] =
      serialized_code_in[order_index * input_count + idx] >> (pooling_depth * 3);
  }

  // Segments are emitted in ascending order-0 code, so the pooled level's order-0 row is simply
  // 0..n-1 and markPoolingRunsKernel's precondition holds for the next stage.
  order_out[segment_index] = segment_index;
  inverse_out[segment_index] = segment_index;
}

/**
 * @brief Marks where the parent voxel changes while walking the input level as listed by
 * serialization order `order_index`.
 *
 * Every serialization order visits each parent's children contiguously and the parents in
 * ascending pooled code, so compacting the run heads lists the pooled level's voxels in ascending
 * order-`order_index` code without sorting.
 *
 * @param order_in Input level's serialization orders, laid out [num_orders, input_count].
 * @param cluster_in Parent segment index of each input voxel (see fillPoolingStageKernel).
 * @param stage_counts_in Per-level voxel counts; entry `stage_index` is the input level's count.
 * @param run_flags_out Flags: 1 where the parent changes, 0 elsewhere (including padding).
 * @param stage_index Level the input arrays describe.
 * @param order_index Serialization order being walked.
 * @param capacity Padded length of the arrays (max_num_voxels).
 */
__global__ void markOrderRunsKernel(
  const std::int64_t * __restrict__ order_in, const std::int64_t * __restrict__ cluster_in,
  const std::int64_t * __restrict__ stage_counts_in, std::int64_t * __restrict__ run_flags_out,
  std::int32_t stage_index, std::int32_t order_index, std::int64_t capacity)
{
  const auto idx = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (idx >= capacity) {
    return;
  }

  const auto input_count = stage_counts_in[stage_index];
  if (idx >= input_count) {
    run_flags_out[idx] = 0;
    return;
  }

  const auto segment_index = cluster_in[order_in[order_index * input_count + idx]];
  run_flags_out[idx] =
    (idx == 0 || segment_index != cluster_in[order_in[order_index * input_count + idx - 1]]) ? 1
                                                                                             : 0;
}

__global__ void fillOrderAndInverseKernel(
  const std::int64_t * __restrict__ order_in, const std::int64_t * __restrict__ cluster_in,
  const std::int64_t * __restrict__ run_flags_in, const std::int64_t * __restrict__ run_ids_in,
  const std::int64_t * __restrict__ stage_counts_in, std::int64_t * __restrict__ order_out,
  std::int64_t * __restrict__ inverse_out, std::int32_t stage_index, std::int32_t order_index,
  std::int64_t capacity)
{
  const auto idx = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (idx >= capacity) {
    return;
  }

  const auto input_count = stage_counts_in[stage_index];
  if (idx >= input_count || run_flags_in[idx] == 0) {
    return;
  }

  // order/inverse are stored densely as [num_orders, out_count] to match the engine input layout.
  const auto out_count = stage_counts_in[stage_index + 1];
  const auto out_idx = run_ids_in[idx] - 1;
  const auto segment_index = cluster_in[order_in[order_index * input_count + idx]];
  order_out[order_index * out_count + out_idx] = segment_index;
  inverse_out[order_index * out_count + segment_index] = out_idx;
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
  const auto clamped_num_voxels = std::min(num_voxels, capacity);
  const auto voxel_blocks = divup(
    static_cast<std::size_t>(std::max<std::int64_t>(clamped_num_voxels, 1)),
    config_.threads_per_block_);

  setInitialStageCountKernel<<<1, 1, 0, stream_>>>(stage_counts, clamped_num_voxels);
  CHECK_CUDA_ERROR(cudaPeekAtLastError());

  // The input level is already sorted by order-0 code, so its order-0 row is simply 0..n-1. The
  // remaining orders cannot be derived from it and need one real sort each - the only sorts in
  // this function.
  if (clamped_num_voxels > 0) {
    fillIdentityKernel<<<voxel_blocks, config_.threads_per_block_, 0, stream_>>>(
      input_level_order_d_.get(), clamped_num_voxels);
    CHECK_CUDA_ERROR(cudaPeekAtLastError());

    for (std::int32_t order_index = 1; order_index < num_orders; ++order_index) {
      prepareInputLevelOrderSortKernel<<<voxel_blocks, config_.threads_per_block_, 0, stream_>>>(
        serialized_code, order_sort_keys_d_.get(), order_sort_indices_d_.get(), clamped_num_voxels,
        order_index);
      CHECK_CUDA_ERROR(cudaPeekAtLastError());

      CHECK_CUDA_ERROR(
        cub::DeviceRadixSort::SortPairs(
          pooling_workspace_d_.get(), pooling_workspace_size_, order_sort_keys_d_.get(),
          order_sort_sorted_keys_d_.get(), order_sort_indices_d_.get(),
          input_level_order_d_.get() + order_index * clamped_num_voxels, clamped_num_voxels, 0,
          code_sort_end_bit_, stream_));
    }
  }

  const std::int32_t * current_grid_coord = grid_coord;
  const std::int64_t * current_serialized_code = serialized_code;
  const std::int64_t * current_order = input_level_order_d_.get();

  for (std::size_t stage_index = 0; stage_index < stages.size(); ++stage_index) {
    const auto & stage = stages[stage_index];
    const auto pooling_depth = poolingDepth(config_.pooling_strides_[stage_index]);

    markPoolingRunsKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
      current_serialized_code, stage_counts, run_flags_d_.get(),
      static_cast<std::int32_t>(stage_index), pooling_depth, capacity);
    CHECK_CUDA_ERROR(cudaPeekAtLastError());

    CHECK_CUDA_ERROR(
      cub::DeviceScan::InclusiveSum(
        pooling_workspace_d_.get(), pooling_workspace_size_, run_flags_d_.get(), run_ids_d_.get(),
        capacity, stream_));

    fillPoolingStageKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
      current_grid_coord, current_serialized_code, run_flags_d_.get(), run_ids_d_.get(),
      stage.indices, stage.indptr, stage.head_indices, stage.cluster, stage.grid_coord,
      stage.serialized_code, stage.serialized_order, stage.serialized_inverse, stage_counts,
      static_cast<std::int32_t>(stage_index), pooling_depth, num_orders, capacity);
    CHECK_CUDA_ERROR(cudaPeekAtLastError());

    // Order 0 was already written as the identity by fillPoolingStageKernel above.
    for (std::int32_t order_index = 1; order_index < num_orders; ++order_index) {
      markOrderRunsKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
        current_order, stage.cluster, stage_counts, run_flags_d_.get(),
        static_cast<std::int32_t>(stage_index), order_index, capacity);
      CHECK_CUDA_ERROR(cudaPeekAtLastError());

      CHECK_CUDA_ERROR(
        cub::DeviceScan::InclusiveSum(
          pooling_workspace_d_.get(), pooling_workspace_size_, run_flags_d_.get(), run_ids_d_.get(),
          capacity, stream_));

      fillOrderAndInverseKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
        current_order, stage.cluster, run_flags_d_.get(), run_ids_d_.get(), stage_counts,
        stage.serialized_order, stage.serialized_inverse, static_cast<std::int32_t>(stage_index),
        order_index, capacity);
      CHECK_CUDA_ERROR(cudaPeekAtLastError());
    }

    current_grid_coord = stage.grid_coord;
    current_serialized_code = stage.serialized_code;
    current_order = stage.serialized_order;
  }
}

std::size_t PreprocessCuda::generateFeatures(
  const void * input_data, CloudFormat input_format, unsigned int num_points,
  float * voxel_features, std::int32_t * voxel_coords, std::int64_t * serialized_code,
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

  voxelizationCodeKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
    reinterpret_cast<float4 *>(cropped_points_d_.get()), codes_d_.get(), *num_cropped_points_,
    config_.voxel_x_size_, config_.voxel_y_size_, config_.voxel_z_size_, coord_min_x, coord_min_y,
    coord_min_z, config_.serialization_depth_);

  // This sort both groups duplicates for the compaction below and leaves the voxels in order-0
  // serialization order, which generateSerializedPoolingMetadata requires.
  CHECK_CUDA_ERROR(
    cub::DeviceRadixSort::SortPairs(
      reinterpret_cast<void *>(generate_feature_workspace_d_.get()),
      generate_feature_workspace_size_, codes_d_.get(), sorted_codes_d_.get(),
      code_indices_d_.get(), sorted_code_indices_d_.get(), *num_cropped_points_, 0,
      code_sort_end_bit_, stream_));

  CHECK_CUDA_ERROR(
    cub::DeviceAdjacentDifference::SubtractLeftCopy(
      generate_feature_workspace_d_.get(), generate_feature_workspace_size_, sorted_codes_d_.get(),
      unique_mask_d_.get(), *num_cropped_points_, NotEqual{}, stream_));

  std::uint32_t one = 1;
  cudaMemcpyAsync(
    unique_mask_d_.get(), &one, sizeof(std::uint32_t), cudaMemcpyHostToDevice, stream_);

  CHECK_CUDA_ERROR(
    cub::DeviceScan::InclusiveSum(
      generate_feature_workspace_d_.get(), generate_feature_workspace_size_, unique_mask_d_.get(),
      unique_indices_d_.get(), *num_cropped_points_, stream_));

  *num_unique_points_ = 0;
  cudaMemcpyAsync(
    num_unique_points_.get(), unique_indices_d_.get() + *num_cropped_points_ - 1,
    sizeof(std::uint32_t), cudaMemcpyDeviceToHost, stream_);
  CHECK_CUDA_ERROR(
    cudaEventRecord(num_unique_points_copy_event_, stream_));  // Lazy sync. use later

  extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
    reinterpret_cast<float4 *>(cropped_points_d_.get()), unique_mask_d_.get(),
    unique_indices_d_.get(), sorted_code_indices_d_.get(),
    reinterpret_cast<float4 *>(voxel_features), *num_cropped_points_,
    static_cast<int>(config_.max_num_voxels_));
  if (inverse_map != nullptr) {
    scatterInverseMapKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
      unique_indices_d_.get(), sorted_code_indices_d_.get(), inverse_map, *num_cropped_points_);
  }

  switch (input_format) {
    case CloudFormat::XYZIRCAEDT:
      extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
        reinterpret_cast<CloudPointTypeXYZIRCAEDT *>(cropped_input_points_d_.get()),
        unique_mask_d_.get(), unique_indices_d_.get(), sorted_code_indices_d_.get(),
        reinterpret_cast<CloudPointTypeXYZIRCAEDT *>(compact_points), *num_cropped_points_,
        static_cast<int>(config_.max_num_voxels_));
      break;
    case CloudFormat::XYZIRADRT:
      extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
        reinterpret_cast<CloudPointTypeXYZIRADRT *>(cropped_input_points_d_.get()),
        unique_mask_d_.get(), unique_indices_d_.get(), sorted_code_indices_d_.get(),
        reinterpret_cast<CloudPointTypeXYZIRADRT *>(compact_points), *num_cropped_points_,
        static_cast<int>(config_.max_num_voxels_));
      break;
    case CloudFormat::XYZIRC:
      extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
        reinterpret_cast<CloudPointTypeXYZIRC *>(cropped_input_points_d_.get()),
        unique_mask_d_.get(), unique_indices_d_.get(), sorted_code_indices_d_.get(),
        reinterpret_cast<CloudPointTypeXYZIRC *>(compact_points), *num_cropped_points_,
        static_cast<int>(config_.max_num_voxels_));
      break;
    case CloudFormat::XYZI:
      extractIndicesKernel<<<num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
        reinterpret_cast<CloudPointTypeXYZI *>(cropped_input_points_d_.get()), unique_mask_d_.get(),
        unique_indices_d_.get(), sorted_code_indices_d_.get(),
        reinterpret_cast<CloudPointTypeXYZI *>(compact_points), *num_cropped_points_,
        static_cast<int>(config_.max_num_voxels_));
      break;
    default:
      throw std::runtime_error("Unsupported input point cloud format.");
  }

  CHECK_CUDA_ERROR(cudaEventSynchronize(num_unique_points_copy_event_));
  const auto num_unique_points = static_cast<std::uint64_t>(*num_unique_points_);

  // The extract kernels above dropped any voxels beyond max_num_voxels, so only the first
  // max_num_voxels entries of voxel_features/voxel_coords/serialized_code are valid. Cap the count
  // fed to the grid-coord kernel to that same limit; writing more would overrun those buffers
  // (which are sized max_num_voxels) exactly as the unguarded extract did. The true count is still
  // returned so the caller logs the "over the limit" warning and clips consistently.
  const auto max_num_voxels_u = static_cast<std::uint64_t>(config_.max_num_voxels_);
  const auto num_voxels_capped =
    num_unique_points < max_num_voxels_u ? num_unique_points : max_num_voxels_u;

  computeGridCoordsAndSerializationKernel<<<
    num_cropped_blocks, config_.threads_per_block_, 0, stream_>>>(
    reinterpret_cast<float4 *>(voxel_features), reinterpret_cast<int3 *>(voxel_coords),
    serialized_code, static_cast<int>(num_voxels_capped), config_.voxel_x_size_,
    config_.voxel_y_size_, config_.voxel_z_size_, coord_min_x, coord_min_y, coord_min_z,
    config_.serialization_depth_);

  return num_unique_points;
}

}  // namespace autoware::ptv3
