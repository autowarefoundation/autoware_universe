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

#include "autoware/lidar_frnet/preprocess_kernel.hpp"
#include "autoware/lidar_frnet/utils.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <cub/device/device_radix_sort.cuh>

#include <cuda_fp16.h>
#include <cuda_runtime.h>
#include <thrust/binary_search.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/gather.h>
#include <thrust/sequence.h>
#include <thrust/sort.h>
#include <thrust/transform.h>
#include <thrust/unique.h>

#include <cstdint>
#include <cstdio>

namespace autoware::lidar_frnet
{
__constant__ utils::FieldOfView const_fov;
__constant__ utils::Dims2d const_interpolation;
__constant__ utils::Dims2d const_frustum;
__constant__ bool const_crop_box_enabled;
__constant__ float const_crop_box_bounds[6];          // [min_x, min_y, min_z, max_x, max_y, max_z]
__constant__ float const_crop_box_sensor_to_ref[12];  // row-major 3x3 R then tx, ty, tz

PreprocessCuda::PreprocessCuda(const utils::NetworkParams & params, cudaStream_t stream)
: interpolation_(params.interpolation), stream_(stream)
{
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_fov, &params.fov, sizeof(utils::FieldOfView), 0, cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_interpolation, &params.interpolation, sizeof(utils::Dims2d), 0, cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_frustum, &params.frustum, sizeof(utils::Dims2d), 0, cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_crop_box_enabled, &params.crop_box_enabled, sizeof(bool), 0, cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_crop_box_bounds, params.crop_box_bounds.data(), sizeof(float) * 6, 0,
    cudaMemcpyHostToDevice));
  // Identity transform (used only when crop enabled; pipeline sets real transform each frame)
  const float identity[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_crop_box_sensor_to_ref, identity, sizeof(float) * 12, 0, cudaMemcpyHostToDevice));
}

void PreprocessCuda::setCropBoxTransform(const float * T_sensor_to_ref_12)
{
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_crop_box_sensor_to_ref, T_sensor_to_ref_12, sizeof(float) * 12, 0,
    cudaMemcpyHostToDevice));
}

struct alignas(8) half4
{
  half4() = default;
  __device__ half4(__half x, __half y, __half z, __half w) : x(x), y(y), z(z), w(w) {}

  __half x, y, z, w;
};

__device__ inline unsigned long long pack_half4(const half4 & h)
{
  unsigned long long packed = 0;
  auto p = reinterpret_cast<uint16_t *>(&packed);
  p[0] = __half_as_ushort(h.x);
  p[1] = __half_as_ushort(h.y);
  p[2] = __half_as_ushort(h.z);
  p[3] = __half_as_ushort(h.w);
  return packed;
}

__device__ inline void unpack_half4(unsigned long long packed, half4 & h)
{
  auto p = reinterpret_cast<uint16_t *>(&packed);
  h.x = __ushort_as_half(p[0]);
  h.y = __ushort_as_half(p[1]);
  h.z = __ushort_as_half(p[2]);
  h.w = __ushort_as_half(p[3]);
}

__device__ void atomicMaxByDepth_half4(unsigned long long * address, const half4 & new_val)
{
  half4 old_val;
  unsigned long long old_packed = *address;

  while (true) {
    unpack_half4(old_packed, old_val);

    float depth_old = __half2float(old_val.x) * __half2float(old_val.x) +
                      __half2float(old_val.y) * __half2float(old_val.y) +
                      __half2float(old_val.z) * __half2float(old_val.z);

    float depth_new = __half2float(new_val.x) * __half2float(new_val.x) +
                      __half2float(new_val.y) * __half2float(new_val.y) +
                      __half2float(new_val.z) * __half2float(new_val.z);

    if (depth_new <= depth_old) return;

    unsigned long long new_packed = pack_half4(new_val);
    unsigned long long prev = atomicCAS(address, old_packed, new_packed);

    if (prev == old_packed) break;

    old_packed = prev;  // retry
  }
}

__device__ int2 project2d(const float3 point, const uint32_t scale_x, const uint32_t scale_y)
{
  // Compute the depth
  float depth = sqrtf(point.x * point.x + point.y * point.y + point.z * point.z);

  // Get angles (yaw and pitch)
  float yaw = -atan2f(point.y, point.x);
  float pitch = asinf(point.z / depth);

  // Project to image coordinates (normalized)
  float proj_x = 0.5f * (yaw / M_PIf + 1.0f);
  float proj_y = 1.0f - (pitch + fabs(const_fov.down)) / const_fov.total;

  // Scale to image size using angular resolution
  proj_x *= static_cast<float>(scale_x);
  proj_y *= static_cast<float>(scale_y);

  // Return clamped and floor projection
  return make_int2(
    static_cast<int32_t>(fminf(fmaxf(proj_x, 0.0f), static_cast<float>(scale_x) - 1.0f)),
    static_cast<int32_t>(fminf(fmaxf(proj_y, 0.0f), static_cast<float>(scale_y) - 1.0f)));
}

/// @brief Trait to get intensity as float from different point types (CUDA device code)
template <typename PointT>
__device__ inline float get_intensity(const PointT & point);

template <>
__device__ inline float get_intensity(const InputPointTypeXYZI & point)
{
  return point.intensity;
}

template <>
__device__ inline float get_intensity(const InputPointTypeXYZIRC & point)
{
  return static_cast<float>(point.intensity);
}

template <>
__device__ inline float get_intensity(const InputPointTypeXYZIRADRT & point)
{
  return point.intensity;
}

template <>
__device__ inline float get_intensity(const InputPointTypeXYZIRCAEDT & point)
{
  return static_cast<float>(point.intensity);
}

template <typename PointT>
__global__ void projectPoints_kernel(
  const PointT * cloud, const uint32_t num_points, uint32_t * output_num_points,
  float * output_points, int64_t * output_coors, int64_t * output_coors_keys,
  uint32_t * output_proj_idxs, uint64_t * output_proj_2d, PointT * output_cloud_compact)
{
  uint32_t point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= num_points) return;

  const PointT & point = cloud[point_idx];
  const auto point3d = make_float3(point.x, point.y, point.z);

  // Ego crop box in reference frame: transform point to reference, then test AABB
  if (const_crop_box_enabled) {
    const float px = point3d.x, py = point3d.y, pz = point3d.z;
    const float p_ref_x = const_crop_box_sensor_to_ref[0] * px +
                          const_crop_box_sensor_to_ref[1] * py +
                          const_crop_box_sensor_to_ref[2] * pz + const_crop_box_sensor_to_ref[9];
    const float p_ref_y = const_crop_box_sensor_to_ref[3] * px +
                          const_crop_box_sensor_to_ref[4] * py +
                          const_crop_box_sensor_to_ref[5] * pz + const_crop_box_sensor_to_ref[10];
    const float p_ref_z = const_crop_box_sensor_to_ref[6] * px +
                          const_crop_box_sensor_to_ref[7] * py +
                          const_crop_box_sensor_to_ref[8] * pz + const_crop_box_sensor_to_ref[11];
    if (
      p_ref_x > const_crop_box_bounds[0] && p_ref_x < const_crop_box_bounds[3] &&
      p_ref_y > const_crop_box_bounds[1] && p_ref_y < const_crop_box_bounds[4] &&
      p_ref_z > const_crop_box_bounds[2] && p_ref_z < const_crop_box_bounds[5]) {
      return;
    }
  }

  const uint32_t append_idx = atomicAdd(output_num_points, 1);
  const float intensity = get_intensity(point);
  const auto proj_point = project2d(point3d, const_interpolation.w, const_interpolation.h);
  const auto proj_coor = project2d(point3d, const_frustum.w, const_frustum.h);
  const auto proj_idx = proj_point.y * const_interpolation.w + proj_point.x;

  output_points[append_idx * 4 + 0] = point.x;
  output_points[append_idx * 4 + 1] = point.y;
  output_points[append_idx * 4 + 2] = point.z;
  output_points[append_idx * 4 + 3] = intensity;

  output_coors[append_idx * 3 + 0] = 0;
  output_coors[append_idx * 3 + 1] = proj_coor.y;
  output_coors[append_idx * 3 + 2] = proj_coor.x;

  output_coors_keys[append_idx] = proj_coor.y * const_frustum.w + proj_coor.x;

  if (output_cloud_compact != nullptr) {
    output_cloud_compact[append_idx] = point;
  }

  // Update projection if not yet filled
  auto point_half4 = half4(point.x, point.y, point.z, intensity);
  auto output_proj_2d_address = reinterpret_cast<unsigned long long *>(&output_proj_2d[proj_idx]);
  if (!atomicExch(&output_proj_idxs[proj_idx], 1)) {
    *output_proj_2d_address = pack_half4(point_half4);
  } else {
    atomicMaxByDepth_half4(output_proj_2d_address, point_half4);
  }
}

template <typename PointT>
cudaError_t PreprocessCuda::projectPoints_launch_impl(
  const PointT * cloud, const uint32_t num_points, uint32_t * output_num_points,
  float * output_points, int64_t * output_coors, int64_t * output_coors_keys,
  uint32_t * output_proj_idxs, uint64_t * output_proj_2d, void * output_cloud_compact)
{
  dim3 block(utils::divup(num_points, utils::kernel_1d_size));
  dim3 threads(utils::kernel_1d_size);

  projectPoints_kernel<<<block, threads, 0, stream_>>>(
    cloud, num_points, output_num_points, output_points, output_coors, output_coors_keys,
    output_proj_idxs, output_proj_2d, static_cast<PointT *>(output_cloud_compact));

  return cudaGetLastError();
}

// Explicit instantiations
template cudaError_t PreprocessCuda::projectPoints_launch_impl<InputPointTypeXYZI>(
  const InputPointTypeXYZI *, const uint32_t, uint32_t *, float *, int64_t *, int64_t *, uint32_t *,
  uint64_t *, void *);
template cudaError_t PreprocessCuda::projectPoints_launch_impl<InputPointTypeXYZIRC>(
  const InputPointTypeXYZIRC *, const uint32_t, uint32_t *, float *, int64_t *, int64_t *,
  uint32_t *, uint64_t *, void *);
template cudaError_t PreprocessCuda::projectPoints_launch_impl<InputPointTypeXYZIRADRT>(
  const InputPointTypeXYZIRADRT *, const uint32_t, uint32_t *, float *, int64_t *, int64_t *,
  uint32_t *, uint64_t *, void *);
template cudaError_t PreprocessCuda::projectPoints_launch_impl<InputPointTypeXYZIRCAEDT>(
  const InputPointTypeXYZIRCAEDT *, const uint32_t, uint32_t *, float *, int64_t *, int64_t *,
  uint32_t *, uint64_t *, void *);

cudaError_t PreprocessCuda::projectPoints_launch(
  const void * cloud, const uint32_t num_points, InputFormat format, uint32_t * output_num_points,
  float * output_points, int64_t * output_coors, int64_t * output_coors_keys,
  uint32_t * output_proj_idxs, uint64_t * output_proj_2d, void * output_cloud_compact)
{
  switch (format) {
    case InputFormat::XYZIRCAEDT:
      return projectPoints_launch_impl(
        static_cast<const InputPointTypeXYZIRCAEDT *>(cloud), num_points, output_num_points,
        output_points, output_coors, output_coors_keys, output_proj_idxs, output_proj_2d,
        output_cloud_compact);
    case InputFormat::XYZIRADRT:
      return projectPoints_launch_impl(
        static_cast<const InputPointTypeXYZIRADRT *>(cloud), num_points, output_num_points,
        output_points, output_coors, output_coors_keys, output_proj_idxs, output_proj_2d,
        output_cloud_compact);
    case InputFormat::XYZIRC:
      return projectPoints_launch_impl(
        static_cast<const InputPointTypeXYZIRC *>(cloud), num_points, output_num_points,
        output_points, output_coors, output_coors_keys, output_proj_idxs, output_proj_2d,
        output_cloud_compact);
    case InputFormat::XYZI:
      return projectPoints_launch_impl(
        static_cast<const InputPointTypeXYZI *>(cloud), num_points, output_num_points,
        output_points, output_coors, output_coors_keys, output_proj_idxs, output_proj_2d,
        output_cloud_compact);
    default:
      return cudaErrorInvalidValue;
  }
}

__global__ void interpolatePoints_kernel(
  const uint32_t * proj_idxs, uint64_t * proj_2d, uint32_t * output_num_points,
  float * output_points, int64_t * output_coors, int64_t * output_coors_keys)
{
  uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
  uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= const_interpolation.w || y >= const_interpolation.h) return;

  uint32_t idx = y * const_interpolation.w + x;

  if (proj_idxs[idx]) {
    return;  // Skip if the pixel is already filled
  }

  // Look for neighboring pixels (left and right)
  if (x > 0 && x < const_interpolation.w - 1) {
    if (
      proj_idxs[(y * const_interpolation.w) + (x - 1)] &&
      proj_idxs[(y * const_interpolation.w) + (x + 1)]) {
      // Interpolate points
      half4 left, right;
      unpack_half4(proj_2d[idx - 1], left);
      unpack_half4(proj_2d[idx + 1], right);

      // Compute the mean of the valid neighbors
      const auto interpolated_point = make_float4(
        (__half2float(left.x + right.x)) * 0.5f, (__half2float(left.y + right.y)) * 0.5f,
        (__half2float(left.z + right.z)) * 0.5f, (__half2float(left.w + right.w)) * 0.5f);

      auto append_idx = atomicAdd(output_num_points, 1);
      output_points[append_idx * 4 + 0] = interpolated_point.x;
      output_points[append_idx * 4 + 1] = interpolated_point.y;
      output_points[append_idx * 4 + 2] = interpolated_point.z;
      output_points[append_idx * 4 + 3] = interpolated_point.w;

      const auto proj_coor = project2d(
        make_float3(interpolated_point.x, interpolated_point.y, interpolated_point.z),
        const_frustum.w, const_frustum.h);

      output_coors[append_idx * 3 + 0] = 0;
      output_coors[append_idx * 3 + 1] = proj_coor.y;
      output_coors[append_idx * 3 + 2] = proj_coor.x;

      output_coors_keys[append_idx] = proj_coor.y * const_frustum.w + proj_coor.x;
    }
  }
}

cudaError_t PreprocessCuda::interpolatePoints_launch(
  uint32_t * proj_idxs, uint64_t * proj_2d, uint32_t * output_num_points, float * output_points,
  int64_t * output_coors, int64_t * output_coors_keys)
{
  dim3 block(
    utils::divup(interpolation_.w, utils::kernel_2d_size),
    utils::divup(interpolation_.h, utils::kernel_2d_size));
  dim3 threads(utils::kernel_2d_size, utils::kernel_2d_size);

  interpolatePoints_kernel<<<block, threads, 0, stream_>>>(
    proj_idxs, proj_2d, output_num_points, output_points, output_coors, output_coors_keys);

  return cudaGetLastError();
}

void PreprocessCuda::generateUniqueCoors(
  const uint32_t num_points, const int64_t * coors, const int64_t * coors_keys,
  uint32_t & output_num_unique_coors, int64_t * output_voxel_coors, int64_t * output_inverse_map)
{
  // Input unordered coors
  thrust::device_vector<Coord> coors_d(
    reinterpret_cast<const Coord *>(coors), reinterpret_cast<const Coord *>(coors) + num_points);

  // Original indices of coors
  thrust::device_vector<int64_t> sorted_idxs(num_points);
  thrust::sequence(thrust::cuda::par.on(stream_), sorted_idxs.begin(), sorted_idxs.end(), 0);

  // Sort indices by coors keys
  thrust::device_vector<int64_t> coors_keys_d(num_points);
  int64_t * d_sorted_keys_output_ptr = thrust::raw_pointer_cast(coors_keys_d.data());
  int64_t * d_values_inout_ptr = thrust::raw_pointer_cast(sorted_idxs.data());
  void * d_temp_storage = nullptr;
  size_t temp_storage_bytes = 0;

  // Memory requirements calculation
  // Note: If num_points is 0, CUB's SortPairs is a no-op and temp_storage_bytes will be 0
  cub::DeviceRadixSort::SortPairs(
    d_temp_storage, temp_storage_bytes, coors_keys, d_sorted_keys_output_ptr, d_values_inout_ptr,
    d_values_inout_ptr, num_points, 0, sizeof(int64_t) * 8, stream_);

  // Allocate temporary storage
  thrust::device_vector<u_char> cub_temp_storage_buffer_d(temp_storage_bytes);
  if (temp_storage_bytes > 0) {  // Only assign if memory is actually needed
    d_temp_storage = thrust::raw_pointer_cast(cub_temp_storage_buffer_d.data());
  } else {
    d_temp_storage = nullptr;  // Ensure d_temp_storage is nullptr if no bytes are needed
  }

  // Perform the sort if there are points to sort
  if (num_points > 0) {
    cub::DeviceRadixSort::SortPairs(
      d_temp_storage, temp_storage_bytes, coors_keys, d_sorted_keys_output_ptr, d_values_inout_ptr,
      d_values_inout_ptr, num_points, 0, sizeof(int64_t) * 8, stream_);
  }

  // Reorder coors based on the sorted indices
  thrust::device_vector<Coord> coors_sorted(num_points);
  thrust::gather(
    thrust::cuda::par.on(stream_), sorted_idxs.begin(), sorted_idxs.end(), coors_d.begin(),
    coors_sorted.begin());

  // Generate inverse_map for the sorted coors
  thrust::device_vector<int64_t> idxs_update(num_points);

  auto not_equal = [] __device__(const Coord & a, const Coord & b) { return a == b ? 0 : 1; };
  thrust::transform(
    thrust::cuda::par.on(stream_), coors_sorted.begin(), coors_sorted.end() - 1,
    coors_sorted.begin() + 1, idxs_update.begin() + 1, not_equal);

  thrust::device_vector<int64_t> labels(num_points);
  thrust::inclusive_scan(
    thrust::cuda::par.on(stream_), idxs_update.begin(), idxs_update.end(), labels.begin());

  thrust::scatter(
    thrust::cuda::par.on(stream_), labels.begin(), labels.end(), sorted_idxs.begin(),
    output_inverse_map);

  // Generate unique coors
  auto new_end = thrust::unique_copy(
    thrust::cuda::par.on(stream_), coors_sorted.begin(), coors_sorted.end(),
    reinterpret_cast<Coord *>(output_voxel_coors));
  output_num_unique_coors = new_end - reinterpret_cast<Coord *>(output_voxel_coors);
}

}  // namespace autoware::lidar_frnet
