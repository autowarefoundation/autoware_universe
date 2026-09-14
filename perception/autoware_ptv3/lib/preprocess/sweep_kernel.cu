// Copyright 2026 TIER IV, Inc.
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
#include "autoware/ptv3/preprocess/sweep_kernel.hpp"
#include "autoware/ptv3/utils.hpp"

#include <stdexcept>

namespace autoware::ptv3
{
namespace
{

// Intensity normalization must match training: the 8-bit intensity formats are
// scaled to [0, 1]; the float intensity formats are consumed raw.
__device__ float normalizedIntensity(const CloudPointTypeXYZIRCAEDT & point)
{
  return static_cast<float>(point.intensity) / 255.f;
}

__device__ float normalizedIntensity(const CloudPointTypeXYZIRC & point)
{
  return static_cast<float>(point.intensity) / 255.f;
}

__device__ float normalizedIntensity(const CloudPointTypeXYZIRADRT & point)
{
  return point.intensity;
}

__device__ float normalizedIntensity(const CloudPointTypeXYZI & point)
{
  return point.intensity;
}

template <typename PointT>
__global__ void generateSweepFeaturesKernel(
  const PointT * __restrict__ input_points, std::size_t points_size, float time_lag,
  float close_radius, const float * __restrict__ transform, std::int64_t num_features,
  float * __restrict__ output_points)
{
  const auto idx = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (idx >= points_size) {
    return;
  }

  const PointT & input_point = input_points[idx];
  const float x = input_point.x;
  const float y = input_point.y;
  const float z = input_point.z;

  float * output_point = &output_points[idx * num_features];

  // Ego ghosts are the recording vehicle's own returns, boxed around the sweep's origin
  // before the motion compensation moves it, matching the training-side remove_close.
  // NaN coordinates make the range crop drop them while every frame keeps its row count,
  // so the current frame stays the leading block.
  const bool ego_ghost = time_lag != 0.f && fabsf(x) < close_radius && fabsf(y) < close_radius;
  if (ego_ghost) {
    output_point[0] = nanf("");
    output_point[1] = nanf("");
    output_point[2] = nanf("");
    output_point[3] = 0.f;
    output_point[4] = time_lag;
    return;
  }
  output_point[0] = transform[0] * x + transform[4] * y + transform[8] * z + transform[12];
  output_point[1] = transform[1] * x + transform[5] * y + transform[9] * z + transform[13];
  output_point[2] = transform[2] * x + transform[6] * y + transform[10] * z + transform[14];
  output_point[3] = normalizedIntensity(input_point);
  output_point[4] = time_lag;
}

}  // namespace

void generateSweepFeaturesLaunch(
  const void * input_data, CloudFormat input_format, std::size_t num_points, float time_lag,
  float close_radius, const float * transform_d, std::int64_t num_features, float * output_points,
  std::uint32_t threads_per_block, cudaStream_t stream)
{
  // A zero-sized grid is an invalid CUDA launch; an empty frame contributes nothing.
  if (num_points == 0) {
    return;
  }
  const auto num_blocks = divup(num_points, threads_per_block);
  switch (input_format) {
    case CloudFormat::XYZIRCAEDT:
      generateSweepFeaturesKernel<<<num_blocks, threads_per_block, 0, stream>>>(
        static_cast<const CloudPointTypeXYZIRCAEDT *>(input_data), num_points, time_lag,
        close_radius, transform_d, num_features, output_points);
      break;
    case CloudFormat::XYZIRADRT:
      generateSweepFeaturesKernel<<<num_blocks, threads_per_block, 0, stream>>>(
        static_cast<const CloudPointTypeXYZIRADRT *>(input_data), num_points, time_lag,
        close_radius, transform_d, num_features, output_points);
      break;
    case CloudFormat::XYZIRC:
      generateSweepFeaturesKernel<<<num_blocks, threads_per_block, 0, stream>>>(
        static_cast<const CloudPointTypeXYZIRC *>(input_data), num_points, time_lag, close_radius,
        transform_d, num_features, output_points);
      break;
    case CloudFormat::XYZI:
      generateSweepFeaturesKernel<<<num_blocks, threads_per_block, 0, stream>>>(
        static_cast<const CloudPointTypeXYZI *>(input_data), num_points, time_lag, close_radius,
        transform_d, num_features, output_points);
      break;
    default:
      throw std::runtime_error("Unsupported input point cloud format.");
  }
}

}  // namespace autoware::ptv3
