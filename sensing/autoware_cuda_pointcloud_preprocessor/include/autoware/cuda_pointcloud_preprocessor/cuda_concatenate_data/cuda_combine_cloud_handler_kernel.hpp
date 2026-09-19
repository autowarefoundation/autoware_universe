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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CONCATENATE_DATA__CUDA_COMBINE_CLOUD_HANDLER_KERNEL_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CONCATENATE_DATA__CUDA_COMBINE_CLOUD_HANDLER_KERNEL_HPP_

#include <autoware/point_types/types.hpp>

#include <cuda_runtime.h>

#include <cstdint>

namespace autoware::pointcloud_preprocessor
{

struct TransformStruct
{
  float translation_x;
  float translation_y;
  float translation_z;
  float m11;
  float m12;
  float m13;
  float m21;
  float m22;
  float m23;
  float m31;
  float m32;
  float m33;
};

/// Applies `transform` to `input_points` and writes the result to `output_points`. PointT is
/// autoware::point_types::PointXYZIRC or PointXYZIRCT; for the latter, `time_offset_ns` is added
/// to every point's time_stamp.
template <typename PointT>
void transform_launch(
  const PointT * input_points, int num_points, TransformStruct transform,
  std::uint32_t time_offset_ns, PointT * output_points, cudaStream_t & stream);

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CONCATENATE_DATA__CUDA_COMBINE_CLOUD_HANDLER_KERNEL_HPP_
