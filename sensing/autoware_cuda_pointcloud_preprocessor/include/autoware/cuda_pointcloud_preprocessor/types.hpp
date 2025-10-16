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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__TYPES_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__TYPES_HPP_

#include <cmath>
#include <cstdint>

namespace autoware::cuda_pointcloud_preprocessor
{

struct TwistStruct2D
{
  float cum_x;
  float cum_y;
  float cum_theta;
  float cum_cos_theta;
  float cum_sin_theta;
  std::uint32_t last_stamp_nsec;  // relative to the start of the pointcloud
  std::uint32_t stamp_nsec;       // relative to the start of the pointcloud
  float v_x;
  float v_theta;
};

struct TwistStruct3D
{
  float cum_transform_buffer[16];
  std::uint32_t last_stamp_nsec;  // relative to the start of the pointcloud
  std::uint32_t stamp_nsec;       // relative to the start of the pointcloud
  float v[3];
  float w[3];
};

struct TransformStruct
{
  float x;
  float y;
  float z;
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

struct CropBoxParameters
{
  float min_x;
  float max_x;
  float min_y;
  float max_y;
  float min_z;
  float max_z;
};

struct RingOutlierFilterParameters
{
  float distance_ratio{std::nanf("")};
  float object_length_threshold{std::nanf("")};
};
}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__TYPES_HPP_
