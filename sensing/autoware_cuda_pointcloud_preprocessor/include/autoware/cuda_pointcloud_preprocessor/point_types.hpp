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
#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__POINT_TYPES_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__POINT_TYPES_HPP_

#include <autoware/point_types/types.hpp>

#include <cstdint>

#define FIELD_IS_EQUAL(type1, type2, field) \
  (offsetof(type1, field) == offsetof(type2, field) && sizeof(type1::field) == sizeof(type2::field))

namespace autoware::cuda_pointcloud_preprocessor
{

// Note: We can not use PCL nor uniform initialization here because of thrust
#pragma pack(push, 1)
struct OutputPointType
{
  float x;
  float y;
  float z;
  std::uint8_t intensity;
  std::uint8_t return_type;
  std::uint16_t channel;
};

struct InputPointType
{
  float x;
  float y;
  float z;
  std::uint8_t intensity;
  std::uint8_t return_type;
  std::uint16_t channel;
  float azimuth;
  float elevation;
  float distance;
  std::uint32_t time_stamp;
};
#pragma pack(pop)

static_assert(sizeof(OutputPointType) == sizeof(point_types::PointXYZIRC));
static_assert(FIELD_IS_EQUAL(OutputPointType, point_types::PointXYZIRC, x));
static_assert(FIELD_IS_EQUAL(OutputPointType, point_types::PointXYZIRC, y));
static_assert(FIELD_IS_EQUAL(OutputPointType, point_types::PointXYZIRC, z));
static_assert(FIELD_IS_EQUAL(OutputPointType, point_types::PointXYZIRC, intensity));
static_assert(FIELD_IS_EQUAL(OutputPointType, point_types::PointXYZIRC, return_type));
static_assert(FIELD_IS_EQUAL(OutputPointType, point_types::PointXYZIRC, channel));

static_assert(sizeof(InputPointType) == sizeof(point_types::PointXYZIRCAEDT));
static_assert(FIELD_IS_EQUAL(InputPointType, point_types::PointXYZIRCAEDT, x));
static_assert(FIELD_IS_EQUAL(InputPointType, point_types::PointXYZIRCAEDT, y));
static_assert(FIELD_IS_EQUAL(InputPointType, point_types::PointXYZIRCAEDT, z));
static_assert(FIELD_IS_EQUAL(InputPointType, point_types::PointXYZIRCAEDT, intensity));
static_assert(FIELD_IS_EQUAL(InputPointType, point_types::PointXYZIRCAEDT, return_type));
static_assert(FIELD_IS_EQUAL(InputPointType, point_types::PointXYZIRCAEDT, channel));
static_assert(FIELD_IS_EQUAL(InputPointType, point_types::PointXYZIRCAEDT, azimuth));
static_assert(FIELD_IS_EQUAL(InputPointType, point_types::PointXYZIRCAEDT, elevation));
static_assert(FIELD_IS_EQUAL(InputPointType, point_types::PointXYZIRCAEDT, distance));
static_assert(FIELD_IS_EQUAL(InputPointType, point_types::PointXYZIRCAEDT, time_stamp));

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__POINT_TYPES_HPP_
