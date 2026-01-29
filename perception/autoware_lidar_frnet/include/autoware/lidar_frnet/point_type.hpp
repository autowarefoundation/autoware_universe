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

#ifndef AUTOWARE__LIDAR_FRNET__POINT_TYPE_HPP_
#define AUTOWARE__LIDAR_FRNET__POINT_TYPE_HPP_

#include <cstddef>
#include <cstdint>

namespace autoware::lidar_frnet
{

/// @brief Enum representing supported input point cloud formats
enum class InputFormat { XYZIRCAEDT, XYZIRADRT, XYZIRC, XYZI, UNKNOWN };

/// @brief Input point type for XYZI format (x, y, z, intensity as float)
struct InputPointTypeXYZI
{
  float x;
  float y;
  float z;
  float intensity;
} __attribute__((packed));

/// @brief Input point type for XYZIRC format (x, y, z, intensity, return_type, channel)
struct InputPointTypeXYZIRC
{
  float x;
  float y;
  float z;
  std::uint8_t intensity;
  std::uint8_t return_type;
  std::uint16_t channel;
} __attribute__((packed));

/// @brief Input point type for XYZIRADRT format
struct InputPointTypeXYZIRADRT
{
  float x;
  float y;
  float z;
  float intensity;
  std::uint16_t ring;
  float azimuth;
  float distance;
  std::uint8_t return_type;
  double time_stamp;
} __attribute__((packed));

/// @brief Input point type for XYZIRCAEDT format
struct InputPointTypeXYZIRCAEDT
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
} __attribute__((packed));

/// @brief Get point step size for a given input format
inline std::size_t get_point_step(InputFormat format)
{
  switch (format) {
    case InputFormat::XYZIRCAEDT:
      return sizeof(InputPointTypeXYZIRCAEDT);
    case InputFormat::XYZIRADRT:
      return sizeof(InputPointTypeXYZIRADRT);
    case InputFormat::XYZIRC:
      return sizeof(InputPointTypeXYZIRC);
    case InputFormat::XYZI:
      return sizeof(InputPointTypeXYZI);
    default:
      return 0;
  }
}

/// @brief Get the number of fields for a given input format
inline std::size_t get_num_fields(InputFormat format)
{
  switch (format) {
    case InputFormat::XYZIRCAEDT:
      return 10;
    case InputFormat::XYZIRADRT:
      return 9;
    case InputFormat::XYZIRC:
      return 6;
    case InputFormat::XYZI:
      return 4;
    default:
      return 0;
  }
}

struct OutputSegmentationPointType
{
  float x;
  float y;
  float z;
  std::uint8_t class_id;
} __attribute__((packed));

struct OutputVisualizationPointType
{
  float x;
  float y;
  float z;
  float rgb;
} __attribute__((packed));

}  // namespace autoware::lidar_frnet

#endif  // AUTOWARE__LIDAR_FRNET__POINT_TYPE_HPP_
