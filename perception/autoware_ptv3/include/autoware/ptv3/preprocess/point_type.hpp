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

#ifndef AUTOWARE__PTV3__PREPROCESS__POINT_TYPE_HPP_
#define AUTOWARE__PTV3__PREPROCESS__POINT_TYPE_HPP_

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <string>

namespace autoware::ptv3
{

enum class CloudFormat { XYZIRCAEDT, XYZIRADRT, XYZIRC, XYZI, UNKNOWN };

inline CloudFormat parse_cloud_format_string(std::string format)
{
  for (auto & ch : format) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }

  if (format == "xyzircaedt") {
    return CloudFormat::XYZIRCAEDT;
  }
  if (format == "xyziradrt") {
    return CloudFormat::XYZIRADRT;
  }
  if (format == "xyzirc") {
    return CloudFormat::XYZIRC;
  }
  if (format == "xyzi") {
    return CloudFormat::XYZI;
  }
  return CloudFormat::UNKNOWN;
}

inline const char * to_string(CloudFormat format)
{
  switch (format) {
    case CloudFormat::XYZIRCAEDT:
      return "xyzircaedt";
    case CloudFormat::XYZIRADRT:
      return "xyziradrt";
    case CloudFormat::XYZIRC:
      return "xyzirc";
    case CloudFormat::XYZI:
      return "xyzi";
    default:
      return "unknown";
  }
}

inline bool can_convert_format(const CloudFormat input_format, const CloudFormat output_format)
{
  switch (input_format) {
    case CloudFormat::XYZIRCAEDT:
      return output_format == CloudFormat::XYZIRCAEDT || output_format == CloudFormat::XYZIRC ||
             output_format == CloudFormat::XYZI;
    case CloudFormat::XYZIRADRT:
      return output_format == CloudFormat::XYZIRADRT || output_format == CloudFormat::XYZI;
    case CloudFormat::XYZIRC:
      return output_format == CloudFormat::XYZIRC || output_format == CloudFormat::XYZI;
    case CloudFormat::XYZI:
      return output_format == CloudFormat::XYZI;
    default:
      return false;
  }
}

struct CloudPointTypeXYZI
{
  float x;
  float y;
  float z;
  float intensity;
} __attribute__((packed));

struct CloudPointType
{
  float x;
  float y;
  float z;
  std::uint8_t intensity;
  std::uint8_t return_type;
  std::uint16_t channel;
} __attribute__((packed));

using CloudPointTypeXYZIRC = CloudPointType;

struct CloudPointTypeXYZIRADRT
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

struct CloudPointTypeXYZIRCAEDT
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

inline std::size_t get_point_step(CloudFormat format)
{
  switch (format) {
    case CloudFormat::XYZIRCAEDT:
      return sizeof(CloudPointTypeXYZIRCAEDT);
    case CloudFormat::XYZIRADRT:
      return sizeof(CloudPointTypeXYZIRADRT);
    case CloudFormat::XYZIRC:
      return sizeof(CloudPointTypeXYZIRC);
    case CloudFormat::XYZI:
      return sizeof(CloudPointTypeXYZI);
    default:
      return 0;
  }
}

inline std::size_t get_num_fields(CloudFormat format)
{
  switch (format) {
    case CloudFormat::XYZIRCAEDT:
      return 10;
    case CloudFormat::XYZIRADRT:
      return 9;
    case CloudFormat::XYZIRC:
      return 6;
    case CloudFormat::XYZI:
      return 4;
    default:
      return 0;
  }
}

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PREPROCESS__POINT_TYPE_HPP_
