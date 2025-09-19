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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__MEMORY_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__MEMORY_HPP_

#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{

// TODO(knzo25): this checks already exists within the pointcloud_preprocessor. After this PR is
// merged I will move it somewhere all classes that require checks can access it
[[nodiscard]] inline bool is_data_layout_compatible_with_point_xyzircaedt(
  const std::vector<sensor_msgs::msg::PointField> & fields)
{
  if (fields.size() != 10) {
    return false;
  }
  bool same_layout = true;
  const auto & field_x = fields.at(0);
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(InputPointType, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = fields.at(1);
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(InputPointType, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = fields.at(2);
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(InputPointType, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_intensity = fields.at(3);
  same_layout &= field_intensity.name == "intensity";
  same_layout &= field_intensity.offset == offsetof(InputPointType, intensity);
  same_layout &= field_intensity.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_intensity.count == 1;
  const auto & field_return_type = fields.at(4);
  same_layout &= field_return_type.name == "return_type";
  same_layout &= field_return_type.offset == offsetof(InputPointType, return_type);
  same_layout &= field_return_type.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_return_type.count == 1;
  const auto & field_ring = fields.at(5);
  same_layout &= field_ring.name == "channel";
  same_layout &= field_ring.offset == offsetof(InputPointType, channel);
  same_layout &= field_ring.datatype == sensor_msgs::msg::PointField::UINT16;
  same_layout &= field_ring.count == 1;
  const auto & field_azimuth = fields.at(6);
  same_layout &= field_azimuth.name == "azimuth";
  same_layout &= field_azimuth.offset == offsetof(InputPointType, azimuth);
  same_layout &= field_azimuth.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_azimuth.count == 1;
  const auto & field_elevation = fields.at(7);
  same_layout &= field_elevation.name == "elevation";
  same_layout &= field_elevation.offset == offsetof(InputPointType, elevation);
  same_layout &= field_elevation.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_elevation.count == 1;
  const auto & field_distance = fields.at(8);
  same_layout &= field_distance.name == "distance";
  same_layout &= field_distance.offset == offsetof(InputPointType, distance);
  same_layout &= field_distance.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_distance.count == 1;
  const auto & field_time_stamp = fields.at(9);
  same_layout &= field_time_stamp.name == "time_stamp";
  same_layout &= field_time_stamp.offset == offsetof(InputPointType, time_stamp);
  same_layout &= field_time_stamp.datatype == sensor_msgs::msg::PointField::UINT32;
  same_layout &= field_time_stamp.count == 1;
  return same_layout;
}
}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__MEMORY_HPP_
