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

#ifndef AUTOWARE__PTV3__EXPERIMENTAL__POINT_TYPE_HPP_
#define AUTOWARE__PTV3__EXPERIMENTAL__POINT_TYPE_HPP_

#include <autoware/point_types/types.hpp>
#include <point_cloud_msg_wrapper/default_field_generators.hpp>
#include <point_cloud_msg_wrapper/field_generators.hpp>

#include <sensor_msgs/msg/point_field.hpp>

#include <pcl/register_point_struct.h>

#include <cstddef>
#include <cstdint>
#include <tuple>
#include <vector>

namespace autoware::ptv3::experimental
{
/**
 * @brief PointXYZCPEIndex is an enum class representing the index of a field in the PointXYZCPE
 * struct.
 */
enum class PointXYZCPEIndex { X, Y, Z, CLASS_ID, PROBABILITY, ENTROPY };

/**
 * @brief PointXYZCPE is a point type with x, y, z coordinates, class id, probability, and entropy.
 */
struct PointXYZCPE
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  std::uint8_t class_id{0U};
  float probability{0.0F};
  float entropy{0.0F};

  friend bool operator==(const PointXYZCPE & p1, const PointXYZCPE & p2)
  {
    return autoware::point_types::float_eq<float>(p1.x, p2.x) &&
           autoware::point_types::float_eq<float>(p1.y, p2.y) &&
           autoware::point_types::float_eq<float>(p1.z, p2.z) && p1.class_id == p2.class_id &&
           autoware::point_types::float_eq<float>(p1.probability, p2.probability) &&
           autoware::point_types::float_eq<float>(p1.entropy, p2.entropy);
  }
};

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(class_id);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(probability);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(entropy);

using PointXYZCPEFieldGenerator = std::tuple<
  point_cloud_msg_wrapper::field_x_generator, point_cloud_msg_wrapper::field_y_generator,
  point_cloud_msg_wrapper::field_z_generator, field_class_id_generator, field_probability_generator,
  field_entropy_generator>;

/// ============================================================
/// Memory layout compatibility check for PointXYZCPE.
/// ============================================================

inline bool is_data_layout_compatible_with_point_xyzcpe(
  const std::vector<sensor_msgs::msg::PointField> & fields)
{
  using PointIndex = autoware::ptv3::experimental::PointXYZCPEIndex;
  using PointType = autoware::ptv3::experimental::PointXYZCPE;

  constexpr std::size_t num_fields = 6;
  if (fields.size() != num_fields) {
    return false;
  }

  bool same_layout = true;
  const auto & field_x = fields.at(static_cast<std::size_t>(PointIndex::X));
  same_layout &= field_x.name == "x";
  same_layout &= field_x.offset == offsetof(PointType, x);
  same_layout &= field_x.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_x.count == 1;
  const auto & field_y = fields.at(static_cast<std::size_t>(PointIndex::Y));
  same_layout &= field_y.name == "y";
  same_layout &= field_y.offset == offsetof(PointType, y);
  same_layout &= field_y.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_y.count == 1;
  const auto & field_z = fields.at(static_cast<std::size_t>(PointIndex::Z));
  same_layout &= field_z.name == "z";
  same_layout &= field_z.offset == offsetof(PointType, z);
  same_layout &= field_z.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_z.count == 1;
  const auto & field_class_id = fields.at(static_cast<std::size_t>(PointIndex::CLASS_ID));
  same_layout &= field_class_id.name == "class_id";
  same_layout &= field_class_id.offset == offsetof(PointType, class_id);
  same_layout &= field_class_id.datatype == sensor_msgs::msg::PointField::UINT8;
  same_layout &= field_class_id.count == 1;
  const auto & field_probability = fields.at(static_cast<std::size_t>(PointIndex::PROBABILITY));
  same_layout &= field_probability.name == "probability";
  same_layout &= field_probability.offset == offsetof(PointType, probability);
  same_layout &= field_probability.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_probability.count == 1;
  const auto & field_entropy = fields.at(static_cast<std::size_t>(PointIndex::ENTROPY));
  same_layout &= field_entropy.name == "entropy";
  same_layout &= field_entropy.offset == offsetof(PointType, entropy);
  same_layout &= field_entropy.datatype == sensor_msgs::msg::PointField::FLOAT32;
  same_layout &= field_entropy.count == 1;

  return same_layout;
}

inline bool is_data_layout_compatible_with_point_xyzcpe(const sensor_msgs::msg::PointCloud2 & input)
{
  return is_data_layout_compatible_with_point_xyzcpe(input.fields);
}
}  // namespace autoware::ptv3::experimental

POINT_CLOUD_REGISTER_POINT_STRUCT(
  autoware::ptv3::experimental::PointXYZCPE,
  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, class_id, class_id)(
    float, probability, probability)(float, entropy, entropy))
#endif  // AUTOWARE__PTV3__EXPERIMENTAL__POINT_TYPE_HPP_
