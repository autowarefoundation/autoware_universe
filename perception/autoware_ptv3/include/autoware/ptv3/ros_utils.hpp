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

#ifndef AUTOWARE__PTV3__ROS_UTILS_HPP_
#define AUTOWARE__PTV3__ROS_UTILS_HPP_

#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/utils.hpp"

#include <autoware/point_types/types.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>

#include <cstdint>
#include <string>
#include <vector>

#define CHECK_OFFSET(structure1, structure2, field)             \
  static_assert(                                                \
    offsetof(structure1, field) == offsetof(structure2, field), \
    "Offset of " #field " in " #structure1 " does not match the one in " #structure2 ".")
#define CHECK_TYPE(structure1, structure2, field)                             \
  static_assert(                                                              \
    std::is_same_v<decltype(structure1::field), decltype(structure2::field)>, \
    "Type of " #field " in " #structure1 " and " #structure1 " have different types.")
#define CHECK_FIELD(structure1, structure2, field) \
  CHECK_OFFSET(structure1, structure2, field);     \
  CHECK_TYPE(structure1, structure2, field)

namespace autoware::ptv3
{
using sensor_msgs::msg::PointField;

CHECK_FIELD(CloudPointType, autoware::point_types::PointXYZIRC, x);
CHECK_FIELD(CloudPointType, autoware::point_types::PointXYZIRC, y);
CHECK_FIELD(CloudPointType, autoware::point_types::PointXYZIRC, z);
CHECK_FIELD(CloudPointType, autoware::point_types::PointXYZIRC, intensity);
static_assert(sizeof(CloudPointType) == sizeof(autoware::point_types::PointXYZIRC));

/**
 * @brief Convert a decoded detection box into a DetectedObject message.
 *
 * @param box3d Decoded detection box.
 * @param class_names Detection class names indexed by box label.
 * @param has_twist Whether to populate the object twist from box velocity.
 * @param obj Output object to fill.
 */
void box3d_to_detected_object(
  const Box3D & box3d, const std::vector<std::string> & class_names, bool has_twist,
  autoware_perception_msgs::msg::DetectedObject & obj);

/**
 * @brief Map a detection class name to an ObjectClassification label.
 *
 * @param class_name Detection class name.
 * @return Matching ObjectClassification label, or UNKNOWN.
 */
std::uint8_t get_semantic_type(const std::string & class_name);

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__ROS_UTILS_HPP_
