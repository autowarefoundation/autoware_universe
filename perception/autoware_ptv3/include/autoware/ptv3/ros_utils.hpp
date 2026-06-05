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

#include "autoware/ptv3/utils.hpp"

#include "autoware_perception_msgs/msg/detected_objects.hpp"

#include <string>
#include <vector>

namespace autoware::ptv3
{

void box3d_to_detected_object(
  const Box3D & box3d, const std::vector<std::string> & class_names, bool has_twist,
  autoware_perception_msgs::msg::DetectedObject & obj);

uint8_t get_semantic_type(const std::string & class_name);

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__ROS_UTILS_HPP_
