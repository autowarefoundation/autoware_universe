// Copyright 2024 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE__OBSTACLE_COLLISION_CHECKER__DEBUG_HPP_
#define AUTOWARE__OBSTACLE_COLLISION_CHECKER__DEBUG_HPP_

#include "autoware/obstacle_collision_checker/obstacle_collision_checker.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

namespace autoware::obstacle_collision_checker
{
/// @brief create debug markers of the given output
/// @param output structure with output data calculated by the obstacle_collision_checker module
/// @param base_link_z current z value of the base_link in map frame
/// @param now current time
/// @return debug markers
visualization_msgs::msg::MarkerArray create_marker_array(
  const Output & output, const double base_link_z, const rclcpp::Time & now);
}  // namespace autoware::obstacle_collision_checker
#endif  // AUTOWARE__OBSTACLE_COLLISION_CHECKER__DEBUG_HPP_
