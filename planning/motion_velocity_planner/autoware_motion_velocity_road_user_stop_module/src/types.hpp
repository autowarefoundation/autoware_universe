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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include "type_alias.hpp"

#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <algorithm>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{
struct StopPointCandidate
{
  size_t stop_index;
  geometry_msgs::msg::Point stop_position;
  double stop_distance;  // distance from ego vehicle
  autoware_perception_msgs::msg::PredictedObject target_object;
  bool is_wrong_way;
  double required_deceleration;
};

struct TrackedObject
{
  unique_identifier_msgs::msg::UUID object_id;
  rclcpp::Time first_detection_time;
  rclcpp::Time last_detection_time;
  geometry_msgs::msg::Point last_position;
  std::optional<StopPointCandidate>
    wrong_way_stop_candidate;  // cached stop point for wrong-way users
};
}  // namespace autoware::motion_velocity_planner

#endif  // TYPES_HPP_
