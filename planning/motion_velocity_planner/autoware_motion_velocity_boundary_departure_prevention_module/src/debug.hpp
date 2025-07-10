// Copyright 2024 TIER IV, Inc.
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

#ifndef DEBUG_HPP_
#define DEBUG_HPP_

#include "parameters.hpp"

namespace autoware::motion_velocity_planner::experimental::debug
{

Marker create_departure_points_marker(
  const DeparturePoints & departure_points, const rclcpp::Time & curr_time,
  const double base_link_z);

MarkerArray create_debug_marker_array(
  const Output & output, const Trajectory & ego_traj, const rclcpp::Clock::SharedPtr & clock_ptr,
  const double base_link_z, const NodeParam & node_param);
}  // namespace autoware::motion_velocity_planner::experimental::debug

#endif  // DEBUG_HPP_
