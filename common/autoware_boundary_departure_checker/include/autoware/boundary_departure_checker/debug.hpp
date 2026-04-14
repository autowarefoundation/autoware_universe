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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DEBUG_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DEBUG_HPP_

#include "autoware/boundary_departure_checker/data_structs.hpp"
#include "autoware/boundary_departure_checker/type_alias.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

namespace autoware::boundary_departure_checker::debug
{
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

/**
 * @brief Create debug markers for boundary departure.
 * @param[in] departure_data data about the departure check
 * @param[in] curr_time current time
 * @param[in] base_link_z z-coordinate of the base_link
 * @return array of markers for visualization
 */
MarkerArray create_debug_markers(
  const DepartureData & departure_data, const rclcpp::Time & curr_time, const double base_link_z);
}  // namespace autoware::boundary_departure_checker::debug

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DEBUG_HPP_
