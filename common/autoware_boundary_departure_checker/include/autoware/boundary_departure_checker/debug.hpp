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
#include "autoware/boundary_departure_checker/parameters.hpp"
#include "autoware/boundary_departure_checker/type_alias.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace autoware::boundary_departure_checker::debug
{
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

MarkerArray create_debug_marker_array(
  const DepartureData & departure_data, const Trajectory & ego_traj, const rclcpp::Time & curr_time,
  const double base_link_z, const Param & bdc_param);
}  // namespace autoware::boundary_departure_checker::debug

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DEBUG_HPP_
