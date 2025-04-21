// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef FOOTPRINTS_HPP_
#define FOOTPRINTS_HPP_

#include "parameters.hpp"
#include "types.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
/// @brief Create the rtree for the segments of a trajectory
void prepare_trajectory_footprint_rtree(TrajectoryCornerFootprint & footprint);

/// @brief Calculate the corner footprint of the given trajectory
TrajectoryCornerFootprint calculate_trajectory_corner_footprint(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  autoware::vehicle_info_utils::VehicleInfo vehicle_info, const Parameters & params);
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // FOOTPRINTS_HPP_
