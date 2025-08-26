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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__UTILS_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__UTILS_HPP_

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <vector>

namespace autoware::trajectory_modifier::utils
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

bool validate_trajectory(const TrajectoryPoints & trajectory);

}  // namespace autoware::trajectory_modifier::utils

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__UTILS_HPP_