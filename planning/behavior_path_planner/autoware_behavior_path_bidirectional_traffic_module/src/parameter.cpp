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

#include "autoware/behavior_path_bidirectional_traffic_module/parameter.hpp"

#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"

namespace autoware::behavior_path_planner
{

EgoParameters::EgoParameters(
  const double & base_link2front, const double & base_link2rear, const double & vehicle_width)
: base_link2front(base_link2front), base_link2rear(base_link2rear), vehicle_width(vehicle_width)
{
}

[[nodiscard]] autoware::universe_utils::Polygon2d EgoParameters::ego_polygon(
  const geometry_msgs::msg::Pose & ego_pose) const
{
  return autoware::universe_utils::toFootprint(
    ego_pose, base_link2front, base_link2rear, vehicle_width);
}

};  // namespace autoware::behavior_path_planner
