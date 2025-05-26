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

#ifndef AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__UTILS_HPP_

#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

namespace autoware::behavior_path_planner
{

// struct EgoGeometry
// {
//   double width;
//   double length;

//   autoware::universe_utils::Polygon2d make_polygon(const geometry_msgs::msg::Pose & ego_pose)
//   const
//   {
//     return autoware::universe_utils::toFootprint(ego_pose, length / 2.0, -length / 2.0, width);
//   }
// };

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__UTILS_HPP_
