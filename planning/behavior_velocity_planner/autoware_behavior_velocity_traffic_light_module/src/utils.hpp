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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#define EIGEN_MPL2_ONLY

#include <autoware/trajectory/path_point_with_lane_id.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <optional>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{

using Trajectory = autoware::experimental::trajectory::Trajectory<
  autoware_internal_planning_msgs::msg::PathPointWithLaneId>;

/**
 * @brief find intersection point between path and stop line and return arc length.
 * @param path input path.
 * @param left_bound left bound.
 * @param right_bound right bound.
 * @param lanelet_stop_lines stop lines.
 * @return arc length of stop point. if there is no intersection point, return std::nullopt.
 */
auto calcStopPoint(
  const Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const lanelet::ConstLineString3d & lanelet_stop_lines, const double & offset)
  -> std::optional<double>;

/**
 * @brief check if the traffic signal is red stop.
 * @param lanelet
 * @param elements
 * @return true if the traffic signal is red stop, false otherwise.
 */
bool isTrafficSignalRedStop(
  const lanelet::ConstLanelet & lanelet,
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & elements);

}  // namespace autoware::behavior_velocity_planner

#endif  // UTILS_HPP_
