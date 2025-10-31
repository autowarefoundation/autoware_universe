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

#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware/trajectory/utils/crossed.hpp>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace bg = boost::geometry;

auto calcStopPoint(
  const Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const lanelet::ConstLineString3d & lanelet_stop_lines, const double & offset)
  -> std::optional<double>
{
  for (size_t i = 0; i < lanelet_stop_lines.size() - 1; ++i) {
    const auto stop_line = planning_utils::extendSegmentToBounds(
      {lanelet_stop_lines[i].basicPoint2d(), lanelet_stop_lines[i + 1].basicPoint2d()}, left_bound,
      right_bound);
    const auto stop_point = autoware::experimental::trajectory::crossed(path, stop_line);
    if (stop_point.empty()) {
      continue;
    }
    return stop_point.front() - offset;
  }
  return std::nullopt;
}

bool isTrafficSignalRedStop(
  const lanelet::ConstLanelet & lanelet,
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & elements)
{
  using autoware::traffic_light_utils::hasTrafficLightCircleColor;
  using autoware::traffic_light_utils::hasTrafficLightShape;

  if (!hasTrafficLightCircleColor(
        elements, autoware_perception_msgs::msg::TrafficLightElement::RED)) {
    return false;
  }

  const std::string turn_direction = lanelet.attributeOr("turn_direction", "else");
  if (turn_direction == "else") {
    return true;
  }
  if (
    turn_direction == "right" &&
    hasTrafficLightShape(
      elements, autoware_perception_msgs::msg::TrafficLightElement::RIGHT_ARROW)) {
    return false;
  }
  if (
    turn_direction == "left" &&
    hasTrafficLightShape(
      elements, autoware_perception_msgs::msg::TrafficLightElement::LEFT_ARROW)) {
    return false;
  }
  if (
    turn_direction == "straight" &&
    hasTrafficLightShape(elements, autoware_perception_msgs::msg::TrafficLightElement::UP_ARROW)) {
    return false;
  }
  return true;
}
}  // namespace autoware::behavior_velocity_planner
