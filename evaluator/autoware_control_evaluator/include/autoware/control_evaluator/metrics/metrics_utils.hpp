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

#ifndef AUTOWARE__CONTROL_EVALUATOR__METRICS__METRICS_UTILS_HPP_
#define AUTOWARE__CONTROL_EVALUATOR__METRICS__METRICS_UTILS_HPP_

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>

#include <vector>
namespace control_diagnostics
{
namespace metrics
{
namespace utils
{
using autoware::route_handler::RouteHandler;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

/**
 * @brief Get the closest lanelets to the ego vehicle, considering shoulder lanelets.
 * @param [in] route_handler route handler
 * @param [in] ego_pose ego vehicle pose
 * @return closest lanelets to the ego vehicle
 **/
lanelet::ConstLanelets get_current_lanes(const RouteHandler & route_handler, const Pose & ego_pose);

/**
 * @brief Calculate the Euler distance between the vehicle and the lanelet.
 * @param [in] vehicle_footprint vehicle footprint
 * @param [in] line lanelet line
 * @return distance between the vehicle footprint and the lanelet, 0.0 if the vehicle intersects
 *with the line
 **/
double calc_distance_to_line(
  const autoware_utils::LinearRing2d & vehicle_footprint,
  const autoware_utils::LineString2d & line);

/**
 * @brief Check if the point is on the left side of the line.
 * @param [in] point point
 * @param [in] line line
 * @return true if the ego vehicle is on the left side of the lanelet line, false otherwise
 **/
bool is_point_left_of_line(const Point & point, const std::vector<Point> & line);

}  // namespace utils
}  // namespace metrics
}  // namespace control_diagnostics
#endif  // AUTOWARE__CONTROL_EVALUATOR__METRICS__METRICS_UTILS_HPP_
