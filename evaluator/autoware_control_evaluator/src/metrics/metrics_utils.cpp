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

#include "autoware/control_evaluator/metrics/metrics_utils.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>

#include <boost/geometry.hpp>
// #include <boost/geometry/algorithms/detail/distance/interface.hpp>

#include <lanelet2_core/Forward.h>

#include <cstddef>
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

lanelet::ConstLanelets get_current_lanes(const RouteHandler & route_handler, const Pose & ego_pose)
{
  lanelet::ConstLanelet closest_route_lanelet;
  route_handler.getClosestLaneletWithinRoute(ego_pose, &closest_route_lanelet);
  const auto shoulder_lanelets = route_handler.getShoulderLaneletsAtPose(ego_pose);
  lanelet::ConstLanelets closest_lanelets{closest_route_lanelet};
  closest_lanelets.insert(
    closest_lanelets.end(), shoulder_lanelets.begin(), shoulder_lanelets.end());
  return closest_lanelets;
}

double calc_distance_to_line(
  const autoware_utils::LinearRing2d & vehicle_footprint, const autoware_utils::LineString2d & line)
{
  return boost::geometry::distance(vehicle_footprint, line);
}

bool is_point_left_of_line(const Point & point, const std::vector<Point> & line)
{
  const size_t closest_idx = autoware::motion_utils::findNearestSegmentIndex(line, point);
  const auto & p1 = line[closest_idx];
  const auto & p2 = line[closest_idx + 1];
  return ((p2.x - p1.x) * (point.y - p1.y) - (p2.y - p1.y) * (point.x - p1.x)) > 0;
}

}  // namespace utils
}  // namespace metrics
}  // namespace control_diagnostics
