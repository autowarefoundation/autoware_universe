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
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <vector>

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__TYPE_ALIAS_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__TYPE_ALIAS_HPP_

namespace autoware::boundary_departure_checker
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_vehicle_msgs::msg::SteeringReport;

using autoware_utils::Box2d;
using autoware_utils::LinearRing2d;
using autoware_utils::LineString2d;
using autoware_utils::MultiPoint2d;
using autoware_utils::MultiPolygon2d;
using autoware_utils::Point2d;  // NOLINT
using autoware_utils::Polygon2d;
using autoware_utils::Segment2d;

using autoware::vehicle_info_utils::VehicleInfo;  // NOLINT

namespace bg = boost::geometry;
namespace bgi = bg::index;                        // NOLINT
namespace trajectory = experimental::trajectory;  // NOLINT

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

using TrajectoryPoints = std::vector<TrajectoryPoint>;

}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__TYPE_ALIAS_HPP_
