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

#include "footprints.hpp"

#include "parameters.hpp"
#include "types.hpp"

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <boost/geometry/algorithms/correct.hpp>

#include <Eigen/src/Geometry/Rotation2D.h>
#include <Eigen/src/Geometry/RotationBase.h>

#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
void prepare_trajectory_footprint_rtree(TrajectoryCornerFootprint & footprint)
{
  SegmentRtree rtree;
  std::vector<FootprintSegmentNode> nodes;
  nodes.emplace_back(footprint.get_rear_segment(), std::make_pair(rear, 0UL));
  nodes.emplace_back(footprint.get_front_segment(), std::make_pair(front, 0UL));
  for (const auto corner : {front_left, front_right, rear_left, rear_right}) {
    const auto & ls = footprint.corner_footprint.corner_linestrings[corner];
    for (auto i = 0UL; i + 1 < ls.size(); ++i) {
      nodes.emplace_back(universe_utils::Segment2d{ls[i], ls[i + 1]}, std::make_pair(corner, i));
    }
  }
  footprint.rtree = FootprintSegmentRtree(nodes);
}

TrajectoryCornerFootprint calculate_trajectory_corner_footprint(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  autoware::vehicle_info_utils::VehicleInfo vehicle_info, const Parameters & params)
{
  run_out::TrajectoryCornerFootprint trajectory_footprint;
  trajectory_footprint.ego_trajectory = trajectory;
  auto & footprint = trajectory_footprint.corner_footprint;
  const auto base_footprint =
    vehicle_info.createFootprint(params.ego_lateral_margin, params.ego_longitudinal_margin);
  for (const auto & p : trajectory) {
    const universe_utils::Point2d base_link(p.pose.position.x, p.pose.position.y);
    const auto angle = tf2::getYaw(p.pose.orientation);
    const Eigen::Rotation2Dd rotation(angle);
    const auto rotated_front_left_offset =
      rotation * base_footprint[vehicle_info_utils::VehicleInfo::FrontLeftIndex];
    const auto rotated_front_right_offset =
      rotation * base_footprint[vehicle_info_utils::VehicleInfo::FrontRightIndex];
    const auto rotated_rear_right_offset =
      rotation * base_footprint[vehicle_info_utils::VehicleInfo::RearRightIndex];
    const auto rotated_rear_left_offset =
      rotation * base_footprint[vehicle_info_utils::VehicleInfo::RearLeftIndex];
    footprint.corner_linestrings[front_left].emplace_back(
      base_link.x() + rotated_front_left_offset.x(), base_link.y() + rotated_front_left_offset.y());
    footprint.corner_linestrings[front_right].emplace_back(
      base_link.x() + rotated_front_right_offset.x(),
      base_link.y() + rotated_front_right_offset.y());
    footprint.corner_linestrings[rear_right].emplace_back(
      base_link.x() + rotated_rear_right_offset.x(), base_link.y() + rotated_rear_right_offset.y());
    footprint.corner_linestrings[rear_left].emplace_back(
      base_link.x() + rotated_rear_left_offset.x(), base_link.y() + rotated_rear_left_offset.y());
    trajectory_footprint.max_longitudinal_offset = vehicle_info.max_longitudinal_offset_m;
  }

  const auto push_to_polygon_fn = [](universe_utils::Polygon2d & poly) {
    return [&](const universe_utils::Point2d & p) { poly.outer().push_back(p); };
  };
  std::for_each(
    footprint.corner_linestrings[front_left].begin(),
    footprint.corner_linestrings[front_left].end(),
    push_to_polygon_fn(trajectory_footprint.front_polygon));
  std::for_each(
    footprint.corner_linestrings[front_right].rbegin(),
    footprint.corner_linestrings[front_right].rend(),
    push_to_polygon_fn(trajectory_footprint.front_polygon));
  boost::geometry::correct(trajectory_footprint.front_polygon);
  std::for_each(
    footprint.corner_linestrings[rear_left].begin(), footprint.corner_linestrings[rear_left].end(),
    push_to_polygon_fn(trajectory_footprint.rear_polygon));
  std::for_each(
    footprint.corner_linestrings[rear_right].rbegin(),
    footprint.corner_linestrings[rear_right].rend(),
    push_to_polygon_fn(trajectory_footprint.rear_polygon));
  boost::geometry::correct(trajectory_footprint.rear_polygon);
  prepare_trajectory_footprint_rtree(trajectory_footprint);
  return trajectory_footprint;
}
}  // namespace autoware::motion_velocity_planner::run_out
