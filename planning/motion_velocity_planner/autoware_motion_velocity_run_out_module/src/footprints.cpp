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
  std::vector<FootprintSegmentNode> nodes;
  nodes.emplace_back(footprint.get_rear_segment(0), std::make_pair(rear, 0UL));
  nodes.emplace_back(footprint.get_front_segment(0), std::make_pair(front, 0UL));
  for (const auto corner : {front_left, front_right, rear_left, rear_right}) {
    const auto & ls = footprint.predicted_path_footprint.corner_linestrings[corner];
    for (auto i = 0UL; i + 1 < ls.size(); ++i) {
      nodes.emplace_back(universe_utils::Segment2d{ls[i], ls[i + 1]}, std::make_pair(corner, i));
    }
  }
  const auto max_index = footprint.ego_trajectory.size() - 1UL;
  nodes.emplace_back(footprint.get_rear_segment(max_index), std::make_pair(rear, max_index));
  nodes.emplace_back(footprint.get_front_segment(max_index), std::make_pair(front, max_index));
  footprint.segments_rtree = FootprintSegmentRtree(nodes);
  footprint.front_polygons_rtree = PolygonRtree(footprint.front_polygons);
  footprint.rear_polygons_rtree = PolygonRtree(footprint.rear_polygons);
}

TrajectoryCornerFootprint calculate_trajectory_corner_footprint(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  autoware::vehicle_info_utils::VehicleInfo vehicle_info, const Parameters & params)
{
  run_out::TrajectoryCornerFootprint trajectory_footprint;
  trajectory_footprint.ego_trajectory = trajectory;
  auto & footprint = trajectory_footprint.predicted_path_footprint;
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
  for (auto i = 0UL; i + 1 < footprint.corner_linestrings[front_left].size(); ++i) {
    universe_utils::LinearRing2d front_polygon = {
      footprint.corner_linestrings[front_left][i],
      footprint.corner_linestrings[front_left][i + 1],
      footprint.corner_linestrings[front_right][i + 1],
      footprint.corner_linestrings[front_right][i],
    };
    universe_utils::LinearRing2d rear_polygon = {
      footprint.corner_linestrings[rear_left][i],
      footprint.corner_linestrings[rear_left][i + 1],
      footprint.corner_linestrings[rear_right][i + 1],
      footprint.corner_linestrings[rear_right][i],
    };
    boost::geometry::correct(front_polygon);
    boost::geometry::correct(rear_polygon);
    trajectory_footprint.front_polygons.push_back(front_polygon);
    trajectory_footprint.rear_polygons.push_back(rear_polygon);
  }
  prepare_trajectory_footprint_rtree(trajectory_footprint);
  return trajectory_footprint;
}
}  // namespace autoware::motion_velocity_planner::run_out
