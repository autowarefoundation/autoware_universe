// Copyright 2026 Tier IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_INTERSECTION_MODULE__EXPERIMENTAL__UTIL_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_INTERSECTION_MODULE__EXPERIMENTAL__UTIL_HPP_

#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware/trajectory/utils/find_intervals.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_object_kinematics.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <optional>
#include <set>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental::util
{

using Trajectory = autoware::experimental::trajectory::Trajectory<
  autoware_internal_planning_msgs::msg::PathPointWithLaneId>;

struct FirstIndexInsidePolygons
{
  size_t polygon_index;
  double s;
};

struct MergedLanelets
{
  lanelet::ConstLanelets merged;
  std::vector<lanelet::ConstLanelets> originals;
};

std::vector<lanelet::CompoundPolygon3d> getPolygon3dFromLanelets(
  const lanelet::ConstLanelets & lanelets);

lanelet::ConstLanelets getPrevLanelets(
  const lanelet::ConstLanelets & lanelets_on_path, const std::set<lanelet::Id> & associative_ids);

double getHighestCurvature(const lanelet::ConstLineString3d & centerline);

std::optional<autoware_utils::Polygon2d> getIntersectionArea(
  const lanelet::ConstLanelet & assigned_lane, const lanelet::LaneletMapConstPtr lanelet_map_ptr);

std::vector<lanelet::ConstLanelet> getLaneletsOnPathFromCurrent(
  const Trajectory & path, const lanelet::LaneletMapPtr lanelet_map,
  const geometry_msgs::msg::Pose & current_pose);

std::optional<autoware::experimental::trajectory::Interval> findLaneIdsInterval(
  const Trajectory & path, const std::set<lanelet::Id> & ids);

std::optional<double> getFirstIndexInsidePolygon(
  const Trajectory & path, const autoware::experimental::trajectory::Interval & interval,
  const lanelet::CompoundPolygon3d & polygon, const bool search_forward = true);

std::optional<FirstIndexInsidePolygons> getFirstIndexInsidePolygons(
  const Trajectory & path, const autoware::experimental::trajectory::Interval & interval,
  const lanelet::CompoundPolygons3d & polygons, const bool search_forward = true);

std::optional<double> getFirstIndexInsidePolygonByFootprint(
  const Trajectory & path, const autoware::experimental::trajectory::Interval & interval,
  const lanelet::CompoundPolygon3d & polygon, const autoware_utils::LinearRing2d & footprint,
  const double vehicle_length);

std::optional<FirstIndexInsidePolygons> getFirstIndexInsidePolygonsByFootprint(
  const Trajectory & path, const autoware::experimental::trajectory::Interval & interval,
  const lanelet::CompoundPolygons3d & polygons, const autoware_utils::LinearRing2d & footprint,
  const double vehicle_length);

geometry_msgs::msg::Pose getObjectPoseWithVelocityDirection(
  const autoware_perception_msgs::msg::PredictedObjectKinematics & obj_state);

MergedLanelets mergeLaneletsByTopologicalSort(
  const lanelet::ConstLanelets & lanelets, const lanelet::ConstLanelets & terminal_lanelets,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr);

std::optional<double> findMaximumFootprintOvershootPosition(
  const Trajectory & path, const autoware_utils::LinearRing2d & local_footprint,
  const lanelet::ConstLanelet & lanelet, const double min_distance_threshold,
  const std::string & turn_direction);

std::optional<lanelet::ConstLanelet> generatePathLanelet(
  const Trajectory & path, const double start_s, const double end_s, const double width,
  const double interval);

}  // namespace autoware::behavior_velocity_planner::experimental::util

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_INTERSECTION_MODULE__EXPERIMENTAL__UTIL_HPP_
