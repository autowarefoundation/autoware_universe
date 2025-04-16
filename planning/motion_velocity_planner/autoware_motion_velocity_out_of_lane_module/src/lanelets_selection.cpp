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

#include "lanelets_selection.hpp"

#include "types.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/strategies/cartesian/buffer_point_square.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <vector>

namespace autoware::motion_velocity_planner::out_of_lane
{

namespace
{
bool is_road_lanelet(const lanelet::ConstLanelet & lanelet)
{
  return lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
         lanelet.attribute(lanelet::AttributeName::Subtype) == lanelet::AttributeValueString::Road;
}
}  // namespace

lanelet::ConstLanelets calculate_trajectory_lanelets(
  const autoware_utils::LineString2d & trajectory_ls,
  const route_handler::RouteHandler & route_handler)
{
  const auto lanelet_map_ptr = route_handler.getLaneletMapPtr();
  lanelet::ConstLanelets trajectory_lanelets;
  const auto candidates = lanelet_map_ptr->laneletLayer.search(
    boost::geometry::return_envelope<lanelet::BoundingBox2d>(trajectory_ls));
  for (const auto & ll : candidates) {
    if (
      is_road_lanelet(ll) &&
      !boost::geometry::disjoint(trajectory_ls, ll.polygon2d().basicPolygon())) {
      trajectory_lanelets.push_back(ll);
    }
  }
  return trajectory_lanelets;
}

lanelet::ConstLanelets calculate_ignored_lanelets(
  const lanelet::ConstLanelets & trajectory_lanelets,
  const route_handler::RouteHandler & route_handler)
{
  lanelet::ConstLanelets ignored_lanelets;
  // ignore lanelets directly preceding a trajectory lanelet
  for (const auto & trajectory_lanelet : trajectory_lanelets) {
    for (const auto & ll : route_handler.getPreviousLanelets(trajectory_lanelet)) {
      const auto is_trajectory_lanelet = contains_lanelet(trajectory_lanelets, ll.id());
      if (!is_trajectory_lanelet) ignored_lanelets.push_back(ll);
    }
  }
  return ignored_lanelets;
}

lanelet::ConstLanelets calculate_out_lanelets(
  const lanelet::LaneletLayer & lanelet_layer,
  const autoware_utils::MultiPolygon2d & trajectory_footprints,
  const lanelet::ConstLanelets & trajectory_lanelets,
  const lanelet::ConstLanelets & ignored_lanelets)
{
  lanelet::ConstLanelets out_lanelets;
  const auto candidates = lanelet_layer.search(
    boost::geometry::return_envelope<lanelet::BoundingBox2d>(trajectory_footprints));
  for (const auto & lanelet : candidates) {
    const auto id = lanelet.id();
    if (
      contains_lanelet(trajectory_lanelets, id) || contains_lanelet(ignored_lanelets, id) ||
      !is_road_lanelet(lanelet)) {
      continue;
    }
    if (!boost::geometry::disjoint(trajectory_footprints, lanelet.polygon2d().basicPolygon())) {
      out_lanelets.push_back(lanelet);
    }
  }
  return out_lanelets;
}

OutLaneletRtree calculate_out_lanelet_rtree(const lanelet::ConstLanelets & lanelets)
{
  std::vector<LaneletNode> nodes;
  nodes.reserve(lanelets.size());
  for (auto i = 0UL; i < lanelets.size(); ++i) {
    nodes.emplace_back(
      boost::geometry::return_envelope<autoware_utils::Box2d>(
        lanelets[i].polygon2d().basicPolygon()),
      i);
  }
  return {nodes.begin(), nodes.end()};
}

void calculate_out_lanelet_rtree(
  EgoData & ego_data, const route_handler::RouteHandler & route_handler,
  const PlannerParam & params)
{
  autoware_utils::LineString2d trajectory_ls;
  for (const auto & p : ego_data.trajectory_points) {
    trajectory_ls.emplace_back(p.pose.position.x, p.pose.position.y);
  }
  // add a point beyond the last trajectory point to account for the ego front offset
  const auto pose_beyond = autoware_utils::calc_offset_pose(
    ego_data.trajectory_points.back().pose, params.front_offset, 0.0, 0.0, 0.0);
  trajectory_ls.emplace_back(pose_beyond.position.x, pose_beyond.position.y);
  const auto trajectory_lanelets = calculate_trajectory_lanelets(trajectory_ls, route_handler);
  const auto ignored_lanelets = calculate_ignored_lanelets(trajectory_lanelets, route_handler);

  const auto max_ego_footprint_offset = std::max({
    params.front_offset + params.extra_front_offset,
    params.left_offset + params.extra_left_offset,
    params.right_offset + params.extra_right_offset,
    params.rear_offset + params.extra_rear_offset,
  });
  autoware_utils::MultiPolygon2d trajectory_footprints;
  const boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(
    max_ego_footprint_offset);
  const boost::geometry::strategy::buffer::join_miter join_strategy;
  const boost::geometry::strategy::buffer::end_flat end_strategy;
  const boost::geometry::strategy::buffer::point_square circle_strategy;
  const boost::geometry::strategy::buffer::side_straight side_strategy;
  boost::geometry::buffer(
    trajectory_ls, trajectory_footprints, distance_strategy, side_strategy, join_strategy,
    end_strategy, circle_strategy);

  ego_data.out_lanelets = calculate_out_lanelets(
    route_handler.getLaneletMapPtr()->laneletLayer, trajectory_footprints, trajectory_lanelets,
    ignored_lanelets);
  ego_data.out_lanelets_rtree = calculate_out_lanelet_rtree(ego_data.out_lanelets);
}
}  // namespace autoware::motion_velocity_planner::out_of_lane
