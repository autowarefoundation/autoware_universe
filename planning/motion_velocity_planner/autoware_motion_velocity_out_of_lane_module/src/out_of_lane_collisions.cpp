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

#include "out_of_lane_collisions.hpp"

#include "types.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/duration.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <boost/geometry/algorithms/detail/intersects/interface.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/index/predicates.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <iterator>
#include <limits>
#include <stack>
#include <unordered_set>
#include <vector>

namespace autoware::motion_velocity_planner::out_of_lane
{

void update_collision_times(
  OutOfLaneData & out_of_lane_data, const std::unordered_set<size_t> & potential_collision_indexes,
  const autoware_utils::Polygon2d & object_footprint, const double time)
{
  for (const auto index : potential_collision_indexes) {
    auto & out_of_lane_point = out_of_lane_data.outside_points[index];
    if (out_of_lane_point.collision_times.count(time) == 0UL) {
      const auto has_collision =
        !boost::geometry::disjoint(out_of_lane_point.out_overlaps, object_footprint.outer());
      if (has_collision) {
        out_of_lane_point.collision_times.insert(time);
      }
    }
  }
}

/**
 * @brief Finds lanelet ids followed by a PredictedPath.
 * @details searches the lanelet routing graph for a full path of succeeding lanelets containing the
 * predicted path
 * @param predicted_path The predicted path of an object.
 * @param route_handler the route handler with the lanelet map and routing graph
 * @return the sorted lanelet ids followed by the path
 */
lanelet::Ids get_predicted_path_lanelet_ids(
  const autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const route_handler::RouteHandler & route_handler)
{
  lanelet::Ids followed_ids;

  if (predicted_path.path.empty()) {
    return followed_ids;
  }
  struct SearchState  // search state for exploring the routing graph
  {
    lanelet::Id lanelet_id{};                      // current lanelet considered
    std::unordered_set<lanelet::Id> explored_ids;  // the lanelet ids already traversed by the path
  };
  std::stack<SearchState> search_stack;
  // initial search states starting from the first possible lanelets
  lanelet::BasicLineString2d ls;
  for (const auto & p : predicted_path.path) {
    ls.emplace_back(p.position.x, p.position.y);
  }
  const auto candidates =
    route_handler.getLaneletMapPtr()->laneletLayer.search(lanelet::geometry::boundingBox2d(ls));
  const auto final_candidates =
    route_handler.getLaneletMapPtr()->laneletLayer.search({ls.back(), ls.back()});
  lanelet::Ids final_ids;
  for (const auto & final_candidate : final_candidates) {
    if (lanelet::geometry::within(
          ls.back(), route_handler.getLaneletsFromId(final_candidate.id()).polygon2d())) {
      final_ids.push_back(final_candidate.id());
    }
  }
  std::sort(final_ids.begin(), final_ids.end());
  const auto initial_candidates =
    route_handler.getLaneletMapPtr()->laneletLayer.search({ls.front(), ls.front()});
  lanelet::Ids initial_ids;
  for (const auto & initial_candidate : initial_candidates) {
    if (lanelet::geometry::within(
          ls.front(), route_handler.getLaneletsFromId(initial_candidate.id()).polygon2d())) {
      initial_ids.push_back(initial_candidate.id());
    }
  }
  std::sort(initial_ids.begin(), initial_ids.end());
  lanelet::Ids overlapped_lanelets = final_ids;
  overlapped_lanelets.insert(overlapped_lanelets.end(), initial_ids.begin(), initial_ids.end());
  for (const auto & candidate : candidates) {
    const auto id = candidate.id();
    // check if the id is already in the initial/final ids to avoid redoing the geometry operation
    if (
      !std::binary_search(initial_ids.begin(), initial_ids.end(), id) &&
      !std::binary_search(final_ids.begin(), final_ids.end(), id) &&
      !boost::geometry::disjoint(
        ls, route_handler.getLaneletsFromId(id).polygon2d().basicPolygon())) {
      overlapped_lanelets.push_back(id);
    }
  }
  std::sort(overlapped_lanelets.begin(), overlapped_lanelets.end());

  for (const auto initial_id : initial_ids) {
    SearchState ss;
    ss.lanelet_id = initial_id;
    ss.explored_ids = {initial_id};
    search_stack.push(ss);
  }

  while (!search_stack.empty()) {
    const auto current_state = search_stack.top();
    search_stack.pop();
    const auto is_final_lanelet =
      std::binary_search(final_ids.begin(), final_ids.end(), current_state.lanelet_id);
    if (is_final_lanelet) {
      followed_ids.insert(
        followed_ids.end(), current_state.explored_ids.begin(), current_state.explored_ids.end());
      continue;
    }
    const lanelet::ConstLanelets succeeding_lanelets =
      route_handler.getRoutingGraphPtr()->following(
        route_handler.getLaneletsFromId(current_state.lanelet_id));
    for (const auto & succeeding_lanelet : succeeding_lanelets) {
      const auto is_overlapped_lanelet = std::binary_search(
        overlapped_lanelets.begin(), overlapped_lanelets.end(), succeeding_lanelet.id());
      // prevent loops by making sure the id was not already explored
      if (is_overlapped_lanelet && current_state.explored_ids.count(succeeding_lanelet.id()) == 0) {
        SearchState ss;
        ss.lanelet_id = succeeding_lanelet.id();
        ss.explored_ids = current_state.explored_ids;
        ss.explored_ids.insert(succeeding_lanelet.id());
        search_stack.push(ss);
      }
    }
  }
  // Remove duplicate ids
  std::sort(followed_ids.begin(), followed_ids.end());
  followed_ids.erase(std::unique(followed_ids.begin(), followed_ids.end()), followed_ids.end());
  return followed_ids;
}

/**
 * @brief return true if there is at least one match between the given lanelets and lanelets ids
 */
bool at_least_one_lanelet_in_common(
  const lanelet::ConstLanelets & out_lanelets, std::optional<lanelet::Ids> & predicted_path_ids,
  const autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const route_handler::RouteHandler & route_handler)
{
  if (!predicted_path_ids.has_value()) {  // ensure the predicted path ids are only calculated once
    predicted_path_ids = get_predicted_path_lanelet_ids(predicted_path, route_handler);
  }
  for (const auto & ll : out_lanelets) {
    if (std::binary_search(predicted_path_ids->begin(), predicted_path_ids->end(), ll.id())) {
      return true;
    }
  }
  return false;
}

void calculate_object_path_time_collisions(
  OutOfLaneData & out_of_lane_data,
  const autoware_perception_msgs::msg::PredictedPath & object_path,
  const autoware_perception_msgs::msg::Shape & object_shape,
  const route_handler::RouteHandler & route_handler,
  const bool validate_predicted_paths_on_lanelets)
{
  const auto time_step = rclcpp::Duration(object_path.time_step).seconds();
  auto time = 0.0;
  std::optional<lanelet::Ids>
    object_path_lanelet_ids;  // calculated only once for the 1st collision found
  for (const auto & object_pose : object_path.path) {
    const auto object_footprint = autoware_utils::to_polygon2d(object_pose, object_shape);
    std::vector<OutAreaNode> query_results;
    out_of_lane_data.outside_areas_rtree.query(
      boost::geometry::index::intersects(object_footprint.outer()),
      std::back_inserter(query_results));
    std::unordered_set<size_t> potential_collision_indexes;
    for (const auto & [_, index] : query_results) {
      const auto & out_lanelets = out_of_lane_data.outside_points[index].overlapped_lanelets;
      if (
        !validate_predicted_paths_on_lanelets ||
        at_least_one_lanelet_in_common(
          out_lanelets, object_path_lanelet_ids, object_path, route_handler)) {
        potential_collision_indexes.insert(index);
      }
    }
    update_collision_times(out_of_lane_data, potential_collision_indexes, object_footprint, time);
    time += time_step;
  }
}

void calculate_objects_time_collisions(
  OutOfLaneData & out_of_lane_data,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects,
  const route_handler::RouteHandler & route_handler,
  const bool validate_predicted_paths_on_lanelets)
{
  for (const auto & object : objects) {
    for (const auto & path : object.kinematics.predicted_paths) {
      calculate_object_path_time_collisions(
        out_of_lane_data, path, object.shape, route_handler, validate_predicted_paths_on_lanelets);
    }
  }
}

void calculate_min_max_arrival_times(
  OutOfLanePoint & out_of_lane_point,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory)
{
  auto min_time = std::numeric_limits<double>::infinity();
  auto max_time = -std::numeric_limits<double>::infinity();
  for (const auto & t : out_of_lane_point.collision_times) {
    min_time = std::min(t, min_time);
    max_time = std::max(t, max_time);
  }
  if (min_time <= max_time) {
    out_of_lane_point.min_object_arrival_time = min_time;
    out_of_lane_point.max_object_arrival_time = max_time;
    const auto & ego_time =
      rclcpp::Duration(trajectory[out_of_lane_point.trajectory_index].time_from_start).seconds();
    if (ego_time >= min_time && ego_time <= max_time) {
      out_of_lane_point.ttc = 0.0;
    } else {
      out_of_lane_point.ttc =
        std::min(std::abs(ego_time - min_time), std::abs(ego_time - max_time));
    }
  }
};

void calculate_collisions_to_avoid(
  OutOfLaneData & out_of_lane_data,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const PlannerParam & params, const bool is_stopping)
{
  for (auto & p : out_of_lane_data.outside_points) {
    calculate_min_max_arrival_times(p, trajectory);
    if (params.mode == "ttc") {
      const auto threshold = is_stopping ? params.ttc_release_threshold : params.ttc_threshold;
      p.to_avoid = p.ttc && p.ttc <= threshold;
    } else {
      p.to_avoid = p.min_object_arrival_time && p.min_object_arrival_time <= params.time_threshold;
    }
  }
}

OutOfLanePoint calculate_out_of_lane_point(
  const lanelet::BasicPolygon2d & footprint, const lanelet::ConstLanelets & out_lanelets,
  const OutLaneletRtree & out_lanelets_rtree)
{
  OutOfLanePoint p;
  std::vector<LaneletNode> candidates;
  out_lanelets_rtree.query(
    boost::geometry::index::intersects(footprint), std::back_inserter(candidates));
  for (const auto & [_, idx] : candidates) {
    const auto & lanelet = out_lanelets[idx];
    lanelet::BasicPolygons2d intersections;
    boost::geometry::intersection(footprint, lanelet.polygon2d().basicPolygon(), intersections);
    for (const auto & intersection : intersections) {
      autoware_utils::Polygon2d poly;
      boost::geometry::convert(intersection, poly);
      p.out_overlaps.push_back(poly);
    }
    if (!intersections.empty()) {
      p.overlapped_lanelets.push_back(lanelet);
    }
  }
  return p;
}
std::vector<OutOfLanePoint> calculate_out_of_lane_points(const EgoData & ego_data)
{
  std::vector<OutOfLanePoint> out_of_lane_points;
  for (auto i = 0UL; i < ego_data.trajectory_footprints.size(); ++i) {
    const auto & footprint = ego_data.trajectory_footprints[i];
    OutOfLanePoint p =
      calculate_out_of_lane_point(footprint, ego_data.out_lanelets, ego_data.out_lanelets_rtree);
    p.trajectory_index = i;
    if (!p.overlapped_lanelets.empty()) {
      out_of_lane_points.push_back(p);
    }
  }
  return out_of_lane_points;
}

void prepare_out_of_lane_areas_rtree(OutOfLaneData & out_of_lane_data)
{
  std::vector<OutAreaNode> rtree_nodes;
  for (auto i = 0UL; i < out_of_lane_data.outside_points.size(); ++i) {
    for (const auto & out_overlap : out_of_lane_data.outside_points[i].out_overlaps) {
      OutAreaNode n;
      n.first = boost::geometry::return_envelope<autoware_utils::Box2d>(out_overlap);
      n.second = i;
      rtree_nodes.push_back(n);
    }
  }
  out_of_lane_data.outside_areas_rtree = {rtree_nodes.begin(), rtree_nodes.end()};
}

}  // namespace autoware::motion_velocity_planner::out_of_lane
