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
#include <rclcpp/duration.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/index/predicates.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <iterator>
#include <limits>
#include <stack>
#include <unordered_map>
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
 * @return the lanelet ids followed by the path
 */
lanelet::Ids get_predicted_path_lanelet_ids(
  const autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const route_handler::RouteHandler & route_handler)
{
  lanelet::Ids followed_ids;

  if (predicted_path.path.empty()) {
    return followed_ids;
  }
  struct SearchState
  {
    lanelet::Id lanelet_id{};              // current lanelet considered
    size_t current_idx{};                  // current index of the predicted path
    bool already_entered_lanelet = false;  // whether the predicted path already entered the lanelet
    std::vector<lanelet::Id> path_ids;     // the lanelet ids already traversed by the path
  };
  std::stack<SearchState> search_stack;

  // avoid re-calculating the same basic polygon many times
  std::unordered_map<lanelet::Id, lanelet::BasicPolygon2d> lanelet_polygons;
  const auto get_lanelet_polygon = [&](const lanelet::Id & id) {
    if (lanelet_polygons.count(id) == 0UL) {
      const auto & lanelet = route_handler.getLaneletMapPtr()->laneletLayer.get(id);
      lanelet_polygons[id] = lanelet.polygon2d().basicPolygon();
    }
    return lanelet_polygons[id];
  };

  // initial search states starting from the first possible lanelets
  const auto & start_position = predicted_path.path.front().position;
  const lanelet::BasicPoint2d start_point(start_position.x, start_position.y);
  const auto initial_lanelets =
    route_handler.getLaneletMapPtr()->laneletLayer.search({start_point, start_point});
  for (const auto & start_lanelet : initial_lanelets) {
    if (lanelet::geometry::within(start_point, get_lanelet_polygon(start_lanelet.id()))) {
      SearchState ss;
      ss.lanelet_id = start_lanelet.id();
      ss.current_idx = 1UL;
      ss.already_entered_lanelet = true;
      ss.path_ids = {start_lanelet.id()};
      search_stack.push(ss);
    }
  }

  while (!search_stack.empty()) {
    const auto current_state = search_stack.top();
    search_stack.pop();
    SearchState new_state;
    new_state.path_ids = current_state.path_ids;
    const auto & p = predicted_path.path[current_state.current_idx].position;
    const lanelet::BasicPoint2d point(p.x, p.y);

    if (lanelet::geometry::within(point, get_lanelet_polygon(current_state.lanelet_id))) {
      new_state.lanelet_id = current_state.lanelet_id;
      new_state.already_entered_lanelet = true;
      if (!current_state.already_entered_lanelet) {
        new_state.path_ids.push_back(current_state.lanelet_id);
      }
      const auto reached_end_of_path = current_state.current_idx + 1 == predicted_path.path.size();
      if (reached_end_of_path) {
        followed_ids.push_back(current_state.lanelet_id);
        followed_ids.insert(
          followed_ids.end(), current_state.path_ids.begin(), current_state.path_ids.end());
      } else {  // continue following the predicted path
        new_state.current_idx = current_state.current_idx + 1;
        search_stack.push(new_state);
      }
    } else {
      if (!current_state.already_entered_lanelet) {  // the path does not follow the current lanelet
        continue;
      }
      // explore the next lanelets
      new_state.current_idx = current_state.current_idx;
      const lanelet::ConstLanelets succeeding_lanelets =
        route_handler.getRoutingGraphPtr()->following(
          route_handler.getLaneletsFromId(current_state.lanelet_id));
      for (const auto & succeeding_lanelet : succeeding_lanelets) {
        new_state.lanelet_id = succeeding_lanelet.id();
        search_stack.push(new_state);
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
  if (!predicted_path_ids.has_value()) {
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
    object_path_lanelet_ids;  // will be calculated only once if a collision is found
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
