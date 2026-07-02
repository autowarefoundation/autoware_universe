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

#include "autoware/behavior_velocity_intersection_module/experimental/util.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/trajectory/utils/crop.hpp>
#include <autoware/trajectory/utils/curvature_utils.hpp>
#include <autoware/trajectory/utils/find_if.hpp>
#include <autoware/trajectory/utils/footprint.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <limits>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental::util
{

namespace bg = boost::geometry;
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;

namespace
{
bool isPointInsidePolygon(
  const PathPointWithLaneId & point, const lanelet::CompoundPolygon2d & polygon)
{
  return bg::within(
    autoware::experimental::lanelet2_utils::from_ros(point.point.pose.position).basicPoint2d(),
    polygon);
}

bool isPointInsidePolygonByFootprint(
  const PathPointWithLaneId & point, const lanelet::CompoundPolygon2d & polygon,
  const autoware_utils::LinearRing2d & footprint)
{
  const auto path_footprint =
    autoware_utils::transform_vector(footprint, autoware_utils::pose2transform(point.point.pose));
  const autoware_utils_geometry::LineString2d footprint_front_part{
    path_footprint.at(vehicle_info_utils::VehicleInfo::FrontLeftIndex),
    path_footprint.at(vehicle_info_utils::VehicleInfo::FrontRightIndex)};
  const auto basic_polygon = polygon.basicPolygon();
  return bg::intersects(footprint_front_part, basic_polygon) ||
         bg::within(footprint_front_part, basic_polygon);
}

void retrievePathsBackward(
  const std::vector<std::vector<bool>> & adjacency, const size_t src_index,
  const std::vector<size_t> & visited_indices, std::vector<std::vector<size_t>> & paths)
{
  const auto & next_indices = adjacency.at(src_index);
  const auto is_terminal =
    (std::find(next_indices.begin(), next_indices.end(), true) == next_indices.end());

  if (is_terminal) {
    auto path = visited_indices;
    path.push_back(src_index);
    paths.emplace_back(std::move(path));
    return;
  }

  for (size_t next = 0; next < next_indices.size(); next++) {
    if (!next_indices.at(next)) {
      continue;
    }
    if (std::find(visited_indices.begin(), visited_indices.end(), next) != visited_indices.end()) {
      // loop detected
      auto path = visited_indices;
      path.push_back(src_index);
      paths.emplace_back(std::move(path));
      continue;
    }
    auto new_visited_indices = visited_indices;
    new_visited_indices.push_back(src_index);
    retrievePathsBackward(adjacency, next, new_visited_indices, paths);
  }
}
}  // namespace

std::vector<lanelet::CompoundPolygon3d> getPolygon3dFromLanelets(
  const lanelet::ConstLanelets & lanelets)
{
  std::vector<lanelet::CompoundPolygon3d> polys;
  for (const auto & lanelet : lanelets) {
    polys.push_back(lanelet.polygon3d());
  }
  return polys;
}

lanelet::ConstLanelets getPrevLanelets(
  const lanelet::ConstLanelets & lanelets_on_path, const std::set<lanelet::Id> & associative_ids)
{
  lanelet::ConstLanelets previous_lanelets;
  for (const auto & lanelet : lanelets_on_path) {
    if (associative_ids.find(lanelet.id()) != associative_ids.end()) {
      return previous_lanelets;
    }
    previous_lanelets.push_back(lanelet);
  }
  return previous_lanelets;
}

double getHighestCurvature(const lanelet::ConstLineString3d & centerline)
{
  std::vector<geometry_msgs::msg::Point> points(centerline.size());
  std::transform(
    centerline.begin(), centerline.end(), points.begin(), [](const lanelet::ConstPoint3d & p) {
      return autoware::experimental::lanelet2_utils::to_ros(p);
    });

  const auto trajectory =
    autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Point>::Builder{}.build(
      points);
  return autoware::experimental::trajectory::max_curvature(trajectory.value());
}

std::optional<autoware_utils::Polygon2d> getIntersectionArea(
  const lanelet::ConstLanelet & assigned_lane, const lanelet::LaneletMapConstPtr lanelet_map_ptr)
{
  if (!assigned_lane.hasAttribute("intersection_area")) {
    return std::nullopt;
  }

  const auto area_id = assigned_lane.attribute("intersection_area").asId();
  if (!area_id) {
    return std::nullopt;
  }

  const auto polygon_it = lanelet_map_ptr->polygonLayer.find(*area_id);
  if (polygon_it == lanelet_map_ptr->polygonLayer.end()) {
    return std::nullopt;
  }

  autoware_utils::Polygon2d poly{};
  for (const auto & p : *polygon_it) {
    poly.outer().emplace_back(p.x(), p.y());
  }

  return poly;
}

std::vector<lanelet::ConstLanelet> getLaneletsOnPathFromCurrent(
  const Trajectory & path, const lanelet::LaneletMapPtr lanelet_map,
  const geometry_msgs::msg::Pose & current_pose)
{
  const auto lane_ids = path.get_contained_lane_ids();

  std::vector<lanelet::ConstLanelet> lanelets;
  lanelets.reserve(lane_ids.size());
  for (const auto lane_id : lane_ids) {
    lanelets.push_back(lanelet_map->laneletLayer.get(lane_id));
  }

  const auto closest_lanelet_opt =
    autoware::experimental::lanelet2_utils::get_closest_lanelet(lanelets, current_pose);
  if (!closest_lanelet_opt) {
    return lanelets;
  }

  lanelets.erase(
    lanelets.begin(), std::find(lanelets.begin(), lanelets.end(), closest_lanelet_opt.value()));

  return lanelets;
}

std::optional<autoware::experimental::trajectory::Interval> findLaneIdsInterval(
  const Trajectory & path, const std::set<lanelet::Id> & ids)
{
  const auto intervals =
    autoware::experimental::trajectory::find_intervals(path, [&](const PathPointWithLaneId & p) {
      return std::any_of(p.lane_ids.begin(), p.lane_ids.end(), [&](const lanelet::Id lane_id) {
        return ids.find(lane_id) != ids.end();
      });
    });

  if (intervals.empty()) {
    return std::nullopt;
  }
  return intervals.front();
}

std::optional<double> getFirstIndexInsidePolygon(
  const Trajectory & path, const autoware::experimental::trajectory::Interval & interval,
  const lanelet::CompoundPolygon3d & polygon, const bool search_forward)
{
  const auto cropped_path =
    autoware::experimental::trajectory::crop(path, interval.start, interval.end);
  const auto polygon_2d = lanelet::utils::to2D(polygon);

  std::optional<double> first_index{};
  if (search_forward) {
    first_index = autoware::experimental::trajectory::find_first_index_if(
      cropped_path,
      [&](const PathPointWithLaneId & p) { return isPointInsidePolygon(p, polygon_2d); });
  } else {
    first_index = autoware::experimental::trajectory::find_last_index_if(
      cropped_path,
      [&](const PathPointWithLaneId & p) { return isPointInsidePolygon(p, polygon_2d); });
  }

  if (!first_index.has_value()) {
    return std::nullopt;
  }
  return interval.start + first_index.value();
}

std::optional<FirstIndexInsidePolygons> getFirstIndexInsidePolygons(
  const Trajectory & path, const autoware::experimental::trajectory::Interval & interval,
  const lanelet::CompoundPolygons3d & polygons, const bool search_forward)
{
  const auto cropped_path =
    autoware::experimental::trajectory::crop(path, interval.start, interval.end);

  const auto find_polygon_it = [&polygons](const PathPointWithLaneId & p) {
    return std::find_if(
      polygons.begin(), polygons.end(), [&](const lanelet::CompoundPolygon3d & polygon) {
        return isPointInsidePolygon(p, lanelet::utils::to2D(polygon));
      });
  };

  const auto is_inside_polygons = [&find_polygon_it, &polygons](const PathPointWithLaneId & p) {
    return find_polygon_it(p) != polygons.end();
  };

  std::optional<double> first_index{};
  if (search_forward) {
    first_index =
      autoware::experimental::trajectory::find_first_index_if(cropped_path, is_inside_polygons);
  } else {
    first_index =
      autoware::experimental::trajectory::find_last_index_if(cropped_path, is_inside_polygons);
  }

  if (!first_index.has_value()) {
    return std::nullopt;
  }

  const auto polygon_it = find_polygon_it(cropped_path.compute(first_index.value()));
  if (polygon_it == polygons.end()) {
    return std::nullopt;
  }

  return FirstIndexInsidePolygons{
    static_cast<size_t>(std::distance(polygons.begin(), polygon_it)),
    interval.start + first_index.value()};
}

std::optional<double> getFirstIndexInsidePolygonByFootprint(
  const Trajectory & path, const autoware::experimental::trajectory::Interval & interval,
  const lanelet::CompoundPolygon3d & polygon, const autoware_utils::LinearRing2d & footprint,
  const double vehicle_length)
{
  const auto start_s = std::max(0.0, interval.start - vehicle_length);
  const auto cropped_path = autoware::experimental::trajectory::crop(path, start_s, interval.end);

  const auto first_index = autoware::experimental::trajectory::find_first_index_if(
    cropped_path, [&](const PathPointWithLaneId & p) {
      return isPointInsidePolygonByFootprint(p, lanelet::utils::to2D(polygon), footprint);
    });

  if (!first_index.has_value()) {
    return std::nullopt;
  }
  return start_s + first_index.value();
}

std::optional<FirstIndexInsidePolygons> getFirstIndexInsidePolygonsByFootprint(
  const Trajectory & path, const autoware::experimental::trajectory::Interval & interval,
  const lanelet::CompoundPolygons3d & polygons, const autoware_utils::LinearRing2d & footprint,
  const double vehicle_length)
{
  const auto start_s = std::max(0.0, interval.start - vehicle_length);
  const auto cropped_path = autoware::experimental::trajectory::crop(path, start_s, interval.end);

  const auto find_polygon_it = [&polygons, &footprint](const PathPointWithLaneId & p) {
    return std::find_if(
      polygons.begin(), polygons.end(), [&](const lanelet::CompoundPolygon3d & polygon) {
        return isPointInsidePolygonByFootprint(p, lanelet::utils::to2D(polygon), footprint);
      });
  };

  const auto first_index = autoware::experimental::trajectory::find_first_index_if(
    cropped_path, [&find_polygon_it, &polygons](const PathPointWithLaneId & p) {
      return find_polygon_it(p) != polygons.end();
    });

  if (!first_index.has_value()) {
    return std::nullopt;
  }

  const auto polygon_it = find_polygon_it(path.compute(first_index.value()));
  if (polygon_it == polygons.end()) {
    return std::nullopt;
  }

  return FirstIndexInsidePolygons{
    static_cast<size_t>(std::distance(polygons.begin(), polygon_it)),
    start_s + first_index.value()};
}

geometry_msgs::msg::Pose getObjectPoseWithVelocityDirection(
  const autoware_perception_msgs::msg::PredictedObjectKinematics & obj_state)
{
  if (obj_state.initial_twist_with_covariance.twist.linear.x >= 0) {
    return obj_state.initial_pose_with_covariance.pose;
  }

  // When the object velocity is negative, invert orientation (yaw)
  auto obj_pose = obj_state.initial_pose_with_covariance.pose;
  double yaw, pitch, roll;
  tf2::getEulerYPR(obj_pose.orientation, yaw, pitch, roll);
  tf2::Quaternion inv_q;
  inv_q.setRPY(roll, pitch, yaw + M_PI);
  obj_pose.orientation = tf2::toMsg(inv_q);
  return obj_pose;
}

MergedLanelets mergeLaneletsByTopologicalSort(
  const lanelet::ConstLanelets & lanelets, const lanelet::ConstLanelets & terminal_lanelets,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr)
{
  std::set<lanelet::Id> lanelet_ids;
  std::unordered_map<lanelet::Id, size_t> id_to_index;
  std::unordered_map<size_t, lanelet::Id> index_to_id;
  std::unordered_map<lanelet::Id, lanelet::ConstLanelets::const_iterator> id_to_lanelet;
  for (auto it = lanelets.begin(); it != lanelets.end(); ++it) {
    const auto index = index_to_id.size();
    const auto id = it->id();
    lanelet_ids.insert(id);
    id_to_index[id] = index;
    index_to_id[index] = id;
    id_to_lanelet[id] = it;
  }

  std::set<size_t> terminal_indices;
  for (const auto & terminal_lanelet : terminal_lanelets) {
    if (id_to_index.count(terminal_lanelet.id()) > 0) {
      terminal_indices.insert(id_to_index[terminal_lanelet.id()]);
    }
  }

  // create adjacency matrix
  const auto num_nodes = lanelets.size();
  std::vector<std::vector<bool>> adjacency(num_nodes);
  for (size_t dst = 0; dst < num_nodes; ++dst) {
    adjacency[dst].resize(num_nodes);
    for (size_t src = 0; src < num_nodes; ++src) {
      adjacency[dst][src] = false;
    }
  }

  // NOTE: this function aims to traverse the detection lanelet in the lane direction, so if lane B
  // follows lane A on the routing_graph, adj[A][B] = true
  for (const auto & lanelet : lanelets) {
    const auto & followings = routing_graph_ptr->following(lanelet);
    const auto src = lanelet.id();
    for (const auto & following : followings) {
      if (const auto dst = following.id(); lanelet_ids.find(dst) != lanelet_ids.end()) {
        adjacency[id_to_index[dst]][id_to_index[src]] = true;
      }
    }
  }

  std::unordered_map<size_t, std::vector<std::vector<size_t>>> branches;
  for (const auto & terminal_index : terminal_indices) {
    std::vector<std::vector<size_t>> paths;
    std::vector<size_t> visited;
    retrievePathsBackward(adjacency, terminal_index, visited, paths);
    branches[terminal_index] = std::move(paths);
  }

  for (auto & [_, paths] : branches) {
    for (auto & path : paths) {
      std::reverse(path.begin(), path.end());
    }
  }

  lanelet::ConstLanelets merged;
  std::vector<lanelet::ConstLanelets> originals;
  for (const auto & [_, sub_branches] : branches) {
    if (sub_branches.size() == 0) {
      continue;
    }
    for (const auto & sub_indices : sub_branches) {
      auto & original = originals.emplace_back();
      lanelet::ConstLanelets to_be_merged;
      for (const auto & sub_index : sub_indices) {
        to_be_merged.push_back(*id_to_lanelet[index_to_id[sub_index]]);
        original.push_back(*id_to_lanelet[index_to_id[sub_index]]);
      }
      const auto merged_lanelet =
        autoware::experimental::lanelet2_utils::combine_lanelets_shape(to_be_merged);
      if (!merged_lanelet.has_value()) {
        continue;
      }
      merged.push_back(merged_lanelet.value());
    }
  }

  return {merged, originals};
}

std::optional<double> findMaximumFootprintOvershootPosition(
  const Trajectory & path, const autoware_utils::LinearRing2d & local_footprint,
  const lanelet::ConstLanelet & lanelet, const double min_distance_threshold,
  const std::string & turn_direction)
{
  if (turn_direction != "left" && turn_direction != "right") {
    return std::nullopt;
  }

  const auto right_bound = lanelet.rightBound2d().basicLineString();
  const auto left_bound = lanelet.leftBound2d().basicLineString();

  std::optional<lanelet::BasicLineString2d> target_boundary{std::nullopt};

  const auto intersection_index = autoware::experimental::trajectory::find_first_index_if(
    path, [&](const PathPointWithLaneId & p) {
      const auto footprint = autoware_utils::transform_vector(
        local_footprint, autoware_utils::pose2transform(p.point.pose));
      if (boost::geometry::intersects(footprint, right_bound)) {
        if (!target_boundary) {
          target_boundary = right_bound;
        }
        return true;
      }
      if (boost::geometry::intersects(footprint, left_bound)) {
        if (!target_boundary) {
          target_boundary = left_bound;
        }
        return true;
      }
      return false;
    });

  if (!target_boundary || !intersection_index) {
    return std::nullopt;
  }

  const auto cropped_path =
    autoware::experimental::trajectory::crop(path, intersection_index.value(), path.length());

  auto closest_dist = std::numeric_limits<double>::infinity();
  std::optional<double> closest_s{std::nullopt};

  for (const auto & s : cropped_path.get_underlying_bases()) {
    const auto & base_pose = cropped_path.compute(s).point.pose;
    const auto footprint =
      autoware_utils::transform_vector(local_footprint, autoware_utils::pose2transform(base_pose));
    if (boost::geometry::intersects(footprint, *target_boundary)) {
      return s;
    }

    auto footprint_to_boundary_distance = std::numeric_limits<double>::infinity();
    for (const auto & p : footprint) {
      const auto dist = boost::geometry::distance(p, *target_boundary);
      if (dist < footprint_to_boundary_distance) {
        footprint_to_boundary_distance = dist;
      }
    }
    if (footprint_to_boundary_distance < min_distance_threshold) {
      return s;
    }
    if (footprint_to_boundary_distance < closest_dist) {
      closest_dist = footprint_to_boundary_distance;
      closest_s = s;
    }
  }

  return closest_s;
}

std::optional<lanelet::ConstLanelet> generatePathLanelet(
  const Trajectory & path, const double start_s, const double end_s, const double width,
  const double interval)
{
  lanelet::ConstPoints3d left_points{}, right_points{};

  for (const auto & s : path.base_arange({start_s, end_s}, interval)) {
    const auto p = path.compute(s).point.pose;
    const auto yaw = autoware_utils_geometry::get_rpy(p).z;
    const auto left_x = p.position.x + width / 2 * std::sin(yaw);
    const auto left_y = p.position.y - width / 2 * std::cos(yaw);
    const auto right_x = p.position.x - width / 2 * std::sin(yaw);
    const auto right_y = p.position.y + width / 2 * std::cos(yaw);
    left_points.emplace_back(lanelet::InvalId, left_x, left_y, p.position.z);
    right_points.emplace_back(lanelet::InvalId, right_x, right_y, p.position.z);
  }

  return autoware::experimental::lanelet2_utils::create_safe_lanelet(left_points, right_points);
}

}  // namespace autoware::behavior_velocity_planner::experimental::util
