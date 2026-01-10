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

#include "util.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/topology.hpp>
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware/trajectory/utils/find_intervals.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <range/v3/all.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

using autoware::experimental::lanelet2_utils::from_ros;
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
namespace bg = boost::geometry;

namespace
{

lanelet::Point3d remove_const(const lanelet::ConstPoint3d & point)
{
  return lanelet::Point3d{std::const_pointer_cast<lanelet::PointData>(point.constData())};
}

/**
 * @brief return the normal direction of given `line`, multiplied by `length`
 */
Eigen::Vector3d linestring_normal_direction(
  const lanelet::ConstLineString3d & line, const double length)
{
  const auto & p0 = line.front();
  const auto & p1 = line.back();
  const double dx = p1.x() - p0.x();
  const double dy = p1.y() - p0.y();
  const double d = std::hypot(dx, dy);
  return {-dy / d * length, dx / d * length, 0.0};
}

lanelet::ConstLineString3d clip_virtual_line_to_intersection_bound(
  const lanelet::BasicPoint3d & virtual_line_start, const lanelet::BasicPoint3d & virtual_line_end,
  const lanelet::ConstLanelet & intersection_lanelet, const TurnDirection & turn_direction)
{
  const auto virtual_line =
    lanelet::utils::to2D(lanelet::BasicLineString3d{virtual_line_start, virtual_line_end});

  const auto & farthest_bound = (turn_direction == TurnDirection::Left)
                                  ? intersection_lanelet.rightBound().basicLineString()
                                  : intersection_lanelet.leftBound().basicLineString();

  std::vector<lanelet::BasicPoint2d> intersection_points;
  for (auto it = std::prev(farthest_bound.end()); it != farthest_bound.begin(); --it) {
    const auto & p1 = *std::prev(it);
    const auto & p2 = *it;
    const auto farthest_bound_segment = lanelet::utils::to2D(lanelet::BasicLineString3d{p1, p2});

    boost::geometry::intersection(virtual_line, farthest_bound_segment, intersection_points);
    if (!intersection_points.empty()) {
      break;
    }
  }

  lanelet::Points3d clipped_virtual_line_points{
    lanelet::Point3d{lanelet::InvalId, virtual_line_start}};

  if (intersection_points.empty()) {
    clipped_virtual_line_points.push_back(lanelet::Point3d{lanelet::InvalId, virtual_line_end});
  } else {
    clipped_virtual_line_points.push_back(
      lanelet::Point3d{
        lanelet::InvalId, intersection_points.front().x(), intersection_points.front().y(),
        virtual_line_end.z()});
  }

  return lanelet::ConstLineString3d{lanelet::InvalId, clipped_virtual_line_points};
}

/**
 * @brief extend the last part of given `line` by some length
 */
lanelet::ConstLineString3d generate_segment_beyond_linestring_end(
  const lanelet::ConstLineString3d & line, const lanelet::ConstLanelet & intersection_lanelet,
  const TurnDirection & turn_direction)
{
  const auto extend_length = lanelet::utils::getLaneletLength3d(intersection_lanelet);
  const auto & p1 = line[line.size() - 2];
  const auto & p2 = line[line.size() - 1];
  const auto p3 = autoware::experimental::lanelet2_utils::extrapolate_point(p1, p2, extend_length);
  return clip_virtual_line_to_intersection_bound(
    p2.basicPoint(), p3.basicPoint(), intersection_lanelet, turn_direction);
};

bool is_private_lanelet(const lanelet::ConstLanelet & lanelet)
{
  return std::strcmp(lanelet.attributeOr("location", "else"), "private") == 0;
}

/**
 * @brief obtain the lanelet if it has VRU lanelet on the leftside, otherwise return the leftmost
 * road lanelet. if `lanelet` is isolated, `lanelet` itself is returned
 */
lanelet::ConstLanelet get_leftside_lanelet(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const lanelet::ConstLanelet lanelet)
{
  const auto & routing_graph_ptr = route_handler->getRoutingGraphPtr();
  const auto leftmost_ex =
    autoware::experimental::lanelet2_utils::leftmost_lanelet(lanelet, routing_graph_ptr);
  const auto leftmost_road_lane = leftmost_ex ? *leftmost_ex : lanelet;
  if (const auto left_shoulder = route_handler->getLeftShoulderLanelet(leftmost_road_lane);
      left_shoulder) {
    return *left_shoulder;
  }
  if (const auto left_bicycle_lane = route_handler->getLeftBicycleLanelet(leftmost_road_lane);
      left_bicycle_lane) {
    return *left_bicycle_lane;
  }
  return leftmost_road_lane;
}

/**
 * @brief obtain the lanelet if it has VRU lanelet on the leftside, otherwise return the leftmost
 * road lanelet. if `lanelet` is isolated, `lanelet` itself is returned
 */
lanelet::ConstLanelet get_rightside_lanelet(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const lanelet::ConstLanelet lanelet)
{
  const auto & routing_graph_ptr = route_handler->getRoutingGraphPtr();
  const auto rightmost_ex =
    autoware::experimental::lanelet2_utils::rightmost_lanelet(lanelet, routing_graph_ptr);
  const auto rightmost_road_lane = rightmost_ex ? *rightmost_ex : lanelet;
  if (const auto right_shoulder = route_handler->getRightShoulderLanelet(rightmost_road_lane);
      right_shoulder) {
    return *right_shoulder;
  }
  if (const auto right_bicycle_lane = route_handler->getRightBicycleLanelet(rightmost_road_lane);
      right_bicycle_lane) {
    return *right_bicycle_lane;
  }
  return rightmost_road_lane;
}

/**
 * @brief from `path`, generate a linestring with which ego footprint's left/right side sandwiches
 * the blind_spot. the output is trimmed at the entry of `intersection_lanelet`
 */
std::optional<lanelet::ConstLineString3d> generate_blind_ego_side_path_boundary_before_turning(
  const Trajectory & path, const lanelet::ConstLanelet & intersection_lanelet,
  const TurnDirection & turn_direction, const double ego_width)
{
  const auto intersection =
    autoware::experimental::trajectory::crossed(path, get_entry_line(intersection_lanelet));
  lanelet::Points3d points{};

  for (const auto & s : path.get_underlying_bases()) {
    if (!intersection.empty() && s >= intersection.front()) {
      points.push_back(remove_const(from_ros(path.compute(intersection.front()).point.pose)));
      break;
    }

    const auto pose = path.compute(s).point.pose;
    const auto sign = (turn_direction == TurnDirection::Left) ? 1.0 : -1.0;
    const auto blind_side_direction = autoware_utils_geometry::get_rpy(pose).z + sign * M_PI / 2.0;
    points.emplace_back(
      lanelet::InvalId, from_ros(pose).basicPoint() +
                          ego_width / 2.0 *
                            lanelet::BasicPoint3d{
                              std::cos(blind_side_direction), std::sin(blind_side_direction), 0.0});
  }

  if (points.size() < 2) {
    return std::nullopt;
  }

  return lanelet::ConstLineString3d{lanelet::InvalId, points};
}

std::optional<lanelet::ConstLanelet> previous_lane_straight_priority(
  const lanelet::ConstLanelet & lane,
  const lanelet::routing::RoutingGraphConstPtr routing_graph_ptr)
{
  const auto prev_lanes =
    autoware::experimental::lanelet2_utils::previous_lanelets(lane, routing_graph_ptr);
  if (prev_lanes.empty()) {
    return std::nullopt;
  }
  for (const auto & prev_lane : prev_lanes) {
    if (autoware::experimental::lanelet2_utils::is_straight_direction(prev_lane)) {
      return prev_lane;
    }
  }
  return prev_lanes.front();
}

lanelet::ConstLanelet generate_artificial_lanelet(
  const lanelet::Points3d & left_points, const lanelet::Points3d & right_points)
{
  return lanelet::ConstLanelet{
    lanelet::InvalId, lanelet::LineString3d{lanelet::InvalId, left_points},
    lanelet::LineString3d{lanelet::InvalId, right_points}};
}
}  // namespace

std::vector<lanelet::Id> find_lane_ids_up_to(const Trajectory & path, const lanelet::Id lane_id)
{
  const auto contained_lane_ids = path.get_contained_lane_ids();
  std::vector<lanelet::Id> lane_ids;
  lane_ids.reserve(contained_lane_ids.size());
  for (const auto & id : contained_lane_ids) {
    if (id == lane_id) {
      break;
    }
    lane_ids.push_back(id);
  }
  return lane_ids;
}

lanelet::ConstLineString3d get_entry_line(const lanelet::ConstLanelet & lanelet)
{
  return lanelet::ConstLineString3d{
    lanelet::InvalId,
    {remove_const(lanelet.leftBound().front()), remove_const(lanelet.rightBound().front())}};
}

std::optional<lanelet::CompoundPolygon3d> generate_attention_area(
  const lanelet::ConstLanelet & road_lanelets_before_turning_merged,
  const lanelet::ConstLanelets & blind_side_lanelets_before_turning,
  const lanelet::ConstLineString3d & virtual_blind_side_boundary_after_turning,
  const lanelet::ConstLineString3d & virtual_ego_straight_path_after_turning,
  const lanelet::ConstLanelet & intersection_lanelet, const Trajectory & path,
  const TurnDirection & turn_direction, const double ego_width)
{
  const auto left_turn = turn_direction == TurnDirection::Left;

  lanelet::Points3d attention_area_left_boundary{};
  lanelet::Points3d attention_area_right_boundary{};

  auto & far_side_boundary =
    left_turn ? attention_area_left_boundary : attention_area_right_boundary;
  auto & near_side_boundary =
    left_turn ? attention_area_right_boundary : attention_area_left_boundary;

  // far side bound
  const auto blind_side_lanelets_before_turning_merged =
    lanelet::utils::combineLaneletsShape(blind_side_lanelets_before_turning);
  const auto blind_side_lanelet_boundary_before_turning =
    left_turn ? blind_side_lanelets_before_turning_merged.leftBound()
              : blind_side_lanelets_before_turning_merged.rightBound();

  for (const auto & point : blind_side_lanelet_boundary_before_turning) {
    far_side_boundary.push_back(remove_const(point));
  }
  for (const auto & point : virtual_blind_side_boundary_after_turning) {
    far_side_boundary.push_back(remove_const(point));
  }
  if (far_side_boundary.size() < 2) {
    return std::nullopt;
  }

  // near side bound
  const auto blind_ego_side_path_boundary_before_turning =
    generate_blind_ego_side_path_boundary_before_turning(
      path, intersection_lanelet, turn_direction, ego_width);
  if (!blind_ego_side_path_boundary_before_turning) {
    return std::nullopt;
  }

  // NOTE: `backward_road_lane_offset_boundary` overlaps with
  // `blind_ego_side_path_boundary_before_turning`, so latter part of
  // `backward_road_lane_offset_boundary` is ignored
  const auto sign = left_turn ? 1.0 : -1.0;
  const auto backward_road_lane_offset_boundary = lanelet::utils::getCenterlineWithOffset(
    road_lanelets_before_turning_merged, sign * ego_width / 2.0, 3.0 /* [m] */);

  for (const auto & point : backward_road_lane_offset_boundary) {
    if (
      lanelet::geometry::distance3d(point, blind_ego_side_path_boundary_before_turning->front()) <
      3.0) {
      // do not add anymore from this
      break;
    }
    near_side_boundary.push_back(remove_const(point));
  }

  for (const auto & point : *blind_ego_side_path_boundary_before_turning) {
    near_side_boundary.push_back(remove_const(point));
  }
  for (const auto & point : virtual_ego_straight_path_after_turning) {
    near_side_boundary.push_back(remove_const(point));
  }
  if (near_side_boundary.size() < 2) {
    return std::nullopt;
  }

  const auto attention_lanelet =
    generate_artificial_lanelet(attention_area_left_boundary, attention_area_right_boundary);
  return attention_lanelet.polygon3d();
}

std::optional<std::pair<lanelet::ConstLanelets, lanelet::ConstLanelets>>
generate_blind_side_lanelets_before_turning(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const TurnDirection & turn_direction, const double backward_attention_length,
  [[maybe_unused]] const std::vector<lanelet::Id> & lane_ids_upto_intersection,
  const lanelet::Id intersection_lane_id)
{
  const auto & lanelet_map_ptr = route_handler->getLaneletMapPtr();
  const auto & routing_graph_ptr = route_handler->getRoutingGraphPtr();

  auto blind_side_getter_function =
    (turn_direction == TurnDirection::Left) ? get_leftside_lanelet : get_rightside_lanelet;
  lanelet::ConstLanelets road_lanelets;
  lanelet::ConstLanelets blind_side_lanelets;
  double total_length = 0.0;
  /*
  // NOTE: if `lane_ids_upto_intersection is used, it will limit road_lanelets to route lanelet, and
  // if the route lanelet before `intersection_lane_id` was another intersection of left/right, this
  // function will fail to generate proper attention_area
  for (const auto lane_id_upto_intersection : lane_ids_upto_intersection | ranges::views::reverse) {
    const auto road_lane = lanelet_map_ptr->laneletLayer.get(lane_id_upto_intersection);
    road_lanelets.insert(road_lanelets.begin(), road_lane);
    blind_side_lanelets.insert(
      blind_side_lanelets.begin(), blind_side_getter_function(route_handler, road_lane));
    total_length += lanelet::utils::getLaneletLength3d(blind_side_lanelets.back());
    if (total_length >= backward_attention_length) {
      return std::make_pair(road_lanelets, blind_side_lanelets);
    }
  }
  */
  const auto intersection_lane = lanelet_map_ptr->laneletLayer.get(intersection_lane_id);

  const auto is_private_intersection = is_private_lanelet(intersection_lane);
  const auto validate_lane_attribute = [is_private_intersection](
                                         const lanelet::ConstLanelet & lanelet) {
    return is_private_intersection
             ? is_private_lanelet(lanelet)  // If intersection is private, lane must also be private
             : true;                        // ...otherwise, it's always true (allowed).
  };

  const auto previous_lane_opt =
    previous_lane_straight_priority(intersection_lane, routing_graph_ptr);
  if (previous_lane_opt && validate_lane_attribute(*previous_lane_opt)) {
    const auto & previous_lane = *previous_lane_opt;
    road_lanelets.push_back(previous_lane);
    blind_side_lanelets.push_back(blind_side_getter_function(route_handler, previous_lane));
    total_length += lanelet::utils::getLaneletLength3d(blind_side_lanelets.back());
  } else {
    return std::nullopt;
  }

  while (total_length < backward_attention_length) {
    const auto & last_road_lane = road_lanelets.front();
    const auto prev_lane_opt = previous_lane_straight_priority(last_road_lane, routing_graph_ptr);
    if (!prev_lane_opt || !validate_lane_attribute(*prev_lane_opt)) {
      return std::make_pair(road_lanelets, blind_side_lanelets);
    }
    const auto & prev_lane = *prev_lane_opt;
    road_lanelets.insert(road_lanelets.begin(), prev_lane);
    blind_side_lanelets.insert(
      blind_side_lanelets.begin(), blind_side_getter_function(route_handler, prev_lane));
    total_length += lanelet::utils::getLaneletLength3d(blind_side_lanelets.back());
  }
  return std::make_pair(road_lanelets, blind_side_lanelets);
}

lanelet::ConstLineString3d generate_virtual_blind_side_boundary_after_turning(
  const lanelet::ConstLanelet & outermost_lanelet,
  const lanelet::ConstLanelet & intersection_lanelet, const TurnDirection & turn_direction)
{
  const auto & target_linestring = (turn_direction == TurnDirection::Left)
                                     ? outermost_lanelet.leftBound()
                                     : outermost_lanelet.rightBound();
  return generate_segment_beyond_linestring_end(
    target_linestring, intersection_lanelet, turn_direction);
}

std::optional<lanelet::ConstLineString3d> generate_virtual_ego_straight_path_after_turning(
  const lanelet::ConstLanelet & intersection_lanelet, const Trajectory & path,
  const TurnDirection & turn_direction, const double ego_width)
{
  const auto entry_line = get_entry_line(intersection_lanelet);
  const auto intersections = autoware::experimental::trajectory::crossed(path, entry_line);
  if (intersections.empty()) {
    return std::nullopt;
  }
  const auto & intersection = intersections.front();

  const auto sign = (turn_direction == TurnDirection::Left) ? 1.0 : -1.0;
  const auto width = boost::geometry::distance(
                       entry_line.front().basicPoint2d(),
                       from_ros(path.compute(intersection).point.pose).basicPoint2d()) -
                     sign * ego_width / 2.0;
  const auto virtual_straight_path_start =
    autoware::experimental::lanelet2_utils::interpolate_point(
      entry_line.front(), entry_line.back(), width);
  if (!virtual_straight_path_start) {
    return std::nullopt;
  }

  const auto extend_length = lanelet::utils::getLaneletLength3d(intersection_lanelet);
  const Eigen::Vector3d virtual_straight_path_end =
    virtual_straight_path_start->basicPoint() +
    linestring_normal_direction(entry_line, extend_length);

  return clip_virtual_line_to_intersection_bound(
    *virtual_straight_path_start, virtual_straight_path_end, intersection_lanelet, turn_direction);
}

std::optional<lanelet::ConstLanelet> generate_ego_path_polygon(
  const Trajectory & path, const double ego_width)
{
  lanelet::Points3d lefts;
  lanelet::Points3d rights;
  static constexpr auto ds = 0.2;

  for (const auto s : path.base_arange(ds)) {
    const auto pose = path.compute(s).point.pose;
    const auto point = from_ros(pose.position).basicPoint();
    const auto yaw = autoware_utils_geometry::get_rpy(pose).z;
    const auto left_dir = yaw + M_PI / 2.0;
    const auto right_dir = yaw - M_PI / 2.0;
    lefts.emplace_back(
      lanelet::InvalId,
      point + ego_width / 2 * lanelet::BasicPoint3d{std::cos(left_dir), std::sin(left_dir), 0});
    rights.emplace_back(
      lanelet::InvalId,
      point + ego_width / 2 * lanelet::BasicPoint3d{std::cos(right_dir), std::sin(right_dir), 0});
  }
  if (lefts.size() < 2 || rights.size() < 2) {
    return std::nullopt;
  }

  return generate_artificial_lanelet(lefts, rights);
}

std::optional<StopLinePositions> generate_stop_points(
  const autoware_utils::LinearRing2d & footprint, const double ego_length, const double ego_s,
  const lanelet::ConstLanelet & intersection_lanelet,
  const lanelet::ConstLineString3d & virtual_ego_straight_path_after_turning,
  const double braking_distance, const double critical_stop_line_margin, Trajectory & path)
{
  StopLinePositions stop_line_positions;

  const lanelet::BasicLineString2d traffic_light_stop_line{
    intersection_lanelet.leftBound().front().basicPoint2d(),
    intersection_lanelet.rightBound().front().basicPoint2d()};

  const auto lane_id_intervals =
    autoware::experimental::trajectory::find_intervals(path, [&](const PathPointWithLaneId & p) {
      return std::find(p.lane_ids.begin(), p.lane_ids.end(), intersection_lanelet.id()) !=
             p.lane_ids.end();
    });
  if (lane_id_intervals.empty()) {
    return std::nullopt;
  }
  const auto [start_lane, end] = lane_id_intervals.front();
  const auto start = std::max(0., start_lane - ego_length);

  for (const auto & s : path.get_underlying_bases()) {
    if (s < start) {
      continue;
    }
    if (s > end) {
      break;
    }

    const auto path_footprint = autoware_utils::transform_vector(
      footprint, autoware_utils::pose2transform(path.compute(s).point.pose));
    const auto intersect_entry_line =
      boost::geometry::intersects(traffic_light_stop_line, path_footprint);

    if (intersect_entry_line) {
      if (s > start) {
        stop_line_positions.default_s = s;
      }
      break;
    }
  }

  const auto virtual_ego_straight_path_after_turning_2d =
    lanelet::utils::to2D(virtual_ego_straight_path_after_turning).basicLineString();
  const auto second_start = stop_line_positions.default_s.value_or(start);
  auto critical_stop_line_found = false;

  for (const auto & s : path.get_underlying_bases()) {
    if (s < second_start) {
      continue;
    }
    if (s > end) {
      break;
    }

    const auto path_footprint = autoware_utils::transform_vector(
      footprint, autoware_utils::pose2transform(path.compute(s).point.pose));
    const auto intersect_line =
      boost::geometry::intersects(virtual_ego_straight_path_after_turning_2d, path_footprint);

    if (intersect_line) {
      if (s > second_start) {
        // subtract this position by the margin
        stop_line_positions.critical_s = std::max(0., s - critical_stop_line_margin);
        critical_stop_line_found = true;
      }
      break;
    }
  }

  if (!critical_stop_line_found) {
    return std::nullopt;
  }
  if (
    stop_line_positions.default_s &&
    stop_line_positions.default_s > stop_line_positions.critical_s) {
    // NOTE: default_stop_line must be before critical_stop_line
    return std::nullopt;
  }

  stop_line_positions.instant_s = std::min(ego_s + braking_distance, path.length());

  return stop_line_positions;
}

std::optional<double> calc_ego_to_blind_spot_lanelet_lateral_gap(
  const autoware_utils::LinearRing2d & ego_footprint,
  const lanelet::ConstLanelets & last_lanelets_before_turning, const TurnDirection & turn_direction)
{
  const auto front_idx = (turn_direction == TurnDirection::Left)
                           ? vehicle_info_utils::VehicleInfo::FrontLeftIndex
                           : vehicle_info_utils::VehicleInfo::FrontRightIndex;
  const auto rear_idx = (turn_direction == TurnDirection::Left)
                          ? vehicle_info_utils::VehicleInfo::RearLeftIndex
                          : vehicle_info_utils::VehicleInfo::RearRightIndex;
  const auto ego_side =
    autoware_utils::Segment2d{ego_footprint[front_idx], ego_footprint[rear_idx]};

  std::vector<autoware_utils::Point2d> line;
  for (const auto & ll : last_lanelets_before_turning) {
    const auto & attention_area_road_boundary = lanelet::utils::to2D(
      (turn_direction == TurnDirection::Left) ? ll.leftBound() : ll.rightBound());
    const auto ll_2d = lanelet::utils::to2D(attention_area_road_boundary);

    for (const auto & ls : ll_2d) {
      line.emplace_back(ls.x(), ls.y());
    }
  }

  if (line.size() < 2) {
    return std::nullopt;
  }

  std::vector<autoware_utils::Segment2d> segments;
  segments.reserve(line.size() - 1);
  for (const auto & [p1, p2] : ranges::views::zip(line, line | ranges::views::drop(1))) {
    segments.emplace_back(p1, p2);
  }

  bg::index::rtree<autoware_utils::Segment2d, bg::index::rstar<16>> segments_before_turning{
    segments.begin(), segments.end()};

  std::vector<autoware_utils::Segment2d> candidate_segments;
  constexpr size_t max_candidate_size = 5;
  candidate_segments.reserve(max_candidate_size);
  segments_before_turning.query(
    bg::index::nearest(ego_side, max_candidate_size), std::back_inserter(candidate_segments));

  auto min_blind_side_distance = std::numeric_limits<double>::max();
  for (const auto & [candidate_segment, candidate_idx] : candidate_segments) {
    min_blind_side_distance =
      std::min(min_blind_side_distance, bg::distance(ego_side, candidate_segment));
  }

  return min_blind_side_distance;
}

}  // namespace autoware::behavior_velocity_planner::experimental
