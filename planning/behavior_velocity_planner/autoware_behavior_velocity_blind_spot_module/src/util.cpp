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

#include <autoware/behavior_velocity_blind_spot_module/util.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/topology.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <range/v3/all.hpp>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/length.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{

using autoware::experimental::lanelet2_utils::TurnDirection;

namespace
{
bool hasLaneIds(
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p, const lanelet::Id id)
{
  for (const auto & pid : p.lane_ids) {
    if (pid == id) {
      return true;
    }
  }
  return false;
}

std::optional<std::pair<size_t, size_t>> findLaneIdInterval(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & p, const lanelet::Id id)
{
  bool found = false;
  size_t start = 0;
  size_t end = p.points.size() > 0 ? p.points.size() - 1 : 0;
  if (start == end) {
    // there is only one point in the path
    return std::nullopt;
  }
  for (size_t i = 0; i < p.points.size(); ++i) {
    if (hasLaneIds(p.points.at(i), id)) {
      if (!found) {
        // found interval for the first time
        found = true;
        start = i;
      }
    } else if (found) {
      // prior point was in the interval. interval ended
      end = i;
      break;
    }
  }
  start = start > 0 ? start - 1 : 0;  // the idx of last point before the interval
  return found ? std::make_optional(std::make_pair(start, end)) : std::nullopt;
}

lanelet::Point3d remove_const(const lanelet::ConstPoint3d & point)
{
  return lanelet::Point3d{std::const_pointer_cast<lanelet::PointData>(point.constData())};
}

[[maybe_unused]] lanelet::LineString3d remove_const(const lanelet::ConstLineString3d & line)
{
  return lanelet::LineString3d{std::const_pointer_cast<lanelet::LineStringData>(line.constData())};
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

/**
 * @brief extend the last part of given `line` by some length
 */
lanelet::LineString3d generate_segment_beyond_linestring_end(
  const lanelet::ConstLineString3d & line, const double extend_length)
{
  const auto size = line.size();
  const auto & p1 = line[size - 2];
  const auto & p2 = line[size - 1];
  const auto p3 = autoware::experimental::lanelet2_utils::extrapolate_point(p1, p2, extend_length);
  lanelet::Points3d points;
  points.push_back(remove_const(p2));
  points.push_back(remove_const(p3));
  return lanelet::LineString3d{lanelet::InvalId, points};
};

template <typename L1, typename L2>
std::optional<Point2d> find_intersection_point(L1 && line1, L2 && line2)
{
  std::vector<Point2d> intersection_points;
  boost::geometry::intersection(
    std::forward<L1>(line1), std::forward<L2>(line2), intersection_points);
  if (intersection_points.empty()) {
    return std::nullopt;
  }
  return intersection_points.front();
}
}  // namespace

namespace helper
{

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
  const auto leftmost_road_lane = leftmost_ex ? leftmost_ex.value() : lanelet;
  if (const auto left_shoulder = route_handler->getLeftShoulderLanelet(leftmost_road_lane);
      left_shoulder) {
    return left_shoulder.value();
  }
  if (const auto left_bicycle_lane = route_handler->getLeftBicycleLanelet(leftmost_road_lane);
      left_bicycle_lane) {
    return left_bicycle_lane.value();
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
  const auto rightmost_road_lane = rightmost_ex ? rightmost_ex.value() : lanelet;
  if (const auto right_shoulder = route_handler->getRightShoulderLanelet(rightmost_road_lane);
      right_shoulder) {
    return right_shoulder.value();
  }
  if (const auto right_bicycle_lane = route_handler->getRightBicycleLanelet(rightmost_road_lane);
      right_bicycle_lane) {
    return right_bicycle_lane.value();
  }
  return rightmost_road_lane;
}

/**
 * @brief from `path`, generate a linestring with which ego footprint's left/right side sandwiches
 * the blind_spot. the output is trimmed at the entry of `intersection_lanelet`
 */
std::optional<lanelet::LineString3d> generate_blind_ego_side_path_boundary_before_turning(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::ConstLanelet & intersection_lanelet,
  const autoware::experimental::lanelet2_utils::TurnDirection & turn_direction,
  const double ego_width)
{
  lanelet::Points3d points;
  const auto lane_entry_line =
    lanelet::utils::to2D(get_entry_line(intersection_lanelet)).basicLineString();
  for (const auto & [p1, p2] :
       ranges::views::zip(path.points, path.points | ranges::views::drop(1))) {
    const auto path_segment = to_bg2d(std::vector{p1, p2});
    const auto intersection_point_opt = find_intersection_point(path_segment, lane_entry_line);
    if (intersection_point_opt) {
      const auto & point = intersection_point_opt.value();
      points.push_back(
        lanelet::Point3d{
          lanelet::InvalId, lanelet::BasicPoint3d{point.x(), point.y(), p1.point.pose.position.z}});
      break;
    }
    const auto sign = (turn_direction == TurnDirection::Left) ? 1.0 : -1.0;
    const double blind_side_direction =
      autoware_utils_geometry::get_rpy(p1.point.pose).z + sign * M_PI / 2.0;
    points.push_back(
      lanelet::Point3d{
        lanelet::InvalId,
        lanelet::BasicPoint3d{
          p1.point.pose.position.x + ego_width / 2.0 * std::cos(blind_side_direction),
          p1.point.pose.position.y + ego_width / 2.0 * std::sin(blind_side_direction),
          p1.point.pose.position.z}});
  }
  if (points.size() < 2) {
    return std::nullopt;
  }
  return std::make_optional<lanelet::LineString3d>(lanelet::InvalId, points);
}

/**
 * @brief get the sibling lanelet of `intersection_lanelet` whose turn_direction is straight
 */
std::optional<lanelet::ConstLanelet> sibling_straight_lanelet(
  const lanelet::ConstLanelet & intersection_lanelet,
  const lanelet::routing::RoutingGraphConstPtr routing_graph_ptr)
{
  for (const auto & sibling_lanelet : autoware::experimental::lanelet2_utils::sibling_lanelets(
         intersection_lanelet, routing_graph_ptr)) {
    if (autoware::experimental::lanelet2_utils::is_straight_direction(sibling_lanelet)) {
      return sibling_lanelet;
    }
  }
  return std::nullopt;
}

static std::optional<size_t> getDuplicatedPointIdx(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & point)
{
  for (size_t i = 0; i < path.points.size(); i++) {
    const auto & p = path.points.at(i).point.pose.position;

    constexpr double min_dist = 0.05;
    if (autoware_utils::calc_distance2d(p, point) < min_dist) {
      return i;
    }
  }

  return std::nullopt;
}

std::optional<size_t> insert_point_index(
  const geometry_msgs::msg::Pose & in_pose,
  autoware_internal_planning_msgs::msg::PathWithLaneId * inout_path,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold)
{
  const auto duplicate_idx_opt = getDuplicatedPointIdx(*inout_path, in_pose.position);
  if (duplicate_idx_opt) {
    return duplicate_idx_opt.value();
  }

  const size_t closest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    inout_path->points, in_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  // vector.insert(i) inserts element on the left side of v[i]
  // the velocity need to be zero order hold(from prior point)
  size_t insert_idx = closest_idx;
  autoware_internal_planning_msgs::msg::PathPointWithLaneId inserted_point =
    inout_path->points.at(closest_idx);
  if (planning_utils::isAheadOf(in_pose, inout_path->points.at(closest_idx).point.pose)) {
    ++insert_idx;
  } else {
    // copy with velocity from prior point
    const size_t prior_ind = closest_idx > 0 ? closest_idx - 1 : 0;
    inserted_point.point.longitudinal_velocity_mps =
      inout_path->points.at(prior_ind).point.longitudinal_velocity_mps;
  }
  inserted_point.point.pose = in_pose;

  auto it = inout_path->points.begin() + insert_idx;
  inout_path->points.insert(it, inserted_point);

  return insert_idx;
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

}  // namespace helper

std::optional<InterpolatedPathInfo> generateInterpolatedPathInfo(
  const lanelet::Id lane_id,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & input_path, rclcpp::Logger logger)
{
  constexpr double ds = 0.2;
  InterpolatedPathInfo interpolated_path_info;
  if (!splineInterpolate(input_path, ds, interpolated_path_info.path, logger)) {
    return std::nullopt;
  }
  interpolated_path_info.ds = ds;
  interpolated_path_info.lane_id = lane_id;
  const auto lane_id_interval = findLaneIdInterval(interpolated_path_info.path, lane_id);
  if (!lane_id_interval) {
    return std::nullopt;
  }
  interpolated_path_info.lane_id_interval = lane_id_interval.value();
  return interpolated_path_info;
}

std::vector<lanelet::Id> find_lane_ids_upto(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const lanelet::Id lane_id)
{
  std::vector<int64_t> lane_ids;
  /* get lane ids until intersection */
  for (const auto & point : path.points) {
    bool found_intersection_lane = false;
    for (const auto id : point.lane_ids) {
      if (id == lane_id) {
        found_intersection_lane = true;
        break;
      }
      // make lane_ids unique
      if (std::find(lane_ids.begin(), lane_ids.end(), id) == lane_ids.end()) {
        lane_ids.push_back(id);
      }
    }
    if (found_intersection_lane) break;
  }
  return lane_ids;
}

std::optional<size_t> get_first_index_intersects_line(
  const lanelet::ConstLineString2d & line, const InterpolatedPathInfo & interpolated_path_info,
  const autoware_utils::LinearRing2d & footprint, const double vehicle_length)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto [lane_start, lane_end] = interpolated_path_info.lane_id_interval;
  const size_t vehicle_length_idx = static_cast<size_t>(vehicle_length / interpolated_path_info.ds);
  const size_t start =
    static_cast<size_t>(std::max<int>(0, static_cast<int>(lane_start) - vehicle_length_idx));
  const auto line2d = line.basicLineString();
  for (auto i = start; i <= lane_end; ++i) {
    const auto & base_pose = path_ip.points.at(i).point.pose;
    const auto path_footprint =
      autoware_utils::transform_vector(footprint, autoware_utils::pose2transform(base_pose));
    if (boost::geometry::intersects(path_footprint, line2d)) {
      return std::make_optional<size_t>(i);
    }
  }
  return std::nullopt;
}

lanelet::ConstLineString3d get_entry_line(const lanelet::ConstLanelet & lanelet)
{
  return lanelet::ConstLineString3d{
    lanelet::InvalId,
    lanelet::Points3d{
      remove_const(lanelet.leftBound().front()), remove_const(lanelet.rightBound().front())}};
}

std::optional<lanelet::CompoundPolygon3d> generate_attention_area(
  const lanelet::ConstLanelet & road_lanelets_before_turning_merged,
  const lanelet::ConstLanelets & blind_side_lanelets_before_turning,
  const lanelet::ConstLineString3d & virtual_blind_side_boundary_after_turning,
  const lanelet::ConstLineString3d & virtual_ego_straight_path_after_turning,
  const lanelet::ConstLanelet & intersection_lanelet,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const autoware::experimental::lanelet2_utils::TurnDirection & turn_direction,
  const double ego_width)
{
  lanelet::Points3d attention_area_left_boundary;
  lanelet::Points3d attention_area_right_boundary;

  auto & far_side_boundary = (turn_direction == TurnDirection::Left)
                               ? attention_area_left_boundary
                               : attention_area_right_boundary;
  auto & near_side_boundary = (turn_direction == TurnDirection::Left)
                                ? attention_area_right_boundary
                                : attention_area_left_boundary;

  // far side bound
  const auto blind_side_lanelets_before_turning_merged =
    lanelet::utils::combineLaneletsShape(blind_side_lanelets_before_turning);
  const auto blind_side_lanelet_boundary_before_turning =
    (turn_direction == TurnDirection::Left)
      ? blind_side_lanelets_before_turning_merged.leftBound()
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
  const auto blind_ego_side_path_boundary_before_turning_opt =
    helper::generate_blind_ego_side_path_boundary_before_turning(
      path, intersection_lanelet, turn_direction, ego_width);
  if (!blind_ego_side_path_boundary_before_turning_opt) {
    return std::nullopt;
  }
  // NOTE: `backward_road_lane_offset_boundary` overlaps with
  // `blind_ego_side_path_boundary_before_turning`, so latter part of
  // `backward_road_lane_offset_boundary` is ignored
  const double sign = (turn_direction == TurnDirection::Left) ? 1.0 : -1.0;
  const auto backward_road_lane_offset_boundary = lanelet::utils::getCenterlineWithOffset(
    road_lanelets_before_turning_merged, sign * ego_width / 2.0, 3.0 /* [m] */);
  const auto & blind_ego_side_path_boundary_before_turning =
    blind_ego_side_path_boundary_before_turning_opt.value();
  for (const auto & point : backward_road_lane_offset_boundary) {
    if (
      lanelet::geometry::distance3d(point, blind_ego_side_path_boundary_before_turning.front()) <
      3.0) {
      // do not add anymore from this
      break;
    }
    near_side_boundary.push_back(remove_const(point));
  }
  for (const auto & point : blind_ego_side_path_boundary_before_turning) {
    near_side_boundary.push_back(remove_const(point));
  }
  for (const auto & point : virtual_ego_straight_path_after_turning) {
    near_side_boundary.push_back(remove_const(point));
  }
  if (near_side_boundary.size() < 2) {
    return std::nullopt;
  }
  const auto attention_lanelet = helper::generate_artificial_lanelet(
    attention_area_left_boundary, attention_area_right_boundary);
  return attention_lanelet.polygon3d();
}

std::optional<std::pair<lanelet::ConstLanelets, lanelet::ConstLanelets>>
generate_blind_side_lanelets_before_turning(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const autoware::experimental::lanelet2_utils::TurnDirection & turn_direction,
  const double backward_attention_length,
  [[maybe_unused]] const std::vector<lanelet::Id> & lane_ids_upto_intersection,
  const lanelet::Id intersection_lane_id)
{
  const auto & lanelet_map_ptr = route_handler->getLaneletMapPtr();
  const auto & routing_graph_ptr = route_handler->getRoutingGraphPtr();

  auto blind_side_getter_function = (turn_direction == TurnDirection::Left)
                                      ? helper::get_leftside_lanelet
                                      : helper::get_rightside_lanelet;
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
  const auto previous_lane_opt =
    helper::previous_lane_straight_priority(intersection_lane, routing_graph_ptr);
  if (previous_lane_opt) {
    const auto & previous_lane = previous_lane_opt.value();
    road_lanelets.push_back(previous_lane);
    blind_side_lanelets.push_back(blind_side_getter_function(route_handler, previous_lane));
    total_length += lanelet::utils::getLaneletLength3d(blind_side_lanelets.back());
  } else {
    return std::nullopt;
  }

  while (total_length < backward_attention_length) {
    const auto & last_road_lane = road_lanelets.front();
    const auto prev_lane_opt =
      helper::previous_lane_straight_priority(last_road_lane, routing_graph_ptr);
    if (!prev_lane_opt) {
      return std::make_pair(road_lanelets, blind_side_lanelets);
    }
    const auto & prev_lane = prev_lane_opt.value();
    road_lanelets.insert(road_lanelets.begin(), prev_lane);
    blind_side_lanelets.insert(
      blind_side_lanelets.begin(), blind_side_getter_function(route_handler, prev_lane));
    total_length += lanelet::utils::getLaneletLength3d(blind_side_lanelets.back());
  }
  return std::make_pair(road_lanelets, blind_side_lanelets);
}

lanelet::ConstLineString3d generate_virtual_blind_side_boundary_after_turning(
  const lanelet::ConstLanelet & outermost_lanelet,
  const autoware::experimental::lanelet2_utils::TurnDirection & turn_direction,
  const double extend_length)
{
  const auto & target_linestring = (turn_direction == TurnDirection::Left)
                                     ? outermost_lanelet.leftBound()
                                     : outermost_lanelet.rightBound();
  return generate_segment_beyond_linestring_end(target_linestring, extend_length);
}

std::optional<lanelet::LineString3d> generate_virtual_ego_straight_path_after_turning(
  const lanelet::ConstLanelet & intersection_lanelet,
  [[maybe_unused]] const lanelet::routing::RoutingGraphConstPtr routing_graph_ptr,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const autoware::experimental::lanelet2_utils::TurnDirection & turn_direction,
  const double ego_width)
{
  /*
  if (const auto sibling_straight_lanelet_opt =
        helper::sibling_straight_lanelet(intersection_lanelet, routing_graph_ptr);
      sibling_straight_lanelet_opt) {
    const auto & sibling_straight_lanelet = sibling_straight_lanelet_opt.value();
    const auto & target_linestring = (turn_direction == TurnDirection::Left)
                                       ? sibling_straight_lanelet.leftBound()
                                       : sibling_straight_lanelet.rightBound();
    return remove_const(target_linestring);
  }
  */
  const double extend_length = lanelet::utils::getLaneletLength3d(intersection_lanelet);

  const auto path_linestring = to_bg2d(path.points);
  const auto entry_line = get_entry_line(intersection_lanelet);
  const auto intersection_point_opt =
    find_intersection_point(path_linestring, lanelet::utils::to2D(entry_line).basicLineString());
  if (!intersection_point_opt) {
    return std::nullopt;
  }
  const auto & intersection_point = intersection_point_opt.value();
  const auto sign = (turn_direction == TurnDirection::Left) ? 1.0 : -1.0;
  const double width =
    boost::geometry::distance(entry_line.front().basicPoint2d(), intersection_point) -
    sign * ego_width / 2.0;
  const auto virtual_straight_path_start_opt =
    autoware::experimental::lanelet2_utils::interpolate_point(
      entry_line.front(), entry_line.back(), width);
  if (!virtual_straight_path_start_opt) {
    return std::nullopt;
  }
  const auto & virtual_straight_path_start = virtual_straight_path_start_opt.value();
  const Eigen::Vector3d virtual_straight_path_end =
    virtual_straight_path_start.basicPoint() +
    linestring_normal_direction(entry_line, extend_length);
  lanelet::Points3d points;
  points.push_back(lanelet::Point3d{lanelet::InvalId, virtual_straight_path_start});
  points.push_back(lanelet::Point3d{lanelet::InvalId, virtual_straight_path_end});
  return lanelet::LineString3d{lanelet::InvalId, points};
}

std::optional<lanelet::ConstLanelet> generate_ego_path_polygon(
  const InterpolatedPathInfo & interpolated_path_info, const double ego_width)
{
  const auto [start, end] = interpolated_path_info.lane_id_interval;
  lanelet::Points3d lefts;
  lanelet::Points3d rights;
  for (const auto & path_point_with_lane_id :
       interpolated_path_info.path.points | ranges::views::slice(start, end)) {
    const auto & pose = path_point_with_lane_id.point.pose;
    const auto & point = pose.position;
    const auto yaw = autoware_utils_geometry::get_rpy(pose).z;
    const auto left_dir = yaw + M_PI / 2.0;
    const auto right_dir = yaw - M_PI / 2.0;
    lefts.push_back(
      lanelet::Point3d{
        lanelet::InvalId, lanelet::BasicPoint3d{
                            point.x + std::cos(left_dir) * ego_width / 2.0,
                            point.y + std::sin(left_dir) * ego_width / 2.0, point.z}});
    rights.push_back(
      lanelet::Point3d{
        lanelet::InvalId, lanelet::BasicPoint3d{
                            point.x + std::cos(right_dir) * ego_width / 2.0,
                            point.y + std::sin(right_dir) * ego_width / 2.0, point.z}});
  }
  if (lefts.size() < 2 || rights.size() < 2) {
    return std::nullopt;
  }
  const lanelet::ConstLanelet path_polygon_lanelet =
    helper::generate_artificial_lanelet(lefts, rights);
  return std::make_optional<lanelet::ConstLanelet>(path_polygon_lanelet);
}

std::optional<StopPoints> generate_stop_points(
  const InterpolatedPathInfo & interpolated_path_info,
  const autoware_utils::LinearRing2d & footprint, const double ego_length,
  const lanelet::ConstLanelet & intersection_lanelet,
  const lanelet::ConstLineString3d & virtual_ego_straight_path_after_turning,
  const geometry_msgs::msg::Pose & current_pose, const double braking_distance,
  const double critical_stopline_margin, const double ego_nearest_dist_threshold,
  const double ego_nearest_yaw_threshold,
  autoware_internal_planning_msgs::msg::PathWithLaneId * path)
{
  const lanelet::ConstLineString3d traffic_light_stop_line{
    lanelet::InvalId, lanelet::Points3d{
                        remove_const(intersection_lanelet.leftBound().front()),
                        remove_const(intersection_lanelet.rightBound().front())}};
  const auto traffic_light_stop_line_2d =
    lanelet::utils::to2D(traffic_light_stop_line).basicLineString();
  const auto [start_lane, end] = interpolated_path_info.lane_id_interval;
  const size_t start = static_cast<size_t>(std::max<int>(
    0, static_cast<int>(start_lane) - std::ceil(ego_length / interpolated_path_info.ds)));

  std::optional<size_t> default_stopline_ip{};
  for (unsigned i = start; i <= end; ++i) {
    const auto & base_pose = interpolated_path_info.path.points.at(i).point.pose;
    const auto path_footprint =
      autoware_utils::transform_vector(footprint, autoware_utils::pose2transform(base_pose));
    const auto intersect_entry_line =
      boost::geometry::intersects(traffic_light_stop_line_2d, path_footprint);
    // already over the line, so skip
    if (i == start && intersect_entry_line) {
      break;
    }
    if (intersect_entry_line) {
      default_stopline_ip = i;
      break;
    }
  }

  const auto virtual_ego_straight_path_after_turning_2d =
    lanelet::utils::to2D(virtual_ego_straight_path_after_turning).basicLineString();
  std::optional<size_t> critical_stopline_ip{};
  const auto second_start = default_stopline_ip.value_or(start);
  for (unsigned i = second_start; i <= end; ++i) {
    const auto & base_pose = interpolated_path_info.path.points.at(i).point.pose;
    const auto path_footprint =
      autoware_utils::transform_vector(footprint, autoware_utils::pose2transform(base_pose));
    const auto intersect_line =
      boost::geometry::intersects(virtual_ego_straight_path_after_turning_2d, path_footprint);
    // already over the line, so skip
    if (i == second_start && intersect_line) {
      break;
    }
    if (intersect_line) {
      // subtract this position by the margin
      critical_stopline_ip = static_cast<size_t>(std::max<int>(
        0, static_cast<int>(i) - std::ceil(critical_stopline_margin / interpolated_path_info.ds)));
      break;
    }
  }

  if (!critical_stopline_ip) {
    return std::nullopt;
  }
  if (default_stopline_ip && default_stopline_ip.value() > critical_stopline_ip.value()) {
    // NOTE: default_stopline must be before critical_stopline
    return std::nullopt;
  }

  const auto closest_idx_ip = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    interpolated_path_info.path.points, current_pose, ego_nearest_dist_threshold,
    ego_nearest_yaw_threshold);
  const auto instant_stopline_ip = std::min(
    static_cast<size_t>(closest_idx_ip + std::ceil(braking_distance / interpolated_path_info.ds)),
    interpolated_path_info.path.points.size() - 1);

  // NOTE: since the order of three stopline varies, sort them in ascending order and insert
  // corresponding stopline from the head to tail
  struct StopPointsList
  {
    std::size_t default_stopline{};
    std::size_t instant_stopline{};
    std::size_t critical_stopline{};
  } stop_points_list;

  std::list<std::pair<const size_t *, size_t *>> stoplines;
  if (default_stopline_ip) {
    stoplines.emplace_back(
      std::make_pair(&default_stopline_ip.value(), &stop_points_list.default_stopline));
  }
  stoplines.emplace_back(std::make_pair(&instant_stopline_ip, &stop_points_list.instant_stopline));
  stoplines.emplace_back(
    std::make_pair(&critical_stopline_ip.value(), &stop_points_list.critical_stopline));

  // sort in ascending order
  stoplines.sort(
    [](const auto & it1, const auto & it2) { return *(std::get<0>(it1)) < *(std::get<0>(it2)); });

  for (const auto & [stop_idx_ip, stop_idx] : stoplines) {
    const auto & insert_pose = interpolated_path_info.path.points.at(*stop_idx_ip).point.pose;
    const auto inserted_idx = helper::insert_point_index(
      insert_pose, path, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
    if (!inserted_idx) {
      return std::nullopt;
    }
    *stop_idx = inserted_idx.value();
  }

  if (default_stopline_ip) {
    return StopPoints{
      stop_points_list.default_stopline, stop_points_list.instant_stopline,
      stop_points_list.critical_stopline};
  }
  return StopPoints{
    std::nullopt, stop_points_list.instant_stopline, stop_points_list.critical_stopline};
}

}  // namespace autoware::behavior_velocity_planner
