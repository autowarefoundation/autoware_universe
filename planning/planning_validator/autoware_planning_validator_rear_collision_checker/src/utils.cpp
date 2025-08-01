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

#include "utils.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <magic_enum.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/difference.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/length.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::planning_validator::utils
{

namespace
{
std::optional<lanelet::ConstLanelet> get_sibling_straight_lanelet(
  const lanelet::ConstLanelet assigned_lane,
  const lanelet::routing::RoutingGraphConstPtr routing_graph_ptr)
{
  for (const auto & prev : routing_graph_ptr->previous(assigned_lane)) {
    for (const auto & following : routing_graph_ptr->following(prev)) {
      if (std::string(following.attributeOr("turn_direction", "else")) == "straight") {
        return following;
      }
    }
  }
  return std::nullopt;
}

std::pair<lanelet::BasicPoint2d, double> get_smallest_enclosing_circle(
  const lanelet::BasicPolygon2d & poly)
{
  // The `eps` is used to avoid precision bugs in circle inclusion checks.
  // If the value of `eps` is too small, this function doesn't work well. More than 1e-10 is
  // recommended.
  const double eps = 1e-5;
  lanelet::BasicPoint2d center(0.0, 0.0);
  double radius_squared = 0.0;

  auto cross = [](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> double {
    return p1.x() * p2.y() - p1.y() * p2.x();
  };

  auto make_circle_3 = [&](
                         const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2,
                         const lanelet::BasicPoint2d & p3) -> void {
    // reference for circumcenter vector https://en.wikipedia.org/wiki/Circumscribed_circle
    const double a = (p2 - p3).squaredNorm();
    const double b = (p3 - p1).squaredNorm();
    const double c = (p1 - p2).squaredNorm();
    const double s = cross(p2 - p1, p3 - p1);
    if (std::abs(s) < eps) return;
    center = (a * (b + c - a) * p1 + b * (c + a - b) * p2 + c * (a + b - c) * p3) / (4 * s * s);
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto make_circle_2 =
    [&](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> void {
    center = (p1 + p2) * 0.5;
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto in_circle = [&](const lanelet::BasicPoint2d & p) -> bool {
    return (center - p).squaredNorm() <= radius_squared;
  };

  // mini disc
  for (size_t i = 1; i < poly.size(); i++) {
    const auto p1 = poly[i];
    if (in_circle(p1)) continue;

    // mini disc with point
    const auto p0 = poly[0];
    make_circle_2(p0, p1);
    for (size_t j = 0; j < i; j++) {
      const auto p2 = poly[j];
      if (in_circle(p2)) continue;

      // mini disc with two points
      make_circle_2(p1, p2);
      for (size_t k = 0; k < j; k++) {
        const auto p3 = poly[k];
        if (in_circle(p3)) continue;

        // mini disc with tree points
        make_circle_3(p1, p2, p3);
      }
    }
  }

  return std::make_pair(center, radius_squared);
}

auto calc_lookahead_line(
  const std::shared_ptr<PlanningValidatorContext> & context, const double lookahead_time)
  -> std::optional<std::pair<autoware_utils::LineString3d, double>>
{
  const auto & points = context->data->current_trajectory->points;
  const auto & ego_pose = context->data->current_kinematics->pose.pose;
  const auto & current_velocity = context->data->current_kinematics->twist.twist.linear.x;
  const auto & vehicle_info = context->vehicle_info;

  const auto lookahead_distance = current_velocity * lookahead_time;
  const auto p_future = autoware::motion_utils::calcLongitudinalOffsetPose(
    points, ego_pose.position, vehicle_info.max_longitudinal_offset_m + lookahead_distance);

  if (!p_future.has_value()) {
    return std::nullopt;
  }

  autoware_utils::LineString3d lookahead_line;

  const auto p1 = autoware_utils::calc_offset_pose(
    p_future.value(), 0.0, 0.5 * vehicle_info.vehicle_width_m, 0.0);
  const auto p2 = autoware_utils::calc_offset_pose(
    p_future.value(), 0.0, -0.5 * vehicle_info.vehicle_width_m, 0.0);
  lookahead_line.emplace_back(p1.position.x, p1.position.y, ego_pose.position.z);
  lookahead_line.emplace_back(p2.position.x, p2.position.y, ego_pose.position.z);
  return std::make_pair(lookahead_line, lookahead_distance);
}

auto calc_predicted_stop_line(
  const std::shared_ptr<PlanningValidatorContext> & context, const double max_deceleration,
  const double max_positive_jerk, const double max_negative_jerk)
  -> std::optional<std::pair<autoware_utils::LineString3d, double>>
{
  const auto & points = context->data->current_trajectory->points;
  const auto & ego_pose = context->data->current_kinematics->pose.pose;
  const auto & current_velocity = context->data->current_kinematics->twist.twist.linear.x;
  const auto & current_acceleration = context->data->current_acceleration->accel.accel.linear.x;
  const auto & vehicle_info = context->vehicle_info;

  constexpr double target_velocity = 0.0;

  autoware_utils::LineString3d stop_line;

  const auto stop_distance = [&]() -> double {
    const auto opt_stop_distance = autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
      current_velocity, target_velocity, current_acceleration, max_deceleration, max_positive_jerk,
      max_negative_jerk);

    if (opt_stop_distance.has_value()) {
      return opt_stop_distance.value();
    }

    return 0.0;
  }();

  const auto p_future = autoware::motion_utils::calcLongitudinalOffsetPose(
    points, ego_pose.position, vehicle_info.max_longitudinal_offset_m + stop_distance);

  if (!p_future.has_value()) {
    return std::nullopt;
  }

  const auto p1 = autoware_utils::calc_offset_pose(
    p_future.value(), 0.0, 0.5 * vehicle_info.vehicle_width_m, 0.0);
  const auto p2 = autoware_utils::calc_offset_pose(
    p_future.value(), 0.0, -0.5 * vehicle_info.vehicle_width_m, 0.0);
  stop_line.emplace_back(p1.position.x, p1.position.y, ego_pose.position.z);
  stop_line.emplace_back(p2.position.x, p2.position.y, ego_pose.position.z);
  return std::make_pair(stop_line, stop_distance);
}
}  // namespace

auto check_shift_behavior(
  const lanelet::ConstLanelets & lanelets, const bool is_unsafe_holding,
  const std::shared_ptr<PlanningValidatorContext> & context,
  const rear_collision_checker_node::Params & parameters, DebugData & debug)
  -> std::pair<Behavior, double>
{
  const auto & points = context->data->current_trajectory->points;
  const auto & ego_pose = context->data->current_kinematics->pose.pose;
  const auto & vehicle_width = context->vehicle_info.vehicle_width_m;

  const auto combine_lanelet = lanelet::utils::combineLaneletsShape(lanelets);
  const auto nearest_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(points, ego_pose);
  {
    const auto p1 = autoware_utils::calc_offset_pose(ego_pose, 0.0, 0.5 * vehicle_width, 0.0);
    const auto p2 = autoware_utils::calc_offset_pose(ego_pose, 0.0, -0.5 * vehicle_width, 0.0);
    autoware_utils::LineString2d axle;
    axle.emplace_back(p1.position.x, p1.position.y);
    axle.emplace_back(p2.position.x, p2.position.y);

    const auto is_left_shift = boost::geometry::intersects(
      axle, lanelet::utils::to2D(combine_lanelet.leftBound()).basicLineString());
    if (is_left_shift) {
      return std::make_pair(Behavior::NONE, 0.0);
    }

    const auto is_right_shift = boost::geometry::intersects(
      axle, lanelet::utils::to2D(combine_lanelet.rightBound()).basicLineString());
    if (is_right_shift) {
      return std::make_pair(Behavior::NONE, 0.0);
    }
  }

  const auto constraints = parameters.common.ego;
  const auto reachable_point =
    calc_lookahead_line(context, parameters.common.adjacent_lane.lookahead_time);
  const auto stoppable_point = calc_predicted_stop_line(
    context, constraints.max_deceleration, constraints.max_positive_jerk,
    constraints.max_negative_jerk);
  if (!reachable_point.has_value() || !stoppable_point.has_value()) {
    return std::make_pair(Behavior::NONE, 0.0);
  }

  {
    debug.reachable_line = reachable_point.value().first;
    debug.stoppable_line = stoppable_point.value().first;
  }

  for (size_t i = 0; i < points.size(); i++) {
    const auto p1 = autoware_utils::calc_offset_pose(
      autoware_utils::get_pose(points.at(i)), 0.0, 0.5 * vehicle_width, 0.0);
    const auto p2 = autoware_utils::calc_offset_pose(
      autoware_utils::get_pose(points.at(i)), 0.0, -0.5 * vehicle_width, 0.0);
    autoware_utils::LineString2d axle;
    axle.emplace_back(p1.position.x, p1.position.y);
    axle.emplace_back(p2.position.x, p2.position.y);

    const auto distance = autoware::motion_utils::calcSignedArcLength(points, nearest_idx, i);
    if (reachable_point.value().second < distance && !is_unsafe_holding) {
      autoware_utils::LineString2d line_2d{
        reachable_point.value().first.front().to_2d(),
        reachable_point.value().first.back().to_2d()};
      if (boost::geometry::within(line_2d, combine_lanelet.polygon2d().basicPolygon())) {
        debug.text = "sufficient longitudinal distance before potential conflict.";
        break;
      }
    }

    if (stoppable_point.value().second > distance) {
      autoware_utils::LineString2d line_2d{
        stoppable_point.value().first.front().to_2d(),
        stoppable_point.value().first.back().to_2d()};
      if (!boost::geometry::within(line_2d, combine_lanelet.polygon2d().basicPolygon())) {
        debug.text = "unable to stop within current lane under limited braking.";
        break;
      }
    }

    const auto is_left_shift = boost::geometry::intersects(
      axle, lanelet::utils::to2D(combine_lanelet.leftBound()).basicLineString());
    if (is_left_shift && std::abs(points.at(i).longitudinal_velocity_mps) > 1e-3) {
      return std::make_pair((i < nearest_idx ? Behavior::NONE : Behavior::SHIFT_LEFT), distance);
    }

    const auto is_right_shift = boost::geometry::intersects(
      axle, lanelet::utils::to2D(combine_lanelet.rightBound()).basicLineString());
    if (is_right_shift && std::abs(points.at(i).longitudinal_velocity_mps) > 1e-3) {
      return std::make_pair((i < nearest_idx ? Behavior::NONE : Behavior::SHIFT_RIGHT), distance);
    }
  }

  return std::make_pair(Behavior::NONE, 0.0);
}

auto check_turn_behavior(
  const lanelet::ConstLanelets & lanelets, const bool is_unsafe_holding,
  const std::shared_ptr<PlanningValidatorContext> & context,
  const rear_collision_checker_node::Params & parameters, DebugData & debug)
  -> std::pair<Behavior, double>
{
  const auto & points = context->data->current_trajectory->points;
  const auto & ego_pose = context->data->current_kinematics->pose.pose;
  const auto & route_handler = context->data->route_handler;
  const auto & vehicle_info = context->vehicle_info;

  const auto ego_coordinate_on_arc = lanelet::utils::getArcCoordinates(lanelets, ego_pose);

  const auto distance_to_stop_point =
    autoware::motion_utils::calcDistanceToForwardStopPoint(points, ego_pose);

  constexpr double buffer = 0.5;

  const auto routing_graph_ptr = route_handler->getRoutingGraphPtr();

  const auto conflict_point = [&points, &vehicle_info](
                                const auto & sibling_straight_lanelet,
                                const auto is_right) -> std::optional<geometry_msgs::msg::Point> {
    const auto bound =
      is_right ? sibling_straight_lanelet.rightBound() : sibling_straight_lanelet.leftBound();
    for (const auto & p : points) {
      const auto p1 = autoware_utils::calc_offset_pose(
        autoware_utils::get_pose(p), 0.0, 0.5 * vehicle_info.vehicle_width_m, 0.0);
      const auto p2 = autoware_utils::calc_offset_pose(
        autoware_utils::get_pose(p), 0.0, -0.5 * vehicle_info.vehicle_width_m, 0.0);
      autoware_utils::LineString2d axle;
      axle.emplace_back(p1.position.x, p1.position.y);
      axle.emplace_back(p2.position.x, p2.position.y);

      if (boost::geometry::intersects(axle, lanelet::utils::to2D(bound.basicLineString()))) {
        return autoware_utils::get_point(p);
      }
    }

    return std::nullopt;
  };

  const auto & p = parameters.common.blind_spot;
  const auto constraints = parameters.common.ego;
  const auto reachable_point =
    calc_lookahead_line(context, parameters.common.blind_spot.lookahead_time);
  const auto stoppable_point = calc_predicted_stop_line(
    context, constraints.max_deceleration, constraints.max_positive_jerk,
    constraints.max_negative_jerk);
  if (!reachable_point.has_value() || !stoppable_point.has_value()) {
    return std::make_pair(Behavior::NONE, 0.0);
  }

  {
    debug.reachable_line = reachable_point.value().first;
    debug.stoppable_line = stoppable_point.value().first;
  }

  const auto exceed_dead_line =
    [&points, &ego_pose, &conflict_point, &stoppable_point, &vehicle_info](
      const auto & sibling_straight_lanelet, const auto distance, const auto is_right) {
      if (!sibling_straight_lanelet.has_value()) {
        return distance < stoppable_point.value().second;
      }

      const auto dead_line_point = conflict_point(sibling_straight_lanelet.value(), is_right);
      if (!dead_line_point.has_value()) {
        return distance < stoppable_point.value().second;
      }

      return autoware::motion_utils::calcSignedArcLength(
               points, ego_pose.position, dead_line_point.value()) -
               vehicle_info.max_longitudinal_offset_m <
             stoppable_point.value().second;
    };

  double total_length = 0.0;
  for (const auto & lane : lanelets) {
    const auto distance =
      total_length - ego_coordinate_on_arc.length - vehicle_info.max_longitudinal_offset_m;
    const std::string turn_direction = lane.attributeOr("turn_direction", "none");

    total_length += lanelet::utils::getLaneletLength2d(lane);

    const auto is_reachable = distance < reachable_point.value().second || is_unsafe_holding;

    if (turn_direction == "left" && p.check.left) {
      const auto sibling_straight_lanelet = get_sibling_straight_lanelet(lane, routing_graph_ptr);

      if (exceed_dead_line(sibling_straight_lanelet, distance, false)) {
        debug.text = "unable to stop before the conflict area under limited braking.";
        continue;
      }

      if (!distance_to_stop_point.has_value()) {
        return std::make_pair((is_reachable ? Behavior::TURN_LEFT : Behavior::NONE), distance);
      }
      if (distance_to_stop_point.value() < distance + buffer) {
        return std::make_pair(Behavior::NONE, distance);
      }
      return std::make_pair((is_reachable ? Behavior::TURN_LEFT : Behavior::NONE), distance);
    }

    if (turn_direction == "right" && p.check.right) {
      const auto sibling_straight_lanelet = get_sibling_straight_lanelet(lane, routing_graph_ptr);

      if (exceed_dead_line(sibling_straight_lanelet, distance, true)) {
        debug.text = "unable to stop before the conflict area under limited braking.";
        continue;
      }

      if (!distance_to_stop_point.has_value()) {
        return std::make_pair((is_reachable ? Behavior::TURN_RIGHT : Behavior::NONE), distance);
      }
      if (distance_to_stop_point.value() < distance + buffer) {
        return std::make_pair(Behavior::NONE, distance);
      }
      return std::make_pair((is_reachable ? Behavior::TURN_RIGHT : Behavior::NONE), distance);
    }
  }

  return std::make_pair(Behavior::NONE, 0.0);
}

void cut_by_lanelets(const lanelet::ConstLanelets & lanelets, DetectionAreas & detection_areas)
{
  const auto combine_lanelet = lanelet::utils::combineLaneletsShape(lanelets);

  for (auto & [original, _] : detection_areas) {
    if (original.empty()) {
      continue;
    }

    lanelet::BasicPolygons2d polygons2d;
    boost::geometry::difference(
      lanelet::utils::to2D(original), combine_lanelet.polygon2d().basicPolygon(), polygons2d);

    if (polygons2d.empty()) {
      continue;
    }

    lanelet::BasicPolygon3d polygon3d;
    for (const auto & p : polygons2d.front()) {
      polygon3d.push_back(lanelet::BasicPoint3d(p.x(), p.y(), original.front().z()));
    }

    original = polygon3d;
  }
}

void fill_rss_distance(
  PointCloudObjects & objects, const std::shared_ptr<PlanningValidatorContext> & context,
  [[maybe_unused]] const double distance_to_conflict_point, const double reaction_time,
  const double max_deceleration, const double max_velocity,
  const rear_collision_checker_node::Params & parameters)
{
  const auto & p = parameters;
  const auto & max_deceleration_ego = p.common.ego.max_deceleration;
  const auto & current_velocity = context->data->current_kinematics->twist.twist.linear.x;

  for (auto & object : objects) {
    const auto stop_distance_object =
      reaction_time * object.velocity +
      0.5 * std::pow(object.velocity, 2.0) / std::abs(max_deceleration);
    const auto stop_distance_ego =
      0.5 * std::pow(current_velocity, 2.0) / std::abs(max_deceleration_ego);

    object.rss_distance = stop_distance_object - stop_distance_ego;
    object.safe = object.rss_distance < object.relative_distance_with_delay_compensation;
    object.ignore =
      object.moving_time < p.common.filter.moving_time || object.velocity > max_velocity;
  }
}

void fill_time_to_collision(
  PointCloudObjects & objects, const std::shared_ptr<PlanningValidatorContext> & context,
  const double distance_to_conflict_point, [[maybe_unused]] const double reaction_time,
  [[maybe_unused]] const double max_deceleration, const double max_velocity,
  const rear_collision_checker_node::Params & parameters)
{
  const auto & p = parameters;
  const auto & current_velocity = context->data->current_kinematics->twist.twist.linear.x;

  for (auto & object : objects) {
    const auto time_to_reach_ego =
      distance_to_conflict_point / std::max(current_velocity, p.common.ego.min_velocity);
    const auto time_to_reach_obj =
      (distance_to_conflict_point + object.relative_distance_with_delay_compensation) /
      std::max(object.velocity, p.common.filter.min_velocity);
    object.time_to_collision = std::abs(time_to_reach_ego - time_to_reach_obj);
    object.safe = object.time_to_collision > p.common.time_to_collision.margin;
    object.ignore =
      object.moving_time < p.common.filter.moving_time || object.velocity > max_velocity;
  }
}

auto get_current_lanes(
  const std::shared_ptr<PlanningValidatorContext> & context, const double forward_distance,
  const double backward_distance) -> lanelet::ConstLanelets
{
  const auto & ego_pose = context->data->current_kinematics->pose.pose;
  const auto & route_handler = context->data->route_handler;

  lanelet::ConstLanelet closest_lanelet;
  if (!route_handler->getClosestLaneletWithinRoute(ego_pose, &closest_lanelet)) {
    return {};
  }

  return route_handler->getLaneletSequence(
    closest_lanelet, ego_pose, backward_distance, forward_distance);
}

auto get_previous_polygons_with_lane_recursively(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
  const double s1, const double s2,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const double left_offset, const double right_offset) -> DetectionAreas
{
  DetectionAreas ret{};

  if (target_lanes.empty()) {
    return ret;
  }

  if (route_handler->getPreviousLanelets(target_lanes.front()).empty()) {
    const auto total_length = lanelet::utils::getLaneletLength2d(target_lanes);
    const auto expand_lanelets =
      lanelet::utils::getExpandedLanelets(target_lanes, left_offset, -1.0 * right_offset);
    const auto polygon = lanelet::utils::getPolygonFromArcLength(
      expand_lanelets, total_length - s2, total_length - s1);
    ret.emplace_back(polygon.basicPolygon(), target_lanes);
    return ret;
  }

  for (const auto & prev_lane : route_handler->getPreviousLanelets(target_lanes.front())) {
    {
      const auto overlap_current_lanes = std::any_of(
        current_lanes.begin(), current_lanes.end(),
        [&prev_lane](const auto & lane) { return lane.id() == prev_lane.id(); });
      const auto total_length = lanelet::utils::getLaneletLength2d(target_lanes);
      if (overlap_current_lanes) {
        const auto expand_lanelets =
          lanelet::utils::getExpandedLanelets(target_lanes, left_offset, -1.0 * right_offset);
        const auto polygon = lanelet::utils::getPolygonFromArcLength(
          expand_lanelets, total_length - s2, total_length - s1);
        ret.emplace_back(polygon.basicPolygon(), target_lanes);

        continue;
      }
    }

    lanelet::ConstLanelets pushed_lanes = target_lanes;
    pushed_lanes.insert(pushed_lanes.begin(), prev_lane);

    {
      const auto total_length = lanelet::utils::getLaneletLength2d(pushed_lanes);
      if (total_length > s2) {
        const auto expand_lanelets =
          lanelet::utils::getExpandedLanelets(pushed_lanes, left_offset, -1.0 * right_offset);
        const auto polygon = lanelet::utils::getPolygonFromArcLength(
          expand_lanelets, total_length - s2, total_length - s1);
        ret.emplace_back(polygon.basicPolygon(), pushed_lanes);
      } else {
        const auto polygons = get_previous_polygons_with_lane_recursively(
          current_lanes, pushed_lanes, s1, s2, route_handler, left_offset, right_offset);
        ret.insert(ret.end(), polygons.begin(), polygons.end());
      }
    }
  }

  return ret;
}

auto get_obstacle_points(const lanelet::BasicPolygons3d & polygons, const PointCloud & points)
  -> PointCloud::Ptr
{
  PointCloud ret;
  for (const auto & polygon : polygons) {
    const auto circle = get_smallest_enclosing_circle(lanelet::utils::to2D(polygon));
    for (const auto & p : points) {
      const double squared_dist = (circle.first.x() - p.x) * (circle.first.x() - p.x) +
                                  (circle.first.y() - p.y) * (circle.first.y() - p.y);
      if (squared_dist > circle.second) {
        continue;
      }
      if (boost::geometry::within(
            autoware_utils::Point2d{p.x, p.y}, lanelet::utils::to2D(polygon))) {
        ret.push_back(p);
      }
    }
  }
  return std::make_shared<PointCloud>(ret);
}

auto generate_detection_polygon(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & ego_pose,
  const double forward_distance, const double backward_distance) -> lanelet::BasicPolygon3d
{
  const auto ego_coordinate_on_arc = lanelet::utils::getArcCoordinates(lanelets, ego_pose).length;
  const auto polygon = lanelet::utils::getPolygonFromArcLength(
    lanelets, ego_coordinate_on_arc - backward_distance, ego_coordinate_on_arc + forward_distance);
  return polygon.basicPolygon();
}

auto generate_half_lanelet(
  const lanelet::ConstLanelet lanelet, const bool is_right,
  const double ignore_width_from_centerline, const double expand_width_from_bound)
  -> lanelet::ConstLanelet
{
  lanelet::Points3d lefts, rights;

  const double offset = !is_right ? ignore_width_from_centerline : -ignore_width_from_centerline;
  const auto offset_centerline = lanelet::utils::getCenterlineWithOffset(lanelet, offset);

  const auto original_left_bound =
    !is_right ? lanelet::utils::getLeftBoundWithOffset(lanelet, expand_width_from_bound)
              : offset_centerline;
  const auto original_right_bound =
    !is_right ? offset_centerline
              : lanelet::utils::getRightBoundWithOffset(lanelet, expand_width_from_bound);

  for (const auto & pt : original_left_bound) {
    lefts.emplace_back(pt);
  }
  for (const auto & pt : original_right_bound) {
    rights.emplace_back(pt);
  }
  const auto left_bound = lanelet::LineString3d(lanelet::InvalId, std::move(lefts));
  const auto right_bound = lanelet::LineString3d(lanelet::InvalId, std::move(rights));
  auto half_lanelet = lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
  return half_lanelet;
}

auto get_range_for_rss(
  const std::shared_ptr<PlanningValidatorContext> & context,
  [[maybe_unused]] const double distance_to_conflict_point, const double reaction_time,
  const double max_deceleration, const double max_velocity,
  const rear_collision_checker_node::Params & parameters) -> std::pair<double, double>
{
  const auto & p = parameters;
  const auto & max_deceleration_ego = p.common.ego.max_deceleration;
  const auto & current_velocity = context->data->current_kinematics->twist.twist.linear.x;

  const auto stop_distance_object =
    reaction_time * max_velocity + 0.5 * std::pow(max_velocity, 2.0) / std::abs(max_deceleration);
  const auto stop_distance_ego =
    0.5 * std::pow(current_velocity, 2.0) / std::abs(max_deceleration_ego);

  const auto forward_distance = p.common.pointcloud.range.buffer +
                                std::max(
                                  context->vehicle_info.max_longitudinal_offset_m,
                                  (p.common.blind_spot.check.front ? stop_distance_ego : 0.0));
  const auto backward_distance = p.common.pointcloud.range.buffer -
                                 context->vehicle_info.min_longitudinal_offset_m +
                                 std::max(0.0, stop_distance_object - stop_distance_ego);

  return std::make_pair(forward_distance, backward_distance);
}

auto get_range_for_ttc(
  const std::shared_ptr<PlanningValidatorContext> & context,
  const double distance_to_conflict_point, [[maybe_unused]] const double reaction_time,
  [[maybe_unused]] const double max_deceleration, const double max_velocity,
  const rear_collision_checker_node::Params & parameters) -> std::pair<double, double>
{
  const auto & p = parameters;
  const auto & current_velocity = context->data->current_kinematics->twist.twist.linear.x;

  const auto time_to_reach_ego =
    distance_to_conflict_point / std::max(current_velocity, p.common.ego.min_velocity);

  const auto forward_distance =
    p.common.pointcloud.range.buffer + context->vehicle_info.max_longitudinal_offset_m;
  const auto backward_distance =
    p.common.pointcloud.range.buffer +
    (time_to_reach_ego + p.common.time_to_collision.margin) * max_velocity -
    distance_to_conflict_point;

  return std::make_pair(forward_distance, backward_distance);
}

auto create_polygon_marker_array(
  const std::vector<autoware_utils::Polygon3d> & polygons, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color) -> MarkerArray
{
  MarkerArray msg;

  size_t i = 0;
  for (const auto & polygon : polygons) {
    auto marker = autoware_utils::create_default_marker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i++, Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(0.05, 0.0, 0.0), color);

    for (const auto & p : polygon.outer()) {
      marker.points.push_back(autoware_utils::create_point(p.x(), p.y(), p.z()));
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

auto create_polygon_marker_array(
  const lanelet::BasicPolygons3d & polygons, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color) -> MarkerArray
{
  MarkerArray msg;

  size_t i = 0;
  for (const auto & polygon : polygons) {
    auto marker = autoware_utils::create_default_marker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i++, Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(0.2, 0.0, 0.0), color);

    for (const auto & p : polygon) {
      marker.points.push_back(autoware_utils::create_point(p.x(), p.y(), p.z()));
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

auto create_pointcloud_object_marker_array(
  const PointCloudObjects & objects, const std::string & ns,
  const rear_collision_checker_node::Params & parameters) -> MarkerArray
{
  MarkerArray msg;

  size_t i = 0L;
  for (const auto & object : objects) {
    const auto sphere_color = [&object, &parameters]() {
      if (
        object.tracking_duration <
        parameters.common.pointcloud.velocity_estimation.observation_time) {
        return autoware_utils::create_marker_color(1.0, 1.0, 1.0, 0.999);
      }
      if (
        object.velocity > parameters.common.filter.min_velocity &&
        object.moving_time < parameters.common.filter.moving_time) {
        return autoware_utils::create_marker_color(1.0, 0.67, 0.0, 0.999);
      }
      if (object.ignore) {
        return autoware_utils::create_marker_color(0.5, 0.5, 0.5, 0.999);
      }
      return object.safe ? autoware_utils::create_marker_color(0.16, 1.0, 0.69, 0.999)
                         : autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.999);
    }();

    {
      auto marker = autoware_utils::create_default_marker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns + "_sphere", i++, Marker::SPHERE,
        autoware_utils::create_marker_scale(1.0, 1.0, 1.0), sphere_color);
      marker.pose = object.pose;
      msg.markers.push_back(marker);
    }

    {
      const auto x_scale = object.velocity * 0.36;
      auto marker = autoware_utils::create_default_marker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns + "_arrow", i++, Marker::ARROW,
        autoware_utils::create_marker_scale(x_scale, 0.3, 0.3), sphere_color);
      marker.pose = object.pose;
      msg.markers.push_back(marker);
    }

    {
      auto marker = autoware_utils::create_default_marker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns + "_text", i++, Marker::TEXT_VIEW_FACING,
        autoware_utils::create_marker_scale(0.5, 0.5, 0.5),
        autoware_utils::create_marker_color(1.0, 1.0, 1.0, 0.999));

      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2);
      ss << "TrackingDuration:" << object.tracking_duration
         << "[s]\nMovingTime:" << object.moving_time
         << "[s]\nRelativeDistance(RAW):" << object.relative_distance
         << "[m]\nRelativeDistance(w/DC):" << object.relative_distance_with_delay_compensation
         << "[m]\nVelocity:" << object.velocity << "[m/s]\nRSSDistance" << object.rss_distance
         << "[m]\nTimeToCollision:" << object.time_to_collision
         << "[s]\nFurthestLaneID:" << object.furthest_lane.id() << "\nDetail:" << object.detail;

      marker.text = ss.str();
      marker.pose = object.pose;
      marker.pose.position.z += 1.0;
      msg.markers.push_back(marker);
    }
  }

  return msg;
}

auto create_line_marker_array(
  const autoware_utils::LineString3d & line, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color) -> MarkerArray
{
  MarkerArray msg;

  if (line.size() < 2) {
    return msg;
  }

  auto marker = autoware_utils::create_default_marker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::LINE_STRIP,
    autoware_utils::create_marker_scale(0.3, 0.1, 0.1), color);
  marker.points.push_back(autoware_utils::to_msg(line.front()));
  marker.points.push_back(autoware_utils::to_msg(line.back()));

  msg.markers.push_back(marker);
  return msg;
}
}  // namespace autoware::planning_validator::utils
