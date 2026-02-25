// Copyright 2026 TIER IV, Inc.
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

#include "path_planner.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/interpolator/linear.hpp>
#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/trajectory/utils/crop.hpp>
#include <autoware/trajectory/utils/find_intervals.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

// ===========================================================================
// PathPlanner class implementation
// ===========================================================================

PathPlanner::PathPlanner(
  const rclcpp::Logger & logger, rclcpp::Clock::SharedPtr clock,
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper, const Params & params,
  const VehicleInfo & vehicle_info)
: logger_(logger),
  clock_(std::move(clock)),
  time_keeper_(std::move(time_keeper)),
  params_(params),
  vehicle_info_(vehicle_info)
{
}

void PathPlanner::set_planner_data(
  const LaneletMapBin::ConstSharedPtr & lanelet_map_bin_ptr,
  const LaneletRoute::ConstSharedPtr & route_ptr)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  if (lanelet_map_bin_ptr && !route_context_.lanelet_map_ptr) {
    route_context_.lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(
      *lanelet_map_bin_ptr, route_context_.lanelet_map_ptr, &route_context_.traffic_rules_ptr,
      &route_context_.routing_graph_ptr);
  }

  if (route_ptr) {
    set_route(route_ptr);
  }
}

void PathPlanner::set_route(const LaneletRoute::ConstSharedPtr & route_ptr)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  route_context_.route_frame_id = route_ptr->header.frame_id;
  route_context_.goal_pose = route_ptr->goal_pose;

  route_context_.route_lanelets.clear();
  route_context_.preferred_lanelets.clear();
  route_context_.start_lanelets.clear();
  route_context_.goal_lanelets.clear();

  size_t primitives_num = 0;
  for (const auto & route_section : route_ptr->segments) {
    primitives_num += route_section.primitives.size();
  }
  route_context_.route_lanelets.reserve(primitives_num);

  for (const auto & route_section : route_ptr->segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      const auto & lanelet = route_context_.lanelet_map_ptr->laneletLayer.get(id);
      route_context_.route_lanelets.push_back(lanelet);
      if (id == route_section.preferred_primitive.id) {
        route_context_.preferred_lanelets.push_back(lanelet);
      }
    }
  }

  const auto set_lanelets_from_segment =
    [&](
      const autoware_planning_msgs::msg::LaneletSegment & segment,
      lanelet::ConstLanelets & lanelets) {
      lanelets.reserve(segment.primitives.size());
      for (const auto & primitive : segment.primitives) {
        const auto & lanelet = route_context_.lanelet_map_ptr->laneletLayer.get(primitive.id);
        lanelets.push_back(lanelet);
      }
    };
  set_lanelets_from_segment(route_ptr->segments.front(), route_context_.start_lanelets);
  set_lanelets_from_segment(route_ptr->segments.back(), route_context_.goal_lanelets);
}

void PathPlanner::set_route_context(const RouteContext & route_context)
{
  route_context_ = route_context;
}

RouteContext & PathPlanner::route_context()
{
  return route_context_;
}

const RouteContext & PathPlanner::route_context() const
{
  return route_context_;
}

void PathPlanner::update_params(const Params & params)
{
  params_ = params;
}

Trajectory PathPlanner::convert_path_to_trajectory(
  const PathWithLaneId & path, double resample_interval)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> traj_points;
  traj_points.reserve(path.points.size());
  for (const auto & path_point : path.points) {
    autoware_planning_msgs::msg::TrajectoryPoint traj_point;
    traj_point.pose = path_point.point.pose;
    traj_point.longitudinal_velocity_mps = path_point.point.longitudinal_velocity_mps;
    traj_point.lateral_velocity_mps = path_point.point.lateral_velocity_mps;
    traj_point.heading_rate_rps = path_point.point.heading_rate_rps;
    traj_points.push_back(traj_point);
  }

  const auto traj_msg = autoware::motion_utils::convertToTrajectory(traj_points, path.header);
  return autoware::motion_utils::resampleTrajectory(
    traj_msg, resample_interval, false, true, true, true);
}

// ===========================================================================
// Utility functions
// ===========================================================================
namespace utils
{
namespace
{
template <typename T>
bool exists(const std::vector<T> & vec, const T & item)
{
  return std::find(vec.begin(), vec.end(), item) != vec.end();
}

template <typename const_iterator>
std::vector<geometry_msgs::msg::Point> to_geometry_msgs_points(
  const const_iterator begin, const const_iterator end)
{
  std::vector<geometry_msgs::msg::Point> geometry_msgs_points{};
  geometry_msgs_points.reserve(std::distance(begin, end));
  std::transform(begin, end, std::back_inserter(geometry_msgs_points), [](const auto & point) {
    return lanelet::utils::conversion::toGeomMsgPt(point);
  });
  return geometry_msgs_points;
}

lanelet::BasicPoints3d to_lanelet_points(
  const std::vector<geometry_msgs::msg::Point> & geometry_msgs_points)
{
  lanelet::BasicPoints3d lanelet_points{};
  lanelet_points.reserve(geometry_msgs_points.size());
  std::transform(
    geometry_msgs_points.begin(), geometry_msgs_points.end(), std::back_inserter(lanelet_points),
    [](const auto & point) { return lanelet::utils::conversion::toLaneletPoint(point); });
  return lanelet_points;
}
}  // namespace

std::optional<lanelet::ConstLanelets> get_lanelets_within_route_up_to(
  const lanelet::ConstLanelet & lanelet, const RouteContext & planner_data, const double distance)
{
  if (!exists(planner_data.route_lanelets, lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelets{};
  auto current_lanelet = lanelet;
  auto length = 0.;

  while (length < distance) {
    const auto prev_lanelet = get_previous_lanelet_within_route(current_lanelet, planner_data);
    if (!prev_lanelet) {
      break;
    }

    lanelets.push_back(*prev_lanelet);
    current_lanelet = *prev_lanelet;
    length += lanelet::utils::getLaneletLength2d(*prev_lanelet);
  }

  std::reverse(lanelets.begin(), lanelets.end());
  return lanelets;
}

std::optional<lanelet::ConstLanelets> get_lanelets_within_route_after(
  const lanelet::ConstLanelet & lanelet, const RouteContext & planner_data, const double distance)
{
  if (!exists(planner_data.route_lanelets, lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelets{};
  auto current_lanelet = lanelet;
  auto length = 0.;

  while (length < distance) {
    const auto next_lanelet = get_next_lanelet_within_route(current_lanelet, planner_data);
    if (!next_lanelet) {
      break;
    }

    lanelets.push_back(*next_lanelet);
    current_lanelet = *next_lanelet;
    length += lanelet::utils::getLaneletLength2d(*next_lanelet);
  }

  return lanelets;
}

std::optional<lanelet::ConstLanelet> get_previous_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const RouteContext & planner_data)
{
  if (exists(planner_data.start_lanelets, lanelet)) {
    return std::nullopt;
  }

  const auto prev_lanelets = planner_data.routing_graph_ptr->previous(lanelet);
  if (prev_lanelets.empty()) {
    return std::nullopt;
  }

  const auto prev_lanelet_itr = std::find_if(
    prev_lanelets.cbegin(), prev_lanelets.cend(),
    [&](const lanelet::ConstLanelet & l) { return exists(planner_data.route_lanelets, l); });
  if (prev_lanelet_itr == prev_lanelets.cend()) {
    return std::nullopt;
  }
  return *prev_lanelet_itr;
}

std::optional<lanelet::ConstLanelet> get_next_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const RouteContext & planner_data)
{
  if (planner_data.preferred_lanelets.empty()) {
    return std::nullopt;
  }

  if (exists(planner_data.goal_lanelets, lanelet)) {
    return std::nullopt;
  }

  const auto next_lanelets = planner_data.routing_graph_ptr->following(lanelet);
  if (
    next_lanelets.empty() ||
    next_lanelets.front().id() == planner_data.preferred_lanelets.front().id()) {
    return std::nullopt;
  }

  // Fallback: any route lanelet
  const auto next_lanelet_itr = std::find_if(
    next_lanelets.cbegin(), next_lanelets.cend(),
    [&](const lanelet::ConstLanelet & l) { return exists(planner_data.route_lanelets, l); });
  if (next_lanelet_itr == next_lanelets.cend()) {
    return std::nullopt;
  }
  return *next_lanelet_itr;
}

std::vector<WaypointGroup> get_waypoint_groups(
  const lanelet::LaneletSequence & lanelet_sequence, const lanelet::LaneletMap & lanelet_map,
  const double group_separation_threshold, const double interval_margin_ratio)
{
  std::vector<WaypointGroup> waypoint_groups{};

  const auto get_interval_bound =
    [&](const lanelet::ConstPoint3d & point, const double lateral_distance_factor) {
      const auto arc_coordinates = lanelet::geometry::toArcCoordinates(
        lanelet_sequence.centerline2d(), lanelet::utils::to2D(point));
      return arc_coordinates.length + lateral_distance_factor * std::abs(arc_coordinates.distance);
    };

  for (const auto & lanelet : lanelet_sequence) {
    if (!lanelet.hasAttribute("waypoints")) {
      continue;
    }

    const auto waypoints_id = lanelet.attribute("waypoints").asId().value();
    const auto & waypoints = lanelet_map.lineStringLayer.get(waypoints_id);

    if (
      waypoint_groups.empty() || lanelet::geometry::distance2d(
                                   waypoint_groups.back().waypoints.back().point,
                                   waypoints.front()) > group_separation_threshold) {
      waypoint_groups.emplace_back().interval.start =
        get_interval_bound(waypoints.front(), -interval_margin_ratio);
    }
    waypoint_groups.back().interval.end =
      get_interval_bound(waypoints.back(), interval_margin_ratio);

    std::transform(
      waypoints.begin(), waypoints.end(), std::back_inserter(waypoint_groups.back().waypoints),
      [&](const lanelet::ConstPoint3d & waypoint) {
        return WaypointGroup::Waypoint{waypoint, lanelet.id()};
      });
  }

  return waypoint_groups;
}

std::optional<double> get_first_intersection_arc_length(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end,
  const double vehicle_length)
{
  if (lanelet_sequence.empty()) {
    return std::nullopt;
  }

  std::optional<double> s_intersection;

  const auto s_start_on_bounds = get_arc_length_on_bounds(lanelet_sequence, s_start);
  const auto s_end_on_bounds = get_arc_length_on_bounds(lanelet_sequence, s_end);

  const auto crop_and_convert = [](const auto & line, double s0, double s1) {
    return lanelet::utils::to2D(to_lanelet_points(
      crop_line_string(to_geometry_msgs_points(line.begin(), line.end()), s0, s1)));
  };

  const auto cropped_centerline = crop_and_convert(lanelet_sequence.centerline2d(), s_start, s_end);
  const auto cropped_left_bound =
    crop_and_convert(lanelet_sequence.leftBound2d(), s_start_on_bounds.left, s_end_on_bounds.left);
  const auto cropped_right_bound = crop_and_convert(
    lanelet_sequence.rightBound2d(), s_start_on_bounds.right, s_end_on_bounds.right);

  if (cropped_centerline.empty() || cropped_left_bound.empty() || cropped_right_bound.empty()) {
    return std::nullopt;
  }

  const lanelet::BasicLineString2d start_edge{
    cropped_left_bound.front(), cropped_right_bound.front()};

  // Helper: offset an optional arc length by a base value
  auto offset_optional = [](std::optional<double> s, double base) -> std::optional<double> {
    if (s) *s += base;
    return s;
  };

  // Self intersection of bounds
  {
    const auto s_left_bound = offset_optional(
      get_first_self_intersection_arc_length(cropped_left_bound), s_start_on_bounds.left);
    const auto s_right_bound = offset_optional(
      get_first_self_intersection_arc_length(cropped_right_bound), s_start_on_bounds.right);

    const auto [s_left, s_right] =
      get_arc_length_on_centerline(lanelet_sequence, s_left_bound, s_right_bound);

    if (s_left && s_right) {
      s_intersection = std::min(s_left, s_right);
    } else {
      s_intersection = s_left ? s_left : s_right;
    }
  }

  // intersection between left and right bounds
  {
    lanelet::BasicPoints2d intersections;
    boost::geometry::intersection(cropped_left_bound, cropped_right_bound, intersections);
    for (const auto & intersection : intersections) {
      const auto s_on_centerline = get_arc_length_on_centerline(
        lanelet_sequence,
        s_start_on_bounds.left +
          lanelet::geometry::toArcCoordinates(cropped_left_bound, intersection).length,
        s_start_on_bounds.right +
          lanelet::geometry::toArcCoordinates(cropped_right_bound, intersection).length);
      const auto s_mutual = [&]() -> std::optional<double> {
        if (s_on_centerline.left && s_on_centerline.right) {
          return std::max(s_on_centerline.left, s_on_centerline.right);
        }
        return s_on_centerline.left ? s_on_centerline.left : s_on_centerline.right;
      }();
      if (s_intersection && s_mutual) {
        s_intersection = std::min(s_intersection, s_mutual);
      } else if (s_mutual) {
        s_intersection = s_mutual;
      }
    }
  }

  // intersection between start edge of drivable area and left / right bound
  {
    const auto get_start_edge_intersection_arc_length =
      [&](const lanelet::BasicLineString2d & bound) {
        std::optional<double> s_start_edge = std::nullopt;
        if (bound.size() <= 2) {
          return s_start_edge;
        }
        lanelet::BasicPoints2d start_edge_intersections;
        boost::geometry::intersection(start_edge, bound, start_edge_intersections);
        for (const auto & intersection : start_edge_intersections) {
          if (boost::geometry::equals(intersection, bound.front())) {
            continue;
          }
          const auto s = lanelet::geometry::toArcCoordinates(bound, intersection).length;
          s_start_edge = s_start_edge ? std::min(*s_start_edge, s) : s;
        }
        return s_start_edge;
      };
    const auto s_left_bound = [&]() {
      auto s = get_start_edge_intersection_arc_length(cropped_left_bound);
      if (s) {
        *s += s_start_on_bounds.left;
      }
      return s;
    }();
    const auto s_right_bound = [&]() {
      auto s = get_start_edge_intersection_arc_length(cropped_right_bound);
      if (s) {
        *s += s_start_on_bounds.right;
      }
      return s;
    }();

    const auto s_on_centerline =
      get_arc_length_on_centerline(lanelet_sequence, s_left_bound, s_right_bound);

    const auto s_start_edge = [&]() {
      if (s_on_centerline.left && s_on_centerline.right) {
        return std::min(s_on_centerline.left, s_on_centerline.right);
      }
      return s_on_centerline.left ? s_on_centerline.left : s_on_centerline.right;
    }();
    if (s_intersection && s_start_edge) {
      s_intersection = std::min(s_intersection, s_start_edge);
    } else if (s_start_edge) {
      s_intersection = s_start_edge;
    }
  }

  // intersection between start edge of drivable area and center line
  {
    std::optional<double> s_start_edge;
    lanelet::BasicPoints2d start_edge_intersections;
    boost::geometry::intersection(start_edge, cropped_centerline, start_edge_intersections);
    for (const auto & intersection : start_edge_intersections) {
      auto s = lanelet::geometry::toArcCoordinates(cropped_centerline, intersection).length;
      // Ignore intersections near the beginning of the centerline.
      // It is impossible to make a turn shorter than the vehicle_length, so use it as a threshold.
      if (s < vehicle_length) continue;
      s += s_start;
      s_start_edge = s_start_edge ? std::min(*s_start_edge, s) : s;
    }
    if (s_intersection && s_start_edge) {
      s_intersection = std::min(s_intersection, s_start_edge);
    } else if (s_start_edge) {
      s_intersection = s_start_edge;
    }
  }

  return s_intersection;
}

std::optional<double> get_first_self_intersection_arc_length(
  const lanelet::BasicLineString2d & line_string)
{
  if (line_string.size() < 3) {
    return std::nullopt;
  }

  std::optional<size_t> first_self_intersection_index;
  std::optional<double> intersection_arc_length_on_latter_segment;
  double s = 0.;

  for (size_t i = 0; i < line_string.size() - 1; ++i) {
    if (
      first_self_intersection_index && i == first_self_intersection_index &&
      intersection_arc_length_on_latter_segment) {
      return s + *intersection_arc_length_on_latter_segment;
    }

    const auto current_segment = lanelet::BasicSegment2d{line_string.at(i), line_string.at(i + 1)};
    s += lanelet::geometry::length(current_segment);

    lanelet::BasicPoints2d self_intersections{};
    for (size_t j = i + 1; j < line_string.size() - 1; ++j) {
      const auto segment = lanelet::BasicSegment2d{line_string.at(j), line_string.at(j + 1)};
      if (
        segment.first == current_segment.second || segment.second == current_segment.first ||
        segment.first == current_segment.first) {
        continue;
      }
      boost::geometry::intersection(current_segment, segment, self_intersections);
      if (self_intersections.empty()) {
        continue;
      }
      first_self_intersection_index = j;
      intersection_arc_length_on_latter_segment =
        (self_intersections.front() - segment.first).norm();
      break;
    }
  }

  return std::nullopt;
}

double get_arc_length_on_path(
  const lanelet::LaneletSequence & lanelet_sequence, const std::vector<PathPointWithLaneId> & path,
  const double s_centerline)
{
  std::optional<lanelet::Id> target_lanelet_id;
  std::optional<lanelet::BasicPoint2d> point_on_centerline;

  if (lanelet_sequence.empty() || path.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("utils").get_child(__func__),
      "Input lanelet sequence or path is empty, returning 0.");
    return 0.;
  }

  if (s_centerline < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("utils").get_child(__func__),
      "Input arc length is negative, returning 0.");
    return 0.;
  }

  for (auto [it, s] = std::make_tuple(lanelet_sequence.begin(), 0.); it != lanelet_sequence.end();
       ++it) {
    const double centerline_length = lanelet::geometry::length(it->centerline2d());
    if (s + centerline_length < s_centerline) {
      s += centerline_length;
      continue;
    }

    target_lanelet_id = it->id();
    point_on_centerline =
      lanelet::geometry::interpolatedPointAtDistance(it->centerline2d(), s_centerline - s);
    break;
  }

  if (!target_lanelet_id || !point_on_centerline) {
    // lanelet_sequence is too short, thus we return input arc length as is.
    return s_centerline;
  }

  auto s_path = 0.;
  lanelet::BasicLineString2d target_path_segment;

  for (auto it = path.begin(); it != path.end(); ++it) {
    if (
      std::find(it->lane_ids.begin(), it->lane_ids.end(), *target_lanelet_id) ==
      it->lane_ids.end()) {
      if (target_path_segment.empty() && it != std::prev(path.end())) {
        s_path += autoware_utils::calc_distance2d(*it, *std::next(it));
        continue;
      }
      break;
    }
    target_path_segment.push_back(
      lanelet::utils::conversion::toLaneletPoint(it->point.pose.position).basicPoint2d());
  }

  // guard: toArcCoordinates requires at least 2 points in the line string,
  // otherwise closestSegment() dereferences a null pointer.
  if (target_path_segment.size() < 2) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("utils").get_child(__func__),
      "target_path_segment has insufficient points (%zu), "
      "falling back to closest point search on path",
      target_path_segment.size());
    // fallback: find the closest point on the entire path to point_on_centerline
    double min_dist_sq = std::numeric_limits<double>::max();
    double s_closest = 0.;
    double s_acc = 0.;
    for (size_t i = 0; i < path.size(); ++i) {
      const auto & pos = path[i].point.pose.position;
      const double dx = pos.x - point_on_centerline->x();
      const double dy = pos.y - point_on_centerline->y();
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        s_closest = s_acc;
      }
      if (i + 1 < path.size()) {
        s_acc += autoware_utils::calc_distance2d(path[i], path[i + 1]);
      }
    }
    return s_closest;
  }

  s_path += lanelet::geometry::toArcCoordinates(target_path_segment, *point_on_centerline).length;

  return s_path;
}

PathRange<std::vector<geometry_msgs::msg::Point>> get_path_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end)
{
  if (lanelet_sequence.empty()) {
    return {};
  }

  const auto [s_left_start, s_right_start] = get_arc_length_on_bounds(lanelet_sequence, s_start);
  const auto [s_left_end, s_right_end] = get_arc_length_on_bounds(lanelet_sequence, s_end);

  return {
    crop_line_string(
      to_geometry_msgs_points(
        lanelet_sequence.leftBound().begin(), lanelet_sequence.leftBound().end()),
      s_left_start, s_left_end),
    crop_line_string(
      to_geometry_msgs_points(
        lanelet_sequence.rightBound().begin(), lanelet_sequence.rightBound().end()),
      s_right_start, s_right_end)};
}

std::vector<geometry_msgs::msg::Point> crop_line_string(
  const std::vector<geometry_msgs::msg::Point> & line_string, const double s_start,
  const double s_end)
{
  if (s_start < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("utils").get_child(__func__),
      "Start of crop range is negative, returning input as is");
    return line_string;
  }

  if (s_start > s_end) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("utils").get_child(__func__),
      "Start of crop range is larger than end, returning input as is");
    return line_string;
  }

  auto trajectory =
    autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Point>::Builder()
      .set_xy_interpolator<autoware::experimental::trajectory::interpolator::Linear>()
      .build(line_string);
  if (!trajectory) {
    return {};
  }

  trajectory->crop(s_start, s_end - s_start);
  return trajectory->restore();
}

PathRange<double> get_arc_length_on_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_centerline)
{
  if (s_centerline < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("utils").get_child(__func__),
      "Input arc length is negative, returning 0.");
    return {0., 0.};
  }

  auto s = 0.;
  auto s_left = 0.;
  auto s_right = 0.;

  for (auto it = lanelet_sequence.begin(); it != lanelet_sequence.end(); ++it) {
    const double centerline_length = lanelet::geometry::length(it->centerline2d());
    const double left_bound_length = lanelet::geometry::length(it->leftBound2d());
    const double right_bound_length = lanelet::geometry::length(it->rightBound2d());

    if (s + centerline_length < s_centerline) {
      s += centerline_length;
      s_left += left_bound_length;
      s_right += right_bound_length;
      continue;
    }

    const auto point_on_centerline =
      lanelet::geometry::interpolatedPointAtDistance(it->centerline2d(), s_centerline - s);
    s_left += lanelet::geometry::toArcCoordinates(it->leftBound2d(), point_on_centerline).length;
    s_right += lanelet::geometry::toArcCoordinates(it->rightBound2d(), point_on_centerline).length;

    return {s_left, s_right};
  }

  // If the loop ends without returning, it means that the lanelet_sequence is too short.
  // In this case, we return the original arc length on the centerline.
  return {s_centerline, s_centerline};
}

PathRange<std::optional<double>> get_arc_length_on_centerline(
  const lanelet::LaneletSequence & lanelet_sequence, const std::optional<double> & s_left_bound,
  const std::optional<double> & s_right_bound)
{
  std::optional<double> s_left_centerline;
  std::optional<double> s_right_centerline;

  if (s_left_bound && *s_left_bound < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("utils").get_child(__func__),
      "Input left arc length is negative, returning 0.");
    s_left_centerline = 0.;
  }
  if (s_right_bound && *s_right_bound < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("utils").get_child(__func__),
      "Input right arc length is negative, returning 0.");
    s_right_centerline = 0.;
  }

  auto s = 0.;
  auto s_left = 0.;
  auto s_right = 0.;

  for (auto it = lanelet_sequence.begin(); it != lanelet_sequence.end(); ++it) {
    const auto is_left_done = !s_left_bound || s_left_centerline;
    const auto is_right_done = !s_right_bound || s_right_centerline;
    if (is_left_done && is_right_done) {
      break;
    }

    const double centerline_length = lanelet::utils::getLaneletLength2d(*it);
    const double left_bound_length = lanelet::geometry::length(it->leftBound2d());
    const double right_bound_length = lanelet::geometry::length(it->rightBound2d());

    if (!is_left_done && s_left + left_bound_length > s_left_bound) {
      s_left_centerline = s + lanelet::geometry::toArcCoordinates(
                                it->centerline2d(), lanelet::geometry::interpolatedPointAtDistance(
                                                      it->leftBound2d(), *s_left_bound - s_left))
                                .length;
    }
    if (!is_right_done && s_right + right_bound_length > s_right_bound) {
      s_right_centerline =
        s + lanelet::geometry::toArcCoordinates(
              it->centerline2d(), lanelet::geometry::interpolatedPointAtDistance(
                                    it->rightBound2d(), *s_right_bound - s_right))
              .length;
    }

    s += centerline_length;
    s_left += left_bound_length;
    s_right += right_bound_length;
  }

  return {
    s_left_centerline ? s_left_centerline : s_left_bound,
    s_right_centerline ? s_right_centerline : s_right_bound};
}

PathPointTrajectory refine_path_for_goal(
  const PathPointTrajectory & input, const geometry_msgs::msg::Pose & goal_pose,
  const lanelet::Id goal_lane_id, const double search_radius_range, const double pre_goal_offset)
{
  auto contain_goal_lane_id = [&](const PathPointWithLaneId & point) {
    const auto & ids = point.lane_ids;
    return std::find(ids.begin(), ids.end(), goal_lane_id) != ids.end();
  };

  auto outside_circle = [&](const PathPointWithLaneId & point) {
    return autoware_utils::calc_distance2d(point.point.pose, goal_pose) > search_radius_range;
  };

  auto closest_to_goal = autoware::experimental::trajectory::closest_with_constraint(
    input, goal_pose, contain_goal_lane_id);

  // If no point with the goal lane ID exists in the trajectory (e.g. goal is on an adjacent lane),
  // fall back to the geometrically closest point so the goal connection still applies.
  if (!closest_to_goal) {
    closest_to_goal = autoware::experimental::trajectory::closest(input, goal_pose);
  }

  auto cropped_path = autoware::experimental::trajectory::crop(input, 0, *closest_to_goal);

  auto intervals =
    autoware::experimental::trajectory::find_intervals(cropped_path, outside_circle, 10);

  std::vector<PathPointWithLaneId> goal_connected_trajectory_points;

  if (!intervals.empty()) {
    auto cropped = autoware::experimental::trajectory::crop(cropped_path, 0, intervals.back().end);
    goal_connected_trajectory_points = cropped.restore(2);
  } else if (cropped_path.length() > pre_goal_offset) {
    // If distance from start to goal is smaller than refine_goal_search_radius_range and start is
    // farther from goal than pre_goal, we just connect start, pre_goal, and goal.
    goal_connected_trajectory_points = {cropped_path.compute(0)};
  }

  auto goal = input.compute(autoware::experimental::trajectory::closest(input, goal_pose));
  goal.point.pose = goal_pose;
  goal.point.longitudinal_velocity_mps = 0.0;

  const auto pre_goal_pose =
    autoware_utils::calc_offset_pose(goal_pose, -pre_goal_offset, 0.0, 0.0);
  auto pre_goal = input.compute(autoware::experimental::trajectory::closest(input, pre_goal_pose));
  pre_goal.point.pose = pre_goal_pose;

  goal_connected_trajectory_points.push_back(pre_goal);
  goal_connected_trajectory_points.push_back(goal);

  if (
    const auto output =
      autoware::experimental::trajectory::pretty_build(goal_connected_trajectory_points)) {
    return *output;
  }
  return input;
}

lanelet::ConstLanelets extract_lanelets_from_trajectory(
  const PathPointTrajectory & trajectory, const RouteContext & planner_data)
{
  lanelet::ConstLanelets lanelets{};
  const auto lane_ids = trajectory.get_contained_lane_ids();
  const auto lane_ids_set = std::set(lane_ids.begin(), lane_ids.end());
  for (const auto & lane_id : lane_ids_set) {
    const auto lanelet = planner_data.lanelet_map_ptr->laneletLayer.get(lane_id);
    lanelets.push_back(lanelet);
  }
  return lanelets;
}

bool is_in_lanelets(const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & lanes)
{
  return std::any_of(lanes.begin(), lanes.end(), [&](const auto & lane) {
    return lanelet::utils::isInLanelet(pose, lane);
  });
}

bool is_trajectory_inside_lanelets(
  const PathPointTrajectory & refined_path, const lanelet::ConstLanelets & lanelets)
{
  const auto points = refined_path.restore();
  return std::none_of(points.begin(), points.end(), [&](const auto & point) {
    return !is_in_lanelets(point.point.pose, lanelets);
  });
}

std::optional<PathPointTrajectory> modify_path_for_smooth_goal_connection(
  const PathPointTrajectory & trajectory, const RouteContext & planner_data,
  const double search_radius_range, const double pre_goal_offset)
{
  if (planner_data.preferred_lanelets.empty()) {
    return std::nullopt;
  }
  // Build the set of lanelets valid for the refined path.
  // Include goal lanelets so that a path connecting to an adjacent goal lane passes validation.
  auto lanelets = extract_lanelets_from_trajectory(trajectory, planner_data);
  for (const auto & goal_ll : planner_data.goal_lanelets) {
    if (std::find(lanelets.begin(), lanelets.end(), goal_ll) == lanelets.end()) {
      lanelets.push_back(goal_ll);
    }
  }

  // Include lanelets that contain the goal pose (e.g., shoulder lanelets) so that
  // the trajectory connecting to an off-lane goal passes the inside-lanelets check.
  const auto goal_point_2d =
    lanelet::BasicPoint2d(planner_data.goal_pose.position.x, planner_data.goal_pose.position.y);
  const auto nearest_lanelets =
    lanelet::geometry::findNearest(planner_data.lanelet_map_ptr->laneletLayer, goal_point_2d, 5);
  for (const auto & [dist, ll] : nearest_lanelets) {
    if (
      lanelet::geometry::inside(ll, goal_point_2d) &&
      std::find(lanelets.begin(), lanelets.end(), ll) == lanelets.end()) {
      lanelets.push_back(ll);
    }
  }

  // This process is to fit the trajectory inside the lanelets. By reducing
  // refine_goal_search_radius_range, we can fit the trajectory inside lanelets even if the
  // trajectory has a high curvature.
  for (double s = search_radius_range; s > 0; s -= 0.1) {
    const auto refined_trajectory = refine_path_for_goal(
      trajectory, planner_data.goal_pose, planner_data.preferred_lanelets.back().id(), s,
      pre_goal_offset);
    if (is_trajectory_inside_lanelets(refined_trajectory, lanelets)) {
      return refined_trajectory;
    }
  }
  return std::nullopt;
}

}  // namespace utils

// ===========================================================================
// File-local helpers
// ===========================================================================

namespace
{
/**
 * @brief Select a lanelet from candidates, preferring one that is in the route.
 */
lanelet::ConstLanelet select_route_preferred_lanelet(
  const lanelet::ConstLanelets & candidates, const RouteContext & route_context)
{
  const auto it =
    std::find_if(candidates.begin(), candidates.end(), [&](const lanelet::ConstLanelet & ll) {
      return std::any_of(
        route_context.route_lanelets.begin(), route_context.route_lanelets.end(),
        [&](const auto & rl) { return rl.id() == ll.id(); });
    });
  return it != candidates.end() ? *it : candidates.front();
}
}  // namespace

// ===========================================================================
// PathPlanner member functions
// ===========================================================================

bool PathPlanner::update_current_lanelet(const geometry_msgs::msg::Pose & current_pose)
{
  if (!current_lanelet_) {
    lanelet::ConstLanelet closest;
    if (lanelet::utils::query::getClosestLanelet(
          route_context_.route_lanelets, current_pose, &closest)) {
      current_lanelet_ = closest;
      return true;
    }
    return false;
  }

  lanelet::ConstLanelets candidates;
  if (
    const auto previous_lanelet =
      utils::get_previous_lanelet_within_route(*current_lanelet_, route_context_)) {
    candidates.push_back(*previous_lanelet);
  }
  candidates.push_back(*current_lanelet_);
  if (
    const auto next_lanelet =
      utils::get_next_lanelet_within_route(*current_lanelet_, route_context_)) {
    candidates.push_back(*next_lanelet);
  }

  // Include adjacent route lanelets so that ego transitions to the correct lane
  // during lane changes (e.g., 2->4) instead of being picked up by a longitudinally
  // adjacent lanelet on the wrong lane (e.g., 3)
  for (const auto & beside : route_context_.routing_graph_ptr->besides(*current_lanelet_)) {
    if (
      beside.id() != current_lanelet_->id() &&
      std::any_of(
        route_context_.route_lanelets.begin(), route_context_.route_lanelets.end(),
        [&](const auto & rl) { return rl.id() == beside.id(); })) {
      candidates.push_back(beside);
    }
  }

  if (lanelet::utils::query::getClosestLaneletWithConstrains(
        candidates, current_pose, &*current_lanelet_,
        params_.path_planning.ego_nearest_lanelet.dist_threshold,
        params_.path_planning.ego_nearest_lanelet.yaw_threshold)) {
    return true;
  }

  if (lanelet::utils::query::getClosestLanelet(
        route_context_.route_lanelets, current_pose, &*current_lanelet_)) {
    return true;
  }

  return false;
}

std::optional<PathWithLaneId> PathPlanner::plan_path(const geometry_msgs::msg::Pose & current_pose)
{
  const auto path_length_backward = params_.path_planning.path_length.backward;
  const auto path_length_forward = params_.path_planning.path_length.forward;

  if (!update_current_lanelet(current_pose)) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 5000, "Failed to update current lanelet");
    return std::nullopt;
  }

  // No transition needed: use existing logic with closest preferred lanelet
  const auto & base_lanelet = [&]() -> const lanelet::ConstLanelet & {
    if (route_context_.preferred_lanelets.empty()) {
      return *current_lanelet_;
    }
    lanelet::ConstLanelet closest;
    if (lanelet::utils::query::getClosestLanelet(
          route_context_.preferred_lanelets, current_pose, &closest)) {
      route_context_.closest_preferred_lanelet = closest;
      return *route_context_.closest_preferred_lanelet;
    }
    return *current_lanelet_;
  }();

  lanelet::ConstLanelets lanelets{base_lanelet};
  const auto s_on_current_lanelet =
    lanelet::utils::getArcCoordinates({base_lanelet}, current_pose).length;

  const auto backward_length = std::max(
    0., path_length_backward + vehicle_info_.max_longitudinal_offset_m - s_on_current_lanelet);
  const auto backward_lanelets_within_route =
    utils::get_lanelets_within_route_up_to(base_lanelet, route_context_, backward_length);
  if (!backward_lanelets_within_route) {
    RCLCPP_ERROR(
      logger_, "Failed to get backward lanelets within route for current lanelet (id: %ld)",
      base_lanelet.id());
    return std::nullopt;
  }
  lanelets.insert(
    lanelets.begin(), backward_lanelets_within_route->begin(),
    backward_lanelets_within_route->end());

  //  Extend lanelets by backward_length even outside planned route to ensure
  //  ego footprint is inside lanelets if ego is at the beginning of start lane
  auto backward_lanelets_length =
    lanelet::utils::getLaneletLength2d(*backward_lanelets_within_route);
  while (backward_lanelets_length < backward_length) {
    const auto prev_lanelets = route_context_.routing_graph_ptr->previous(lanelets.front());
    if (prev_lanelets.empty()) {
      break;
    }
    // [Bug fix] Prefer a lanelet that is in the route at branch points
    const auto selected = select_route_preferred_lanelet(prev_lanelets, route_context_);
    lanelets.insert(lanelets.begin(), selected);
    backward_lanelets_length += lanelet::geometry::length2d(selected);
  }

  const auto forward_length = std::max(
    0., path_length_forward + vehicle_info_.max_longitudinal_offset_m -
          (lanelet::geometry::length2d(base_lanelet) - s_on_current_lanelet));
  const auto forward_lanelets_within_route =
    utils::get_lanelets_within_route_after(base_lanelet, route_context_, forward_length);
  if (!forward_lanelets_within_route) {
    RCLCPP_ERROR(
      logger_, "Failed to get forward lanelets within route for current lanelet (id: %ld)",
      base_lanelet.id());
    return std::nullopt;
  }
  lanelets.insert(
    lanelets.end(), forward_lanelets_within_route->begin(), forward_lanelets_within_route->end());

  //  Extend lanelets by forward_length even outside planned route to ensure
  //  ego footprint is inside lanelets if ego is at the end of goal lane
  auto forward_lanelets_length = lanelet::utils::getLaneletLength2d(*forward_lanelets_within_route);
  while (forward_lanelets_length < forward_length) {
    const auto next_lanelets = route_context_.routing_graph_ptr->following(lanelets.back());
    if (next_lanelets.empty()) {
      break;
    }
    // [Bug fix] Prefer a lanelet that is in the route at branch points
    const auto selected = select_route_preferred_lanelet(next_lanelets, route_context_);
    lanelets.insert(lanelets.end(), selected);
    forward_lanelets_length += lanelet::geometry::length2d(selected);
  }

  const auto s = s_on_current_lanelet + backward_lanelets_length;
  const auto s_start = std::max(0., s - path_length_backward);
  const auto s_end = [&]() {
    auto s_end_val = s + path_length_forward;

    if (!utils::get_next_lanelet_within_route(lanelets.back(), route_context_)) {
      s_end_val = std::min(s_end_val, lanelet::utils::getLaneletLength2d(lanelets));
    }

    auto is_goal_lanelet = [&](const lanelet::ConstLanelet & ll) {
      return std::any_of(
        route_context_.goal_lanelets.begin(), route_context_.goal_lanelets.end(),
        [&](const auto & goal_ll) { return ll.id() == goal_ll.id(); });
    };

    for (auto [it, goal_arc_length] = std::make_tuple(lanelets.begin(), 0.); it != lanelets.end();
         ++it) {
      if (is_goal_lanelet(*it)) {
        goal_arc_length +=
          lanelet::utils::getArcCoordinates({*it}, route_context_.goal_pose).length;
        s_end_val = std::min(s_end_val, goal_arc_length);
        break;
      }
      goal_arc_length += lanelet::geometry::length2d(*it);
    }

    const lanelet::LaneletSequence lanelet_seq(lanelets);
    if (
      const auto s_intersection = utils::get_first_intersection_arc_length(
        lanelet_seq, std::max(0., s_start - vehicle_info_.max_longitudinal_offset_m),
        s_end_val + vehicle_info_.max_longitudinal_offset_m, vehicle_info_.vehicle_length_m)) {
      s_end_val = std::min(
        s_end_val, std::max(0., *s_intersection - vehicle_info_.max_longitudinal_offset_m));
    }

    return s_end_val;
  }();

  if (s_end <= s_start) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "s_end (%.2f) <= s_start (%.2f), cannot generate path", s_end,
      s_start);
    return std::nullopt;
  }

  return generate_path(lanelets, s_start, s_end);
}

std::optional<PathWithLaneId> PathPlanner::generate_path(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end)
{
  if (lanelet_sequence.empty()) {
    RCLCPP_ERROR(logger_, "Lanelet sequence is empty");
    return std::nullopt;
  }

  std::vector<PathPointWithLaneId> path_points_with_lane_id{};

  const auto waypoint_groups = utils::get_waypoint_groups(
    lanelet_sequence, *route_context_.lanelet_map_ptr,
    params_.path_planning.waypoint_group.separation_threshold,
    params_.path_planning.waypoint_group.interval_margin_ratio);

  auto extended_lanelets = lanelet_sequence.lanelets();
  auto extended_arc_length = 0.;
  for (const auto & [waypoints, interval] : waypoint_groups) {
    while (interval.start + extended_arc_length < 0.) {
      const auto prev_lanelets =
        route_context_.routing_graph_ptr->previous(extended_lanelets.front());
      if (prev_lanelets.empty()) {
        break;
      }
      extended_lanelets.insert(extended_lanelets.begin(), prev_lanelets.front());
      extended_arc_length += lanelet::utils::getLaneletLength2d(prev_lanelets.front());
    }
  }

  const auto add_path_point =
    [&](const lanelet::ConstPoint3d & path_point, const lanelet::Id & lane_id) {
      PathPointWithLaneId path_point_with_lane_id{};
      path_point_with_lane_id.lane_ids.push_back(lane_id);
      path_point_with_lane_id.point.pose.position =
        lanelet::utils::conversion::toGeomMsgPt(path_point);
      path_point_with_lane_id.point.longitudinal_velocity_mps =
        route_context_.traffic_rules_ptr
          ->speedLimit(route_context_.lanelet_map_ptr->laneletLayer.get(lane_id))
          .speedLimit.value();
      path_points_with_lane_id.push_back(std::move(path_point_with_lane_id));
    };

  const lanelet::LaneletSequence extended_lanelet_sequence(extended_lanelets);
  std::optional<size_t> overlapping_waypoint_group_index;

  for (auto [lanelet_it, s] = std::make_tuple(extended_lanelet_sequence.begin(), 0.);
       lanelet_it != extended_lanelet_sequence.end(); ++lanelet_it) {
    const auto & centerline = lanelet_it->centerline();

    for (auto point_it = centerline.begin(); point_it != centerline.end(); ++point_it) {
      if (point_it != centerline.begin()) {
        s += lanelet::geometry::distance2d(*std::prev(point_it), *point_it);
      } else if (lanelet_it != extended_lanelet_sequence.begin()) {
        continue;
      }

      if (overlapping_waypoint_group_index) {
        const auto & [waypoints, interval] = waypoint_groups[*overlapping_waypoint_group_index];
        if (s >= interval.start + extended_arc_length && s <= interval.end + extended_arc_length) {
          continue;
        }
        overlapping_waypoint_group_index.reset();
      }

      for (size_t i = 0; i < waypoint_groups.size(); ++i) {
        const auto & [waypoints, interval] = waypoint_groups[i];
        if (s < interval.start + extended_arc_length || s > interval.end + extended_arc_length) {
          continue;
        }
        for (const auto & waypoint : waypoints) {
          add_path_point(waypoint.point, waypoint.lane_id);
        }
        overlapping_waypoint_group_index = i;
        break;
      }
      if (overlapping_waypoint_group_index) {
        continue;
      }

      add_path_point(*point_it, lanelet_it->id());
      if (
        point_it == std::prev(centerline.end()) &&
        lanelet_it != std::prev(extended_lanelet_sequence.end())) {
        if (
          lanelet_it != extended_lanelet_sequence.begin() ||
          lanelet_it->id() == lanelet_sequence.begin()->id()) {
          path_points_with_lane_id.back().lane_ids.push_back(std::next(lanelet_it)->id());
        } else {
          path_points_with_lane_id.back().lane_ids = {std::next(lanelet_it)->id()};
        }
      }
    }
  }

  if (path_points_with_lane_id.empty()) {
    RCLCPP_ERROR(logger_, "No path points generated from lanelet sequence");
    return std::nullopt;
  }

  auto trajectory = autoware::experimental::trajectory::pretty_build(path_points_with_lane_id);
  if (!trajectory) {
    RCLCPP_ERROR(logger_, "Failed to build trajectory from path points");
    return std::nullopt;
  }

  // Attach orientation for all the points
  trajectory->align_orientation_with_trajectory_direction();

  const auto s_path_start = utils::get_arc_length_on_path(
    extended_lanelet_sequence, path_points_with_lane_id, extended_arc_length + s_start);
  const auto s_path_end = utils::get_arc_length_on_path(
    extended_lanelet_sequence, path_points_with_lane_id, extended_arc_length + s_end);

  // Crop start first so that goal connection shortening does not skip start cropping
  if (s_path_start > 0 && trajectory->length() > s_path_start) {
    trajectory->crop(s_path_start, trajectory->length() - s_path_start);
  }

  // Adjust s_path_end relative to the new trajectory origin
  const auto adjusted_s_path_end = s_path_end - s_path_start;

  // Check if the goal point is in the search range
  // Note: We only see if the goal is approaching the tail of the path.
  const auto s_path_end_clamped = std::min(trajectory->length(), adjusted_s_path_end);
  const auto distance_to_goal = autoware_utils::calc_distance2d(
    trajectory->compute(s_path_end_clamped), route_context_.goal_pose);

  bool goal_connection_applied = false;
  if (distance_to_goal < params_.path_planning.smooth_goal_connection.search_radius_range) {
    auto refined_path = utils::modify_path_for_smooth_goal_connection(
      *trajectory, route_context_, params_.path_planning.smooth_goal_connection.search_radius_range,
      params_.path_planning.smooth_goal_connection.pre_goal_offset);

    if (refined_path) {
      refined_path->align_orientation_with_trajectory_direction();
      *trajectory = *refined_path;
      goal_connection_applied = true;
    }
  }

  // Crop end
  if (!goal_connection_applied && trajectory->length() > adjusted_s_path_end) {
    trajectory->crop(0., adjusted_s_path_end);
  }

  if (trajectory->length() < 1e-3) {
    RCLCPP_WARN(logger_, "Trajectory length too short after cropping: %f", trajectory->length());
    return std::nullopt;
  }

  // Compose the polished path
  PathWithLaneId finalized_path_with_lane_id{};
  finalized_path_with_lane_id.points = trajectory->restore();

  if (finalized_path_with_lane_id.points.empty()) {
    RCLCPP_ERROR(logger_, "Finalized path points are empty after cropping");
    return std::nullopt;
  }

  // Set header which is needed to engage
  finalized_path_with_lane_id.header.frame_id = route_context_.route_frame_id;
  finalized_path_with_lane_id.header.stamp = clock_->now();

  const auto [left_bound, right_bound] = utils::get_path_bounds(
    extended_lanelet_sequence,
    std::max(0., extended_arc_length + s_start - vehicle_info_.max_longitudinal_offset_m),
    extended_arc_length + s_end + vehicle_info_.max_longitudinal_offset_m);
  finalized_path_with_lane_id.left_bound = left_bound;
  finalized_path_with_lane_id.right_bound = right_bound;

  return finalized_path_with_lane_id;
}

// ===========================================================================
// Trajectory shifting (from path_shift_to_ego.cpp)
// ===========================================================================

Trajectory PathPlanner::shift_trajectory_to_ego(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & ego_pose,
  const double ego_velocity, const double ego_yaw_rate, const TrajectoryShiftParams & params,
  const double delta_arc_length)
{
  if (trajectory.points.size() < 2) {
    return trajectory;
  }

  const double lateral_offset =
    autoware::motion_utils::calcLateralOffset(trajectory.points, ego_pose.position);
  const size_t nearest_idx =
    autoware::motion_utils::findNearestIndex(trajectory.points, ego_pose.position);
  const double ego_yaw = tf2::getYaw(ego_pose.orientation);
  const double traj_yaw = tf2::getYaw(trajectory.points.at(nearest_idx).pose.orientation);
  const double signed_yaw_dev = autoware_utils::normalize_radian(ego_yaw - traj_yaw);
  const double abs_yaw_dev = std::abs(signed_yaw_dev);

  if (
    std::abs(lateral_offset) < params.minimum_shift_length &&
    abs_yaw_dev < params.minimum_shift_yaw) {
    return trajectory;
  }

  // Lower bound on velocity used to compute kappa0 = yaw_rate / v.
  const double clamped_velocity = std::max(std::abs(ego_velocity), params.min_speed_for_curvature);

  // Allowed lateral acceleration (comfort/safety budget).
  const double a_limit = params.lateral_accel_limit;

  // Peak curvature of the quintic polynomial (q=0, kappa0=0 case):
  //   kappa_max = |y''|_max = (10*sqrt(3)/3) * |d| / L^2  ~=  5.774 * |d| / L^2
  constexpr double kappa_coeff = 5.77;  // 10*sqrt(3)/3

  // Require a_lat = v^2 * kappa_max <= a_limit
  //   => L >= v * sqrt(kappa_coeff * |d| / a_limit)
  const double abs_d = std::abs(lateral_offset);
  double L = std::max(
    params.minimum_shift_distance,
    std::abs(clamped_velocity) * std::sqrt(kappa_coeff * abs_d / a_limit));

  double accumulated_length = 0.0;
  size_t merge_idx = nearest_idx;
  for (size_t i = nearest_idx; i + 1 < trajectory.points.size(); ++i) {
    accumulated_length += autoware_utils::calc_distance2d(
      trajectory.points.at(i).pose.position, trajectory.points.at(i + 1).pose.position);
    merge_idx = i + 1;
    if (accumulated_length >= L) {
      break;
    }
  }
  if (merge_idx >= trajectory.points.size() - 1) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("path_shift_to_ego"),
      "Trajectory is shorter than the target shift length (%.2f m < %.2f m). "
      "Terminal boundary conditions (y=0, y'=0, y''=0) cannot be satisfied; "
      "the shift polynomial will not converge smoothly at the merge point.",
      accumulated_length, L);
    L = accumulated_length;
    merge_idx = trajectory.points.size() - 2;
  }

  const double kappa0 = ego_yaw_rate / clamped_velocity;

  // Quintic polynomial: y(s) = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5
  // BCs: s=0: y=d, y'=tan(dtheta), y''=kappa0; s=L: y=0, y'=0, y''=0
  const double d = lateral_offset;
  const double q = std::tan(std::clamp(signed_yaw_dev, -M_PI / 4.0, M_PI / 4.0));
  const double L2 = L * L;
  const double L3 = L2 * L;
  const double a0 = d;
  const double a1 = q;
  const double a2 = kappa0 / 2.0;
  const double a3 = -(20.0 * d + 12.0 * q * L + 3.0 * kappa0 * L2) / (2.0 * L3);
  const double a4 = (30.0 * d + 16.0 * q * L + 3.0 * kappa0 * L2) / (2.0 * L3 * L);
  const double a5 = -(12.0 * d + 6.0 * q * L + kappa0 * L2) / (2.0 * L3 * L2);

  const double ref_velocity = trajectory.points.at(nearest_idx).longitudinal_velocity_mps;
  std::vector<TrajectoryPoint> shifted_points;

  TrajectoryPoint ego_pt;
  ego_pt.pose = ego_pose;
  ego_pt.longitudinal_velocity_mps = ref_velocity;
  shifted_points.push_back(ego_pt);

  for (double s = delta_arc_length; s < L; s += delta_arc_length) {
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double y_s = a0 + a1 * s + a2 * s2 + a3 * s3 + a4 * s3 * s + a5 * s3 * s2;
    const double yp_s = a1 + 2.0 * a2 * s + 3.0 * a3 * s2 + 4.0 * a4 * s3 + 5.0 * a5 * s3 * s;

    const auto base_pose =
      autoware::motion_utils::calcLongitudinalOffsetPose(trajectory.points, ego_pose.position, s);
    if (!base_pose) {
      break;
    }

    const double base_yaw = tf2::getYaw(base_pose->orientation);
    TrajectoryPoint pt;
    pt.pose.position.x = base_pose->position.x + y_s * (-std::sin(base_yaw));
    pt.pose.position.y = base_pose->position.y + y_s * std::cos(base_yaw);
    pt.pose.position.z = base_pose->position.z;
    pt.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(base_yaw + std::atan2(yp_s, 1.0));
    pt.longitudinal_velocity_mps = ref_velocity;
    shifted_points.push_back(pt);
  }

  for (size_t i = merge_idx; i < trajectory.points.size(); ++i) {
    shifted_points.push_back(trajectory.points.at(i));
  }

  Trajectory output;
  output.header = trajectory.header;
  output.points = shifted_points;
  return output;
}

}  // namespace autoware::minimum_rule_based_planner
