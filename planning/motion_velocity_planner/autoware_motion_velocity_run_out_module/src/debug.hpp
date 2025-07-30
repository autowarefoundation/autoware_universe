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

#ifndef DEBUG_HPP_
#define DEBUG_HPP_

#include "parameters.hpp"
#include "types.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <rclcpp/duration.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

inline MarkerArray make_debug_ego_footprint_markers(const run_out::TrajectoryCornerFootprint & ego)
{
  MarkerArray markers;
  Marker m;
  m.header.frame_id = "map";
  m.type = Marker::LINE_STRIP;
  m.color = universe_utils::createMarkerColor(0.0, 1.0, 0.0, 1.0);
  m.scale.x = 0.2;
  m.ns = "ego_footprint_front_left";
  for (const auto & p : ego.predicted_path_footprint.corner_linestrings[front_left]) {
    m.points.push_back(universe_utils::createPoint(p.x(), p.y(), 0.0));
  }
  markers.markers.push_back(m);
  m.ns = "ego_footprint_front_right";
  m.points.clear();
  for (const auto & p : ego.predicted_path_footprint.corner_linestrings[front_right]) {
    m.points.push_back(universe_utils::createPoint(p.x(), p.y(), 0.0));
  }
  markers.markers.push_back(m);
  m.ns = "ego_footprint_rear_right";
  m.points.clear();
  for (const auto & p : ego.predicted_path_footprint.corner_linestrings[rear_right]) {
    m.points.push_back(universe_utils::createPoint(p.x(), p.y(), 0.0));
  }
  markers.markers.push_back(m);
  m.ns = "ego_footprint_rear_left";
  m.points.clear();
  for (const auto & p : ego.predicted_path_footprint.corner_linestrings[rear_left]) {
    m.points.push_back(universe_utils::createPoint(p.x(), p.y(), 0.0));
  }
  markers.markers.push_back(m);
  return markers;
}

inline MarkerArray make_debug_objects_footprint_markers(
  const std::vector<run_out::Object> & objects)
{
  MarkerArray markers;
  Marker m;
  m.header.frame_id = "map";
  m.type = Marker::LINE_STRIP;
  m.color = universe_utils::createMarkerColor(0.0, 1.0, 0.0, 1.0);
  m.scale.x = 0.2;
  m.type = Marker::LINE_LIST;
  m.ns = "objects_footprints";
  m.color.r = 1.0;
  for (const auto & object : objects) {
    for (const auto & footprint : object.predicted_path_footprints) {
      const auto & f = footprint.predicted_path_footprint;
      for (auto i = 0UL; i + 1 < f.corner_linestrings[front_left].size(); ++i) {
        m.points.push_back(
          universe_utils::createPoint(
            f.corner_linestrings[front_left][i].x(), f.corner_linestrings[front_left][i].y(), 0.0));
        m.points.push_back(
          universe_utils::createPoint(
            f.corner_linestrings[front_left][i + 1].x(),
            f.corner_linestrings[front_left][i + 1].y(), 0.0));
        m.points.push_back(
          universe_utils::createPoint(
            f.corner_linestrings[front_right][i].x(), f.corner_linestrings[front_right][i].y(),
            0.0));
        m.points.push_back(
          universe_utils::createPoint(
            f.corner_linestrings[front_right][i + 1].x(),
            f.corner_linestrings[front_right][i + 1].y(), 0.0));
        m.points.push_back(
          universe_utils::createPoint(
            f.corner_linestrings[rear_left][i].x(), f.corner_linestrings[rear_left][i].y(), 0.0));
        m.points.push_back(
          universe_utils::createPoint(
            f.corner_linestrings[rear_left][i + 1].x(), f.corner_linestrings[rear_left][i + 1].y(),
            0.0));
        m.points.push_back(
          universe_utils::createPoint(
            f.corner_linestrings[rear_right][i].x(), f.corner_linestrings[rear_right][i].y(), 0.0));
        m.points.push_back(
          universe_utils::createPoint(
            f.corner_linestrings[rear_right][i + 1].x(),
            f.corner_linestrings[rear_right][i + 1].y(), 0.0));
      }
    }
  }
  markers.markers.push_back(m);
  return markers;
}

inline MarkerArray make_debug_collisions_markers(const std::vector<Object> & objects)
{
  MarkerArray markers;
  Marker m;
  m.header.frame_id = "base_link";
  m.ns = "collisions_table";
  m.type = Marker::TEXT_VIEW_FACING;
  m.color = universe_utils::createMarkerColor(1.0, 1.0, 0.0, 0.75);
  m.scale = universe_utils::createMarkerScale(0.1, 0.0, 0.5);
  std::stringstream ss;
  ss << std::setprecision(4);
  ss << std::fixed;
  ss << std::left;
  ss << std::setw(10) << "object id";
  ss << "|";
  ss << std::setw(10) << "collision";
  ss << "|";
  ss << std::setw(20) << "ego_time_interval";
  ss << "|";
  ss << std::setw(20) << "object_time_interval";
  ss << "|";
  ss << std::setw(20) << "ego_collision_time";
  ss << "\n";
  for (const auto & o : objects) {
    for (const auto & col : o.collisions) {
      ss << std::setw(10) << o.uuid.substr(0, 5) + "|";
      if (col.type == collision) ss << std::setw(10) << "C|";
      if (col.type == ignored_collision) ss << std::setw(10) << "IC|";
      if (col.type == pass_first_no_collision) ss << std::setw(10) << "P-|";
      if (col.type == no_collision) ss << std::setw(10) << "-|";
      ss << std::setw(20) << col.ego_time_interval << "|";
      ss << std::setw(20) << col.object_time_interval << "|";
      ss << std::setw(20)
         << (col.type == no_collision ? "-" : std::to_string(col.ego_collision_time)) << "|";
      ss << std::setw(20) << col.explanation;
      ss << "\n";
    }
  }
  m.text = ss.str();
  markers.markers.push_back(m);
  m.ns = "collisions_points";
  m.header.frame_id = "map";
  m.type = Marker::POINTS;
  m.color = universe_utils::createMarkerColor(1.0, 0.0, 0.0, 0.75);
  m.scale = universe_utils::createMarkerScale(0.5, 0.5, 0.5);
  for (const auto & o : objects) {
    for (const auto & c : o.collisions) {
      for (const auto & p : {
             c.object_time_interval.first_intersection.intersection,
             c.object_time_interval.last_intersection.intersection,
             c.ego_time_interval.first_intersection.intersection,
             c.ego_time_interval.last_intersection.intersection,
           }) {
        m.points.push_back(universe_utils::createPoint(p.x(), p.y(), 0.0));
      }
    }
  }
  markers.markers.push_back(m);
  return markers;
}

inline MarkerArray make_debug_decisions_markers(const ObjectDecisionsTracker & decisions_tracker)
{
  constexpr auto decision_type_to_str = [](const auto type) {
    switch (type) {
      case stop:
        return "STOP|";
      case slowdown:
        return "SLOW|";
      default:
        return "-|";
    }
  };
  const auto collision_to_str = [](const auto & c) {
    if (!c.has_value()) {
      return "";
    }
    switch (c->type) {
      case collision:
        return "C|";
      case ignored_collision:
        return "IC|";
      case pass_first_no_collision:
        return "P|";
      case no_collision:
      default:
        return "-|";
    }
  };
  MarkerArray markers;
  Marker m;
  m.header.frame_id = "base_link";
  m.ns = "decisions";
  m.pose.position.x = 10.0;
  m.type = Marker::TEXT_VIEW_FACING;
  m.color = universe_utils::createMarkerColor(1.0, 1.0, 0.0, 0.75);
  m.scale = universe_utils::createMarkerScale(0.1, 0.0, 0.5);
  std::stringstream ss;
  ss << std::setprecision(2);
  ss << std::fixed;
  ss << std::left;
  ss << std::setw(10) << "object uuid |";
  ss << std::setw(60) << " decisions |\n";
  ss << " |\n";
  for (const auto & [object, history] : decisions_tracker.history_per_object) {
    ss << std::setw(10) << object.substr(0, 5) + "|";
    for (auto i = 0UL; i < history.times.size(); ++i) {
      std::stringstream line;
      line << std::fixed;
      line << std::setprecision(2);
      line << history.times[i] << ":";
      line << decision_type_to_str(history.decisions[i].type);
      line << collision_to_str(history.decisions[i].collision);
      ss << line.str();
    }
    ss << " | " << history.decisions.back().explanation;
    ss << "\n";
  }
  m.text = ss.str();
  markers.markers.push_back(m);

  return markers;
}

inline motion_utils::VirtualWalls create_virtual_walls(
  const VelocityPlanningResult & result,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const double front_offset)
{
  motion_utils::VirtualWalls virtual_walls;
  motion_utils::VirtualWall wall;
  wall.text = "run_out (mvp)";
  wall.longitudinal_offset = front_offset;
  wall.style = motion_utils::VirtualWallType::stop;
  for (const auto & stop_point : result.stop_points) {
    const auto length = motion_utils::calcSignedArcLength(trajectory, 0, stop_point);
    wall.pose = motion_utils::calcInterpolatedPose(trajectory, length);
    virtual_walls.push_back(wall);
  }
  wall.style = motion_utils::VirtualWallType::slowdown;
  wall.text += " [SLOW]";
  for (const auto & slowdown : result.slowdown_intervals) {
    const auto length = motion_utils::calcSignedArcLength(trajectory, 0, slowdown.from);
    wall.pose = motion_utils::calcInterpolatedPose(trajectory, length);
    wall.pose.position = slowdown.from;
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

inline MarkerArray make_debug_min_stop_marker(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const double time_to_stop)
{
  MarkerArray markers;
  Marker m;
  m.header.frame_id = "map";
  m.ns = "min_stop";
  m.type = Marker::SPHERE;
  m.color = universe_utils::createMarkerColor(1.0, 0.0, 0.0, 0.75);
  m.scale = universe_utils::createMarkerScale(0.5, 0.5, 0.5);

  const auto it_after_time = std::find_if(
    trajectory.begin(), trajectory.end(),
    [&](const autoware_planning_msgs::msg::TrajectoryPoint & p) {
      return rclcpp::Duration(p.time_from_start).seconds() > time_to_stop;
    });
  if (it_after_time == trajectory.begin()) {
    m.pose = it_after_time->pose;
  } else {
    const auto it_before_time = std::prev(it_after_time);
    const auto delta = (rclcpp::Duration(it_after_time->time_from_start) -
                        rclcpp::Duration(it_before_time->time_from_start))
                         .seconds();
    const auto diff = time_to_stop - rclcpp::Duration(it_before_time->time_from_start).seconds();
    const auto ratio = diff / delta;
    m.pose = universe_utils::calcInterpolatedPose(it_before_time->pose, it_after_time->pose, ratio);
  }
  markers.markers.push_back(m);
  return markers;
}

inline MarkerArray make_debug_filtering_data_marker(const FilteringData & data)
{
  MarkerArray markers;
  Marker m;
  m.header.frame_id = "map";
  m.ns = "filtering_data_cut_predicted_paths";
  m.type = Marker::LINE_LIST;
  m.color = universe_utils::createMarkerColor(1.0, 0.0, 0.0, 0.75);
  m.scale = universe_utils::createMarkerScale(0.2, 0.2, 0.2);
  geometry_msgs::msg::Point p;
  for (const auto & segment : data.cut_predicted_paths_segments) {
    p.x = segment.first.x();
    p.y = segment.first.y();
    m.points.push_back(p);
    p.x = segment.second.x();
    p.y = segment.second.y();
    m.points.push_back(p);
  }
  markers.markers.push_back(m);
  m.ns = "filtering_data_strict_cut_predicted_paths";
  m.points.clear();
  m.type = Marker::LINE_LIST;
  m.color = universe_utils::createMarkerColor(1.0, 1.0, 0.0, 0.75);
  m.scale = universe_utils::createMarkerScale(0.2, 0.2, 0.2);
  for (const auto & segment : data.strict_cut_predicted_paths_segments) {
    p.x = segment.first.x();
    p.y = segment.first.y();
    m.points.push_back(p);
    p.x = segment.second.x();
    p.y = segment.second.y();
    m.points.push_back(p);
  }
  markers.markers.push_back(m);
  m.ns = "filtering_data_ignore_objects";
  m.points.clear();
  m.color = universe_utils::createMarkerColor(0.0, 0.0, 1.0, 0.75);
  for (const auto & poly : data.ignore_objects_polygons) {
    for (auto i = 0UL; i + 1 < poly.size(); ++i) {
      p.x = poly[i].x();
      p.y = poly[i].y();
      m.points.push_back(p);
      p.x = poly[i + 1].x();
      p.y = poly[i + 1].y();
      m.points.push_back(p);
    }
  }
  markers.markers.push_back(m);
  m.ns = "filtering_data_ignore_collisions";
  m.points.clear();
  m.color = universe_utils::createMarkerColor(0.0, 1.0, 0.0, 0.75);
  for (const auto & poly : data.ignore_collisions_polygons) {
    for (auto i = 0UL; i + 1 < poly.size(); ++i) {
      p.x = poly[i].x();
      p.y = poly[i].y();
      m.points.push_back(p);
      p.x = poly[i + 1].x();
      p.y = poly[i + 1].y();
      m.points.push_back(p);
    }
  }
  markers.markers.push_back(m);
  return markers;
}

inline MarkerArray make_debug_markers(
  const TrajectoryCornerFootprint & ego_footprint, const std::vector<Object> & filtered_objects,
  const ObjectDecisionsTracker & decisions_tracker,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & smoothed_trajectory_points,
  const FilteringData & filtering_data, const Parameters & params)
{
  MarkerArray markers;
  const auto concat = [&](MarkerArray && a) {
    markers.markers.insert(
      markers.markers.end(), std::make_move_iterator(a.markers.begin()),
      std::make_move_iterator(a.markers.end()));
  };
  if (params.debug.enabled_markers.ego_footprint) {
    concat(run_out::make_debug_ego_footprint_markers(ego_footprint));
  }
  if (params.debug.enabled_markers.objects) {
    concat(run_out::make_debug_objects_footprint_markers(filtered_objects));
  }
  if (params.debug.enabled_markers.collisions) {
    concat(run_out::make_debug_collisions_markers(filtered_objects));
  }
  if (params.debug.enabled_markers.decisions) {
    concat(run_out::make_debug_decisions_markers(decisions_tracker));
  }
  if (params.debug.enabled_markers.filtering_data) {
    concat(run_out::make_debug_filtering_data_marker(filtering_data));
  }
  concat(
    run_out::make_debug_min_stop_marker(
      smoothed_trajectory_points,
      params.ignore_collision_conditions.if_ego_arrives_first_and_cannot_stop
        .calculated_stop_time_limit));
  return markers;
}

}  // namespace autoware::motion_velocity_planner::run_out

#endif  // DEBUG_HPP_
