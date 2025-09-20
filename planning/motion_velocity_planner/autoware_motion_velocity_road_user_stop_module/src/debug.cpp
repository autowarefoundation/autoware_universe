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

#include "road_user_stop_module.hpp"

#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>

#include <std_msgs/msg/color_rgba.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <string>

namespace autoware::motion_velocity_planner
{

namespace
{
using autoware::universe_utils::appendMarkerArray;
using autoware::universe_utils::createDefaultMarker;

void add_polygon_to_marker(Marker & marker, const Polygon2d & polygon, const double z = 0.0)
{
  for (const auto & point : polygon.outer()) {
    Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = z;
    marker.points.push_back(p);
  }

  // Close the polygon
  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
  }
}

}  // namespace

MarkerArray RoadUserStopModule::create_debug_marker_array() const
{
  autoware_utils_debug::ScopedTimeTrack st_debug("create_debug_marker_array", *time_keeper_);
  MarkerArray debug_marker_array;

  if (!debug_data_.ego_lanelets.empty()) {
    autoware_utils_debug::ScopedTimeTrack st_debug_marker(
      "create_debug_marker_array/ego_lanelets", *time_keeper_);
    std_msgs::msg::ColorRGBA pink_color;
    pink_color.r = 1.0;
    pink_color.g = 0.0;
    pink_color.b = 1.0;
    pink_color.a = 0.999;
    const auto ego_lanelets_markers = lanelet::visualization::laneletsAsTriangleMarkerArray(
      "ego_lanelets", debug_data_.ego_lanelets, pink_color);

    appendMarkerArray(ego_lanelets_markers, &debug_marker_array);
  }

  if (!debug_data_.trajectory_polygons.empty()) {
    autoware_utils_debug::ScopedTimeTrack st_debug_marker(
      "create_debug_marker_array/trajectory_polygons", *time_keeper_);
    int traj_poly_id = 0;
    for (const auto & polygon : debug_data_.trajectory_polygons) {
      Marker traj_poly_marker = createDefaultMarker(
        "map", clock_->now(), "trajectory_polygons", traj_poly_id++, Marker::LINE_STRIP,
        autoware::universe_utils::createMarkerScale(0.1, 0, 0),
        autoware::universe_utils::createMarkerColor(1.0, 1.0, 0.0, 0.5));  // Yellow color

      traj_poly_marker.lifetime = rclcpp::Duration::from_seconds(0.3);

      add_polygon_to_marker(traj_poly_marker, polygon);

      debug_marker_array.markers.push_back(traj_poly_marker);
    }
  }

  if (!debug_data_.object_polygons.empty()) {
    autoware_utils_debug::ScopedTimeTrack st_debug_marker(
      "create_debug_marker_array/object_polygons", *time_keeper_);
    int obj_poly_id = 0;
    for (const auto & polygon : debug_data_.object_polygons) {
      Marker obj_poly_marker = createDefaultMarker(
        "map", clock_->now(), "object_polygons", obj_poly_id++, Marker::LINE_STRIP,
        autoware::universe_utils::createMarkerScale(0.15, 0, 0),
        autoware::universe_utils::createMarkerColor(0.8, 0.0, 0.8, 0.9));

      obj_poly_marker.lifetime = rclcpp::Duration::from_seconds(0.3);

      add_polygon_to_marker(obj_poly_marker, polygon);

      debug_marker_array.markers.push_back(obj_poly_marker);
    }
  }

  if (!debug_data_.polygons_for_vru.empty()) {
    autoware_utils_debug::ScopedTimeTrack st_debug_marker(
      "create_debug_marker_array/polygons_for_vru", *time_keeper_);
    int vru_poly_id = 0;
    for (const auto & polygon : debug_data_.polygons_for_vru) {
      Marker vru_poly_marker = createDefaultMarker(
        "map", clock_->now(), "polygons_for_vru", vru_poly_id++, Marker::LINE_STRIP,
        autoware::universe_utils::createMarkerScale(0.15, 0, 0),
        autoware::universe_utils::createMarkerColor(0.5, 1.0, 0.5, 0.6));

      vru_poly_marker.lifetime = rclcpp::Duration::from_seconds(0.3);

      add_polygon_to_marker(vru_poly_marker, polygon);

      debug_marker_array.markers.push_back(vru_poly_marker);
    }
  }

  if (!debug_data_.polygons_for_opposing_traffic.empty()) {
    autoware_utils_debug::ScopedTimeTrack st_debug_marker(
      "create_debug_marker_array/polygons_for_opposing_traffic", *time_keeper_);
    int opposing_poly_id = 0;
    for (const auto & polygon : debug_data_.polygons_for_opposing_traffic) {
      Marker opposing_poly_marker = createDefaultMarker(
        "map", clock_->now(), "polygons_for_opposing_traffic", opposing_poly_id++,
        Marker::LINE_STRIP, autoware::universe_utils::createMarkerScale(0.15, 0, 0),
        autoware::universe_utils::createMarkerColor(1.0, 0.5, 0.5, 0.6));

      opposing_poly_marker.lifetime = rclcpp::Duration::from_seconds(0.3);

      add_polygon_to_marker(opposing_poly_marker, polygon);

      debug_marker_array.markers.push_back(opposing_poly_marker);
    }
  }

  return debug_marker_array;
}

}  // namespace autoware::motion_velocity_planner
