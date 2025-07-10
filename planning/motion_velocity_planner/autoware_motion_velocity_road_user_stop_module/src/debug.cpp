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

#include <lanelet2_core/geometry/Polygon.h>

#include <string>

namespace autoware::motion_velocity_planner
{

namespace
{
using autoware::universe_utils::appendMarkerArray;
using autoware::universe_utils::createDefaultMarker;

MarkerArray createLaneletPolygonsMarkerArray(
  const lanelet::ConstLanelets & lanelets, const std::string & ns,
  const std::array<double, 3> & color, const rclcpp::Clock::SharedPtr clock)
{
  MarkerArray msg;

  for (const auto & lanelet : lanelets) {
    // Use lanelet ID directly to ensure uniqueness
    const int32_t marker_id = static_cast<int32_t>(lanelet.id());
    Marker marker = createDefaultMarker(
      "map", clock->now(), ns, marker_id, Marker::LINE_STRIP,
      autoware::universe_utils::createMarkerScale(0.1, 0, 0),
      autoware::universe_utils::createMarkerColor(color[0], color[1], color[2], 0.999));

    marker.lifetime = rclcpp::Duration::from_seconds(0.3);

    // Add lanelet polygon points
    const auto & polygon = lanelet.polygon3d();
    for (const auto & point : polygon) {
      geometry_msgs::msg::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      marker.points.push_back(p);
    }

    // Close the polygon
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }

    msg.markers.push_back(marker);
  }

  return msg;
}

}  // namespace

MarkerArray RoadUserStopModule::createDebugMarkerArray() const
{
  MarkerArray debug_marker_array;

  if (params_.debug.publish_debug_markers) {
    // Visualize ego lanelets in pink
    if (!debug_data_.ego_lanelets.empty()) {
      const auto ego_lanelets_markers = createLaneletPolygonsMarkerArray(
        debug_data_.ego_lanelets, "ego_lanelets", {1.0, 0.0, 1.0});  // Pink color
      appendMarkerArray(ego_lanelets_markers, &debug_marker_array);
    }

    // Visualize adjacent lanelets in orange
    if (!debug_data_.adjacent_lanelets.empty()) {
      const auto adjacent_lanelets_markers = createLaneletPolygonsMarkerArray(
        debug_data_.adjacent_lanelets, "adjacent_lanelets", {1.0, 0.5, 0.0});  // Orange color
      appendMarkerArray(adjacent_lanelets_markers, &debug_marker_array);
    }

    // Visualize masked adjacent lanelets in green
    if (!debug_data_.masked_adjacent_lanelets.empty()) {
      const auto masked_adjacent_lanelets_markers = createLaneletPolygonsMarkerArray(
        debug_data_.masked_adjacent_lanelets, "masked_adjacent_lanelets",
        {0.0, 1.0, 0.0});  // Green color
      appendMarkerArray(masked_adjacent_lanelets_markers, &debug_marker_array);
    }

    // Visualize trajectory polygons in yellow
    if (!debug_data_.trajectory_polygons.empty()) {
      int traj_poly_id = 0;
      for (const auto & polygon : debug_data_.trajectory_polygons) {
        Marker traj_poly_marker = createDefaultMarker(
          "map", clock_->now(), "trajectory_polygons", traj_poly_id++, Marker::LINE_STRIP,
          autoware::universe_utils::createMarkerScale(0.1, 0, 0),
          autoware::universe_utils::createMarkerColor(1.0, 1.0, 0.0, 0.5));  // Yellow color

        traj_poly_marker.lifetime = rclcpp::Duration::from_seconds(0.3);

        // Add polygon points
        for (const auto & point : polygon.outer()) {
          geometry_msgs::msg::Point p;
          p.x = point.x();
          p.y = point.y();
          p.z = 0.0;
          traj_poly_marker.points.push_back(p);
        }

        debug_marker_array.markers.push_back(traj_poly_marker);
      }
    }

    // Visualize masked adjacent polygons in cyan
    if (!debug_data_.masked_adjacent_polygons.empty()) {
      int poly_id = 0;
      for (const auto & polygon : debug_data_.masked_adjacent_polygons) {
        Marker poly_marker = createDefaultMarker(
          "map", clock_->now(), "masked_adjacent_polygons", poly_id++, Marker::LINE_STRIP,
          autoware::universe_utils::createMarkerScale(0.15, 0, 0),
          autoware::universe_utils::createMarkerColor(0.0, 1.0, 1.0, 0.8));  // Cyan color

        poly_marker.lifetime = rclcpp::Duration::from_seconds(0.3);

        // Add polygon points
        for (const auto & point : polygon.outer()) {
          geometry_msgs::msg::Point p;
          p.x = point.x();
          p.y = point.y();
          p.z = 0.0;
          poly_marker.points.push_back(p);
        }

        debug_marker_array.markers.push_back(poly_marker);
      }
    }

    // Visualize stop point if exists
    if (debug_data_.stop_point) {
      Marker stop_marker = createDefaultMarker(
        "map", clock_->now(), "stop_point", 0, Marker::SPHERE,
        autoware::universe_utils::createMarkerScale(1.0, 1.0, 1.0),
        autoware::universe_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
      stop_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
      stop_marker.pose.position = debug_data_.stop_point.value();
      stop_marker.pose.orientation.w = 1.0;
      debug_marker_array.markers.push_back(stop_marker);
    }

    // Visualize gradual stop positions
    int gradual_stop_id = 0;
    for (const auto & gradual_stop_pos : debug_data_.gradual_stop_positions) {
      Marker gradual_stop_marker = createDefaultMarker(
        "map", clock_->now(), "gradual_stop_positions", gradual_stop_id++, Marker::SPHERE,
        autoware::universe_utils::createMarkerScale(0.8, 0.8, 0.8),
        autoware::universe_utils::createMarkerColor(1.0, 1.0, 0.0, 0.8));  // yellow
      gradual_stop_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
      gradual_stop_marker.pose.position = gradual_stop_pos;
      gradual_stop_marker.pose.orientation.w = 1.0;
      debug_marker_array.markers.push_back(gradual_stop_marker);
    }

    // Visualize filtered objects
    int obj_id = 0;
    for (const auto & object : debug_data_.filtered_objects) {
      Marker obj_marker = createDefaultMarker(
        "map", clock_->now(), "filtered_objects", obj_id++, Marker::CYLINDER,
        autoware::universe_utils::createMarkerScale(0.5, 0.5, 2.0),
        autoware::universe_utils::createMarkerColor(0.0, 0.0, 1.0, 0.8));

      obj_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
      obj_marker.pose = object.kinematics.initial_pose_with_covariance.pose;

      debug_marker_array.markers.push_back(obj_marker);
    }
  }

  return debug_marker_array;
}

MarkerArray RoadUserStopModule::createLaneletPolygonsMarkerArray(
  const lanelet::ConstLanelets & lanelets, const std::string & ns,
  const std::array<double, 3> & color) const
{
  return ::autoware::motion_velocity_planner::createLaneletPolygonsMarkerArray(
    lanelets, ns, color, clock_);
}

}  // namespace autoware::motion_velocity_planner
