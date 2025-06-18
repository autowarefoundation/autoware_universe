// Copyright 2021 Tier IV, Inc.
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

#include "scene_no_stopping_area.hpp"
#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/debug.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace
{
const double marker_lifetime = 0.2;
using DebugData = no_stopping_area::DebugData;
using autoware_utils::append_marker_array;
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;

lanelet::BasicPoint3d get_centroid_point(const lanelet::BasicPolygon3d & poly)
{
  lanelet::BasicPoint3d p_sum{0.0, 0.0, 0.0};
  for (const auto & p : poly) {
    p_sum += p;
  }
  return p_sum / poly.size();
}

geometry_msgs::msg::Point to_msg(const lanelet::BasicPoint3d & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

visualization_msgs::msg::MarkerArray create_lanelet_info_marker_array(
  const lanelet::autoware::NoStoppingArea & no_stopping_area_reg_elem, const rclcpp::Time & now)
{
  visualization_msgs::msg::MarkerArray msg;

  // ID
  {
    auto marker = create_default_marker(
      "map", now, "no_stopping_area_id", static_cast<int32_t>(no_stopping_area_reg_elem.id()),
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, create_marker_scale(0.0, 0.0, 1.0),
      create_marker_color(1.0, 1.0, 1.0, 0.999));
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime);

    for (const auto & detection_area : no_stopping_area_reg_elem.noStoppingAreas()) {
      const auto poly = detection_area.basicPolygon();

      marker.pose.position = to_msg(poly.front());
      marker.pose.position.z += 2.0;
      marker.text = std::to_string(no_stopping_area_reg_elem.id());

      msg.markers.push_back(marker);
    }
  }

  // Polygon
  {
    auto marker = create_default_marker(
      "map", now, "no_stopping_area_polygon", static_cast<int32_t>(no_stopping_area_reg_elem.id()),
      visualization_msgs::msg::Marker::LINE_LIST, create_marker_scale(0.1, 0.0, 0.0),
      create_marker_color(0.1, 0.1, 1.0, 0.500));
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime);

    for (const auto & no_stopping_area : no_stopping_area_reg_elem.noStoppingAreas()) {
      const auto poly = no_stopping_area.basicPolygon();

      for (size_t i = 0; i < poly.size(); ++i) {
        const auto idx_front = i;
        const auto idx_back = (i == poly.size() - 1) ? 0 : i + 1;

        const auto & p_front = poly.at(idx_front);
        const auto & p_back = poly.at(idx_back);

        marker.points.push_back(to_msg(p_front));
        marker.points.push_back(to_msg(p_back));
      }
    }
    msg.markers.push_back(marker);
  }

  const auto & stop_line = no_stopping_area_reg_elem.stopLine();
  // Polygon to StopLine
  if (stop_line) {
    const auto stop_line_center_point =
      (stop_line.value().front().basicPoint() + stop_line.value().back().basicPoint()) / 2;
    auto marker = create_default_marker(
      "map", now, "no_stopping_area_correspondence",
      static_cast<int32_t>(no_stopping_area_reg_elem.id()),
      visualization_msgs::msg::Marker::LINE_STRIP, create_marker_scale(0.1, 0.0, 0.0),
      create_marker_color(0.1, 0.1, 1.0, 0.500));
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime);
    for (const auto & detection_area : no_stopping_area_reg_elem.noStoppingAreas()) {
      const auto poly = detection_area.basicPolygon();
      const auto centroid_point = get_centroid_point(poly);
      for (size_t i = 0; i < poly.size(); ++i) {
        marker.points.push_back(to_msg(centroid_point));
        marker.points.push_back(to_msg(stop_line_center_point));
      }
    }
    msg.markers.push_back(marker);
  }

  return msg;
}
}  // namespace

visualization_msgs::msg::MarkerArray NoStoppingAreaModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;
  const rclcpp::Time now = clock_->now();

  append_marker_array(
    create_lanelet_info_marker_array(no_stopping_area_reg_elem_, now), &debug_marker_array, now);

  if (!debug_data_.stuck_points.empty()) {
    append_marker_array(
      debug::createPointsMarkerArray(
        debug_data_.stuck_points, "stuck_points", module_id_, now, 0.3, 0.3, 0.3, 1.0, 1.0, 0.0),
      &debug_marker_array, now);
  }
  if (!debug_data_.stuck_vehicle_detect_area.points.empty()) {
    append_marker_array(
      debug::createPolygonMarkerArray(
        debug_data_.stuck_vehicle_detect_area, "stuck_vehicle_detect_area", module_id_, now, 0.1,
        0.1, 0.1, 1.0, 1.0, 0.0),
      &debug_marker_array, now);
  }
  if (!debug_data_.stop_line_detect_area.points.empty()) {
    append_marker_array(
      debug::createPolygonMarkerArray(
        debug_data_.stop_line_detect_area, "stop_line_detect_area", module_id_, now, 0.1, 0.1, 0.1,
        1.0, 1.0, 0.0),
      &debug_marker_array, now);
  }
  return debug_marker_array;
}

autoware::motion_utils::VirtualWalls NoStoppingAreaModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  wall.ns = std::to_string(module_id_) + "_";
  wall.text = "no_stopping_area";
  wall.style = autoware::motion_utils::VirtualWallType::stop;
  for (const auto & p : debug_data_.stop_poses) {
    wall.pose = autoware_utils::calc_offset_pose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}
}  // namespace autoware::behavior_velocity_planner
