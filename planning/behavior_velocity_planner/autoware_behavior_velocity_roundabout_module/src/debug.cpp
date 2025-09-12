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

#include "scene_roundabout.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/debug.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <string>
#include <tuple>
#include <vector>

namespace
{
using autoware_utils::append_marker_array;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_orientation;
using autoware_utils::create_marker_scale;

visualization_msgs::msg::MarkerArray createLaneletPolygonsMarkerArray(
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const std::string & ns,
  const int64_t lane_id, const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  int32_t i = 0;
  int32_t uid = autoware::behavior_velocity_planner::planning_utils::bitShift(lane_id);
  for (const auto & polygon : polygons) {
    visualization_msgs::msg::Marker marker{};
    marker.header.frame_id = "map";

    marker.ns = ns;
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation = create_marker_orientation(0, 0, 0, 1.0);
    marker.scale = create_marker_scale(0.1, 0.0, 0.0);
    marker.color = create_marker_color(r, g, b, 0.999);
    for (const auto & p : polygon) {
      geometry_msgs::msg::Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray createPoseMarkerArray(
  const geometry_msgs::msg::Pose & pose, const std::string & ns, const int64_t id, const double r,
  const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker_line{};
  marker_line.header.frame_id = "map";
  marker_line.ns = ns + "_line";
  marker_line.id = id;
  marker_line.lifetime = rclcpp::Duration::from_seconds(0.3);
  marker_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker_line.action = visualization_msgs::msg::Marker::ADD;
  marker_line.pose.orientation = create_marker_orientation(0, 0, 0, 1.0);
  marker_line.scale = create_marker_scale(0.2, 0.0, 0.0);
  marker_line.color = create_marker_color(r, g, b, 0.999);

  const double yaw = tf2::getYaw(pose.orientation);

  const double a = 3.0;
  geometry_msgs::msg::Point p0;
  p0.x = pose.position.x - a * std::sin(yaw);
  p0.y = pose.position.y + a * std::cos(yaw);
  p0.z = pose.position.z;
  marker_line.points.push_back(p0);

  geometry_msgs::msg::Point p1;
  p1.x = pose.position.x + a * std::sin(yaw);
  p1.y = pose.position.y - a * std::cos(yaw);
  p1.z = pose.position.z;
  marker_line.points.push_back(p1);

  msg.markers.push_back(marker_line);

  return msg;
}

constexpr std::tuple<float, float, float> green()
{
  constexpr uint64_t code = 0x5fa641;
  constexpr float r = static_cast<int>(code >> 16) / 255.0;
  constexpr float g = static_cast<int>((code << 48) >> 56) / 255.0;
  constexpr float b = static_cast<int>((code << 56) >> 56) / 255.0;
  return {r, g, b};
}

constexpr std::tuple<float, float, float> yellow()
{
  constexpr uint64_t code = 0xebce2b;
  constexpr float r = static_cast<int>(code >> 16) / 255.0;
  constexpr float g = static_cast<int>((code << 48) >> 56) / 255.0;
  constexpr float b = static_cast<int>((code << 56) >> 56) / 255.0;
  return {r, g, b};
}

constexpr std::tuple<float, float, float> red()
{
  constexpr uint64_t code = 0xba1c30;
  constexpr float r = static_cast<int>(code >> 16) / 255.0;
  constexpr float g = static_cast<int>((code << 48) >> 56) / 255.0;
  constexpr float b = static_cast<int>((code << 56) >> 56) / 255.0;
  return {r, g, b};
}
}  // namespace

namespace autoware::behavior_velocity_planner
{
using autoware_utils::append_marker_array;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_orientation;
using autoware_utils::create_marker_scale;

visualization_msgs::msg::MarkerArray RoundaboutModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  const auto now = this->clock_->now();

  if (debug_data_.attention_area) {
    append_marker_array(
      ::createLaneletPolygonsMarkerArray(
        debug_data_.attention_area.value(), "attention_area", lane_id_, 0.0, 1.0, 0.0),
      &debug_marker_array);
  }

  if (debug_data_.adjacent_area) {
    append_marker_array(
      ::createLaneletPolygonsMarkerArray(
        debug_data_.adjacent_area.value(), "adjacent_area", lane_id_, 0.913, 0.639, 0.149),
      &debug_marker_array);
  }

  if (debug_data_.first_attention_area) {
    append_marker_array(
      ::createLaneletPolygonsMarkerArray(
        {debug_data_.first_attention_area.value()}, "first_attention_area", lane_id_, 1, 0.647,
        0.0),
      &debug_marker_array, now);
  }

  if (debug_data_.ego_lane) {
    append_marker_array(
      ::createLaneletPolygonsMarkerArray(
        {debug_data_.ego_lane.value()}, "ego_lane", lane_id_, 1, 0.647, 0.0),
      &debug_marker_array, now);
  }

  if (debug_data_.candidate_collision_ego_lane_polygon) {
    append_marker_array(
      debug::createPolygonMarkerArray(
        debug_data_.candidate_collision_ego_lane_polygon.value(),
        "candidate_collision_ego_lane_polygon", module_id_, now, 0.3, 0.0, 0.0, 0.5, 0.0, 0.0),
      &debug_marker_array, now);
  }

  if (debug_data_.candidate_collision_object_polygon) {
    append_marker_array(
      debug::createPolygonMarkerArray(
        debug_data_.candidate_collision_object_polygon.value(),
        "candidate_collision_object_polygon", module_id_, now, 0.3, 0.0, 0.0, 0.5, 0.0, 0.0),
      &debug_marker_array, now);
  }

  static constexpr auto green = ::green();
  static constexpr auto yellow = ::yellow();
  static constexpr auto red = ::red();

  append_marker_array(
    debug::createObjectsMarkerArray(
      debug_data_.unsafe_targets, "unsafe_targets", module_id_, now, std::get<0>(green),
      std::get<1>(green), std::get<2>(green)),
    &debug_marker_array, now);

  append_marker_array(
    debug::createObjectsMarkerArray(
      debug_data_.misjudge_targets, "misjudge_targets", module_id_, now, std::get<0>(yellow),
      std::get<1>(yellow), std::get<2>(yellow)),
    &debug_marker_array, now);

  append_marker_array(
    debug::createObjectsMarkerArray(
      debug_data_.too_late_detect_targets, "too_late_detect_targets", module_id_, now,
      std::get<0>(red), std::get<1>(red), std::get<2>(red)),
    &debug_marker_array, now);

  if (debug_data_.first_pass_judge_wall_pose) {
    const double r = debug_data_.passed_first_pass_judge ? 1.0 : 0.0;
    const double g = debug_data_.passed_first_pass_judge ? 0.0 : 1.0;
    append_marker_array(
      ::createPoseMarkerArray(
        debug_data_.first_pass_judge_wall_pose.value(), "first_pass_judge_wall_pose", module_id_, r,
        g, 0.0),
      &debug_marker_array, now);
  }

  return debug_marker_array;
}

autoware::motion_utils::VirtualWalls RoundaboutModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;

  if (debug_data_.collision_stop_wall_pose) {
    wall.style = autoware::motion_utils::VirtualWallType::stop;
    wall.text = "roundabout";
    wall.ns = "roundabout" + std::to_string(module_id_) + "_";
    wall.pose = debug_data_.collision_stop_wall_pose.value();
    virtual_walls.push_back(wall);
  }
  if (debug_data_.too_late_stop_wall_pose) {
    wall.style = autoware::motion_utils::VirtualWallType::pass;
    wall.text = "roundabout";
    wall.detail = "too late to stop";
    wall.ns = "roundabout" + std::to_string(module_id_) + "_";
    wall.pose = debug_data_.too_late_stop_wall_pose.value();
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

}  // namespace autoware::behavior_velocity_planner
