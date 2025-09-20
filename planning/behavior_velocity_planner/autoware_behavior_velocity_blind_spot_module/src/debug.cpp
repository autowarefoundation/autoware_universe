// Copyright 2025 Tier IV, Inc.
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

#include "autoware/behavior_velocity_blind_spot_module/scene.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/debug.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <range/v3/view/enumerate.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <string>
#include <tuple>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_utils_visualization::append_marker_array;
using autoware_utils_visualization::create_default_marker;
using autoware_utils_visualization::create_marker_color;
using autoware_utils_visualization::create_marker_orientation;
using autoware_utils_visualization::create_marker_scale;

namespace
{

visualization_msgs::msg::MarkerArray createLaneletPolygonsMarkerArray(
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const std::string & ns,
  const int64_t lane_id, const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  int32_t i = 0;
  int32_t uid = planning_utils::bitShift(lane_id);
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

visualization_msgs::msg::MarkerArray create_lanelet_linestring_marker_array(
  const lanelet::ConstLineString3d & linestring, const std::string & ns, const int64_t lane_id,
  const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  const int32_t uid = planning_utils::bitShift(lane_id);
  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";

  marker.ns = ns;
  marker.id = uid;
  marker.lifetime = rclcpp::Duration::from_seconds(0.3);
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation = create_marker_orientation(0, 0, 0, 1.0);
  marker.scale = create_marker_scale(0.1, 0.0, 0.0);
  marker.color = create_marker_color(r, g, b, 0.999);
  for (const auto & p : linestring) {
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    marker.points.push_back(point);
  }
  msg.markers.push_back(marker);

  return msg;
}

}  // namespace

autoware::motion_utils::VirtualWalls BlindSpotModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;

  if (debug_data_.virtual_wall_pose) {
    autoware::motion_utils::VirtualWall wall;
    wall.text = "blind_spot";
    wall.pose = debug_data_.virtual_wall_pose.value();
    wall.ns = std::to_string(module_id_) + "_";
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

static constexpr std::tuple<float, float, float> red()
{
  constexpr uint64_t code = 0xba1c30;
  constexpr float r = static_cast<int>(code >> 16) / 255.0;
  constexpr float g = static_cast<int>((code << 48) >> 56) / 255.0;
  constexpr float b = static_cast<int>((code << 56) >> 56) / 255.0;
  return {r, g, b};
}

visualization_msgs::msg::MarkerArray BlindSpotModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  const auto now = this->clock_->now();

  if (debug_data_.attention_area) {
    append_marker_array(
      createLaneletPolygonsMarkerArray(
        {debug_data_.attention_area.value()}, "attention_area", module_id_, 0.0, 0.8, 0.15),
      &debug_marker_array, now);
  }
  if (debug_data_.path_polygon) {
    append_marker_array(
      createLaneletPolygonsMarkerArray(
        {debug_data_.path_polygon.value()}, "path_polygon", module_id_, 1.0, 0.5, 0.0),
      &debug_marker_array, now);
  }
  if (debug_data_.virtual_blind_lane_boundary_after_turning) {
    append_marker_array(
      create_lanelet_linestring_marker_array(
        debug_data_.virtual_blind_lane_boundary_after_turning.value(),
        "virtual_blind_lane_boundary_after_turning", module_id_, 0.7, 0.3, 0.7),
      &debug_marker_array, now);
  }
  if (debug_data_.virtual_ego_straight_path_after_turning) {
    append_marker_array(
      create_lanelet_linestring_marker_array(
        debug_data_.virtual_ego_straight_path_after_turning.value(),
        "virtual_ego_straight_path_after_turning", module_id_, 0.7, 0.3, 0.7),
      &debug_marker_array, now);
  }
  if (debug_data_.ego_passage_interval && debug_data_.virtual_ego_straight_path_after_turning) {
    const auto & [ego_passage_start, ego_passage_end] = debug_data_.ego_passage_interval.value();
    auto marker = create_default_marker(
      "map", now, "ego_passage_interval", module_id_,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, create_marker_scale(0.0, 0.0, 1.0),
      create_marker_color(1.0, 1.0, 1.0, 0.999));
    std::stringstream ss;
    ss << "[" << ego_passage_start << ", " << ego_passage_end << "]";
    if (debug_data_.critical_time) {
      ss << ", collision: " << debug_data_.critical_time.value();
    }
    marker.text = ss.str();
    const auto & line = debug_data_.virtual_ego_straight_path_after_turning.value();
    marker.pose.position.x = line[0].x();
    marker.pose.position.y = line[0].y();
    marker.pose.position.z = line[0].z();
    debug_marker_array.markers.push_back(marker);
  }
  if (debug_data_.unsafe_objects) {
    const auto & unsafe_objects = debug_data_.unsafe_objects.value();
    autoware_perception_msgs::msg::PredictedObjects objects;
    for (const auto & [i, unsafe_object] : ranges::views::enumerate(unsafe_objects)) {
      objects.objects.push_back(unsafe_object.object);
      auto marker = create_default_marker(
        "map", now, "unsafe_objects", module_id_ + i + 1,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING, create_marker_scale(0.0, 0.0, 1.0),
        create_marker_color(1.0, 1.0, 1.0, 0.999));
      std::stringstream ss;
      const auto & interval = unsafe_object.object_passage_interval;
      ss << "[" << std::get<0>(interval) << ", " << std::get<1>(interval) << "]";
      marker.text = ss.str();
      marker.pose.position =
        unsafe_object.object.kinematics.initial_pose_with_covariance.pose.position;
      debug_marker_array.markers.push_back(marker);
    }
    append_marker_array(
      debug::createObjectsMarkerArray(
        objects, "unsafe_objects", module_id_, now, std::get<0>(red()), std::get<1>(red()),
        std::get<2>(red())),
      &debug_marker_array, now);
  }
  return debug_marker_array;
}
}  // namespace autoware::behavior_velocity_planner
