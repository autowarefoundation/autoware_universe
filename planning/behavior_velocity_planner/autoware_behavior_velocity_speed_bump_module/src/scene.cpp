// Copyright 2025 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#include "scene.hpp"

#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware::motion_utils::calcSignedArcLength;
using autoware_utils::append_marker_array;
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;
using autoware_utils_geometry::calc_offset_pose;
using autoware_utils_geometry::create_point;
using geometry_msgs::msg::Point32;
using visualization_msgs::msg::Marker;

namespace
{
visualization_msgs::msg::MarkerArray createSpeedBumpMarkers(
  const SpeedBumpModule::DebugData & debug_data, const rclcpp::Time & now,
  const lanelet::Id module_id)
{
  visualization_msgs::msg::MarkerArray msg;
  const int32_t uid = planning_utils::bitShift(module_id);

  // Speed bump polygon
  if (!debug_data.speed_bump_polygon.empty()) {
    auto marker = create_default_marker(
      "map", now, "speed_bump polygon", uid, Marker::LINE_STRIP, create_marker_scale(0.1, 0.0, 0.0),
      create_marker_color(0.0, 0.0, 1.0, 0.999));
    for (const auto & p : debug_data.speed_bump_polygon) {
      marker.points.push_back(create_point(p.x, p.y, p.z));
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  // Slow start point
  if (debug_data.slow_start_pose) {
    auto marker = create_default_marker(
      "map", now, "slow start point", uid, Marker::POINTS, create_marker_scale(0.25, 0.25, 0.0),
      create_marker_color(1.0, 1.0, 0.0, 0.999));
    marker.points.push_back(debug_data.slow_start_pose->position);
    msg.markers.push_back(marker);
  }

  // Slow end point
  if (debug_data.slow_end_point) {
    auto marker = create_default_marker(
      "map", now, "slow end point", uid, Marker::POINTS, create_marker_scale(0.25, 0.25, 0.0),
      create_marker_color(0.2, 0.8, 1.0, 0.999));
    marker.points.push_back(*debug_data.slow_end_point);
    msg.markers.push_back(marker);
  }

  // Path - polygon intersection points
  {
    auto marker = create_default_marker(
      "map", now, "path_polygon intersection points", uid, Marker::POINTS,
      create_marker_scale(0.25, 0.25, 0.0), create_marker_color(1.0, 0.0, 0.0, 0.999));
    const auto & p_first = debug_data.first_intersection_point;
    if (p_first) {
      marker.points.push_back(*p_first);
    }
    const auto & p_second = debug_data.second_intersection_point;
    if (p_second) {
      marker.points.push_back(*p_second);
    }
    if (!marker.points.empty()) msg.markers.push_back(marker);
  }

  return msg;
}
}  // namespace

SpeedBumpModule::SpeedBumpModule(
  const lanelet::Id module_id, const lanelet::autoware::SpeedBump & speed_bump_reg_elem,
  const PlannerParam & planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  module_id_(module_id),
  speed_bump_reg_elem_(std::move(speed_bump_reg_elem)),
  planner_param_(planner_param),
  debug_data_()
{
  // Read speed bump height [m] from map
  const auto speed_bump_height =
    static_cast<float>(speed_bump_reg_elem_.speedBump().attributeOr("height", 0.5));

  // If slow_down_speed is specified on speed_bump annotation use it instead of calculating it
  if (speed_bump_reg_elem_.speedBump().hasAttribute("slow_down_speed")) {
    speed_bump_slow_down_speed_ = static_cast<float>(
      speed_bump_reg_elem_.speedBump().attribute("slow_down_speed").asDouble().get() / 3.6);
  } else {
    // point.x : height [m] -- point.y : speed [m/s]
    Point32 p1;
    Point32 p2;

    p1.x = planner_param_.speed_calculation_min_height;
    p2.x = planner_param_.speed_calculation_max_height;

    p1.y = planner_param_.speed_calculation_max_speed;
    p2.y = planner_param_.speed_calculation_min_speed;

    // Calculate the speed [m/s] for speed bump
    speed_bump_slow_down_speed_ = calcSlowDownSpeed(p1, p2, speed_bump_height);
  }

  if (planner_param_.print_debug_info) {
    std::cout << "------------------------------" << std::endl;
    std::cout << "Speed Bump ID: " << module_id_ << std::endl;
    std::cout << "Speed Bump Height [cm]: " << speed_bump_height * 100 << std::endl;
    std::cout << "Slow Down Speed [kph]: " << speed_bump_slow_down_speed_ * 3.6 << std::endl;
    std::cout << "------------------------------" << std::endl;
  }
}

bool SpeedBumpModule::modifyPathVelocity(
  experimental::Trajectory & path,
  [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & left_bound,
  [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & right_bound,
  const PlannerData & planner_data)
{
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data.vehicle_info_.max_longitudinal_offset_m;

  const auto & ego_pos = planner_data.current_odometry->pose.position;
  const auto & speed_bump = speed_bump_reg_elem_.speedBump();
  const auto & speed_bump_polygon = lanelet::utils::to2D(speed_bump).basicPolygon();

  const auto & path_polygon_intersection =
    speed_bump::getPathIntersectionWithSpeedBumpPolygon(path, speed_bump_polygon, 2);

  if (path_polygon_intersection.first_intersection_s) {
    debug_data_.first_intersection_point =
      path.compute(*path_polygon_intersection.first_intersection_s).point.pose.position;
  }
  if (path_polygon_intersection.second_intersection_s) {
    debug_data_.second_intersection_point =
      path.compute(*path_polygon_intersection.second_intersection_s).point.pose.position;
  }

  for (const auto & p : speed_bump_reg_elem_.speedBump().basicPolygon()) {
    debug_data_.speed_bump_polygon.push_back(create_point(p.x(), p.y(), ego_pos.z));
  }

  if (!applySlowDownSpeed(path, path_polygon_intersection, planner_data)) {
    return false;
  }

  return true;
}

bool SpeedBumpModule::applySlowDownSpeed(
  experimental::Trajectory & path,
  const speed_bump::PolygonIntersection & path_polygon_intersection,
  const PlannerData & planner_data)
{
  if (isNoRelation(path_polygon_intersection)) {
    return false;
  }

  // decide slow start position
  auto slow_start_s = 0.0;
  // if first intersection point exists
  if (path_polygon_intersection.first_intersection_s) {
    // calculate slow start position wrt the first intersection point between
    // path and the speed bump polygon
    const auto & slow_start_margin_to_base_link =
      -1 *
      (planner_data.vehicle_info_.max_longitudinal_offset_m + planner_param_.slow_start_margin);
    slow_start_s = std::max(
      0.0, *path_polygon_intersection.first_intersection_s + slow_start_margin_to_base_link);
    debug_data_.slow_start_pose = path.compute(slow_start_s).point.pose;
  }

  // decide slow end position
  auto slow_end_s = path.length();
  // if second intersection point exists
  if (path_polygon_intersection.second_intersection_s) {
    // calculate slow end position wrt the second intersection point between path
    // and the speed bump polygon
    const auto & slow_end_margin_to_base_link =
      planner_data.vehicle_info_.rear_overhang_m + planner_param_.slow_end_margin;
    slow_end_s = *path_polygon_intersection.second_intersection_s + slow_end_margin_to_base_link;
    debug_data_.slow_end_point = path.compute(slow_end_s).point.pose.position;
  }

  // clamp speed at path points that intersects with speed bump area
  path.longitudinal_velocity_mps()
    .range(slow_start_s, slow_end_s)
    .clamp(speed_bump_slow_down_speed_);

  return true;
}

autoware::motion_utils::VirtualWalls SpeedBumpModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  wall.text = "speed_bump";
  wall.ns = std::to_string(module_id_) + "_";
  wall.style = autoware::motion_utils::VirtualWallType::slowdown;
  if (debug_data_.slow_start_pose) {
    wall.pose =
      calc_offset_pose(*debug_data_.slow_start_pose, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

visualization_msgs::msg::MarkerArray SpeedBumpModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  append_marker_array(
    createSpeedBumpMarkers(debug_data_, this->clock_->now(), module_id_), &debug_marker_array);

  return debug_marker_array;
}

}  // namespace autoware::behavior_velocity_planner
