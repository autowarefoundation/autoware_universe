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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include "structs.hpp"

#include <autoware/planning_validator/types.hpp>
#include <autoware/route_handler/route_handler.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_validator::utils
{
auto check_shift_behavior(
  const lanelet::ConstLanelets & lanelets, const bool is_unsafe_holding,
  const std::shared_ptr<PlanningValidatorContext> & context,
  const rear_collision_checker_node::Params & parameters, DebugData & debug)
  -> std::pair<Behavior, double>;

auto check_turn_behavior(
  const lanelet::ConstLanelets & lanelets, const bool is_unsafe_holding,
  const std::shared_ptr<PlanningValidatorContext> & context,
  const rear_collision_checker_node::Params & parameters, DebugData & debug)
  -> std::pair<Behavior, double>;

void cut_by_lanelets(const lanelet::ConstLanelets & lanelets, DetectionAreas & detection_areas);

void fill_rss_distance(
  PointCloudObjects & objects, const std::shared_ptr<PlanningValidatorContext> & context,
  const double distance_to_conflict_point, const double reaction_time,
  const double max_deceleration, const double max_velocity,
  const rear_collision_checker_node::Params & parameters);

void fill_time_to_collision(
  PointCloudObjects & objects, const std::shared_ptr<PlanningValidatorContext> & context,
  const double distance_to_conflict_point, const double reaction_time,
  const double max_deceleration, const double max_velocity,
  const rear_collision_checker_node::Params & parameters);

auto generate_half_lanelet(
  const lanelet::ConstLanelet lanelet, const bool is_right,
  const double ignore_width_from_centerline, const double expand_width_from_bound)
  -> lanelet::ConstLanelet;

auto get_current_lanes(
  const std::shared_ptr<PlanningValidatorContext> & context, const double forward_distance,
  const double backward_distance) -> lanelet::ConstLanelets;

auto get_obstacle_points(const lanelet::BasicPolygons3d & polygons, const PointCloud & points)
  -> PointCloud::Ptr;

auto get_previous_polygons_with_lane_recursively(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
  const double s1, const double s2,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const double left_offset, const double right_offset) -> DetectionAreas;

auto generate_detection_polygon(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & ego_pose,
  const double forward_distance, const double backward_distance) -> lanelet::BasicPolygon3d;

auto get_range_for_rss(
  const std::shared_ptr<PlanningValidatorContext> & context,
  const double distance_to_conflict_point, const double reaction_time,
  const double max_deceleration, const double max_velocity,
  const rear_collision_checker_node::Params & parameters) -> std::pair<double, double>;

auto get_range_for_ttc(
  const std::shared_ptr<PlanningValidatorContext> & context,
  const double distance_to_conflict_point, const double reaction_time,
  const double max_deceleration, const double max_velocity,
  const rear_collision_checker_node::Params & parameters) -> std::pair<double, double>;

auto create_polygon_marker_array(
  const std::vector<autoware_utils::Polygon3d> & polygons, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color) -> MarkerArray;

auto create_polygon_marker_array(
  const lanelet::BasicPolygons3d & polygons, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color) -> MarkerArray;

auto create_pointcloud_object_marker_array(
  const PointCloudObjects & objects, const std::string & ns,
  const rear_collision_checker_node::Params & parameters) -> MarkerArray;

auto create_line_marker_array(
  const autoware_utils::LineString3d & line, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color) -> MarkerArray;
}  // namespace autoware::planning_validator::utils

#endif  // UTILS_HPP_
