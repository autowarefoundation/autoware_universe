// Copyright 2025 Tier IV, Inc. All rights reserved.
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

#include "scene_occlusion_spot.hpp"

#include "../risk_predictive_braking.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/debug.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/trajectory_utils.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// turn on only when debugging.
#define DEBUG_PRINT(enable, n, x)                                  \
  if (enable) {                                                    \
    const std::string time_msg = (n) + std::to_string(x);          \
    RCLCPP_INFO_STREAM_THROTTLE(logger_, *clock_, 3000, time_msg); \
  }

namespace autoware::behavior_velocity_planner::experimental
{
namespace
{
namespace utils = occlusion_spot_utils;
using BasicPolygons = std::vector<lanelet::BasicPolygon2d>;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_utils::append_marker_array;
using autoware_utils::calc_offset_pose;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_orientation;
using autoware_utils::create_marker_position;
using autoware_utils::create_marker_scale;
using builtin_interfaces::msg::Time;
using utils::PossibleCollisionInfo;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

std::vector<Marker> makeDebugInfoMarker(
  const PossibleCollisionInfo & possible_collision, const int id, const bool show_text)
{
  std::vector<Marker> debug_markers;
  Marker debug_marker;
  debug_marker.header.frame_id = "map";
  debug_marker.id = id;
  debug_marker.action = Marker::ADD;
  debug_marker.pose.position = create_marker_position(0.0, 0.0, 0.0);
  debug_marker.pose.orientation = create_marker_orientation(0, 0, 0, 1.0);
  debug_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  const auto & pc = possible_collision;
  // for collision point with margin
  {
    debug_marker.ns = "collision_point";
    debug_marker.type = Marker::CYLINDER;
    debug_marker.pose = pc.collision_with_margin.pose;
    debug_marker.scale = create_marker_scale(0.5, 0.5, 0.5);
    debug_marker.color = create_marker_color(1.0, 0.0, 0.0, 0.5);
    debug_markers.push_back(debug_marker);
  }
  // cylinder at collision_point point
  {
    debug_marker.ns = "collision_point_with_margin";
    debug_marker.type = Marker::CYLINDER;
    debug_marker.pose = pc.collision_pose;
    debug_marker.scale = create_marker_scale(0.5, 0.5, 0.5);
    debug_marker.color = create_marker_color(1.0, 0.5, 0.0, 0.5);
    debug_markers.push_back(debug_marker);
  }

  // cylinder at obstacle point
  {
    debug_marker.ns = "obstacle";
    debug_marker.type = Marker::CYLINDER;
    debug_marker.pose.position = pc.obstacle_info.position;
    debug_marker.color = create_marker_color(0.5, 0.5, 0.5, 0.5);
    debug_marker.scale = create_marker_scale(0.8, 0.8, 1.5);
    debug_markers.push_back(debug_marker);
  }

  // arrow marker
  {
    debug_marker.ns = "from_obj_to_collision";
    debug_marker.type = Marker::ARROW;
    debug_marker.scale = create_marker_scale(0.05, 0.2, 0.5);
    debug_marker.color = create_marker_color(0.1, 0.1, 0.1, 0.5);
    debug_marker.points = {pc.obstacle_info.position, pc.intersection_pose.position};
    debug_markers.push_back(debug_marker);
  }

  if (show_text) {
    // info text at obstacle point
    debug_marker.ns = "info";
    debug_marker.type = Marker::TEXT_VIEW_FACING;
    debug_marker.pose = pc.collision_with_margin.pose;
    debug_marker.scale.z = 1.0;
    debug_marker.color = create_marker_color(1.0, 1.0, 0.0, 1.0);
    std::ostringstream string_stream;
    auto r = [](const double v) { return std::round(v * 100.0) / 100.0; };
    const double len = r(pc.arc_lane_dist_at_collision.length);
    const double dist = r(pc.arc_lane_dist_at_collision.distance);
    const double vel = r(pc.obstacle_info.safe_motion.safe_velocity);
    const double margin = r(pc.obstacle_info.safe_motion.stop_dist);
    string_stream << "(s,d,v,m)=(" << len << " , " << dist << " , " << vel << " , " << margin
                  << " )";
    debug_marker.text = string_stream.str();
    debug_markers.push_back(debug_marker);
  }
  return debug_markers;
}

template <class T>
MarkerArray makeDebugInfoMarkers(T & debug_data)
{
  // add slow down markers for occlusion spot
  MarkerArray debug_markers;
  const auto & possible_collisions = debug_data.possible_collisions;
  size_t id = 0;
  // draw obstacle collision
  for (const auto & pc : possible_collisions) {
    // debug marker
    std::vector<Marker> collision_markers = makeDebugInfoMarker(pc, id, true);
    debug_markers.markers.insert(
      debug_markers.markers.end(), collision_markers.begin(), collision_markers.end());
    id++;
  }
  return debug_markers;
}

MarkerArray makePolygonMarker(
  const BasicPolygons & polygons, const std::string & ns, const int id, const double z)
{
  MarkerArray debug_markers;
  Marker debug_marker;
  debug_marker.header.frame_id = "map";
  debug_marker.header.stamp = rclcpp::Time(0);
  debug_marker.id = planning_utils::bitShift(id);
  debug_marker.type = Marker::LINE_STRIP;
  debug_marker.action = Marker::ADD;
  debug_marker.pose.position = create_marker_position(0.0, 0.0, 0);
  debug_marker.pose.orientation = create_marker_orientation(0, 0, 0, 1.0);
  debug_marker.scale = create_marker_scale(0.1, 0.1, 0.1);
  debug_marker.color = create_marker_color(1.0, 1.0, 1.0, 0.5);
  debug_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  debug_marker.ns = ns;
  for (const auto & poly : polygons) {
    for (const auto & p : poly) {
      geometry_msgs::msg::Point point = create_marker_position(p.x(), p.y(), z + 0.5);
      debug_marker.points.push_back(point);
    }
    debug_markers.markers.push_back(debug_marker);
    debug_marker.id++;
    debug_marker.points.clear();
  }
  return debug_markers;
}

MarkerArray makeSlicePolygonMarker(
  const Polygons2d & slices, const std::string & ns, const int id, const double z)
{
  MarkerArray debug_markers;
  Marker debug_marker;
  debug_marker.header.frame_id = "map";
  debug_marker.header.stamp = rclcpp::Time(0);
  debug_marker.id = planning_utils::bitShift(id);
  debug_marker.type = Marker::LINE_STRIP;
  debug_marker.action = Marker::ADD;
  debug_marker.pose.position = create_marker_position(0.0, 0.0, 0);
  debug_marker.pose.orientation = create_marker_orientation(0, 0, 0, 1.0);
  debug_marker.scale = create_marker_scale(0.1, 0.1, 0.1);
  debug_marker.color = create_marker_color(1.0, 0.0, 1.0, 0.3);
  debug_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
  debug_marker.ns = ns;
  for (const auto & slice : slices) {
    for (const auto & p : slice.outer()) {
      geometry_msgs::msg::Point point = create_marker_position(p.x(), p.y(), z);
      debug_marker.points.push_back(point);
    }
    debug_markers.markers.push_back(debug_marker);
    debug_marker.id++;
    debug_marker.points.clear();
  }
  return debug_markers;
}

std::vector<PredictedObject> extractStuckVehicle(
  const std::vector<PredictedObject> & vehicles, const double stop_velocity)
{
  std::vector<PredictedObject> stuck_vehicles;
  for (const auto & obj : vehicles) {
    if (utils::isStuckVehicle(obj, stop_velocity)) {
      stuck_vehicles.emplace_back(obj);
    }
  }
  return stuck_vehicles;
}
}  // namespace

OcclusionSpotModule::OcclusionSpotModule(
  const lanelet::Id module_id, const PlannerData & planner_data, const PlannerParam & planner_param,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  param_(planner_param)
{
  if (param_.detection_method == utils::DETECTION_METHOD::OCCUPANCY_GRID) {
    debug_data_.detection_type = "occupancy";
    //! occupancy grid limitation( 100 * 100 )
    const double max_length = 35.0;  // current available length
    param_.detection_area_length = std::min(max_length, param_.detection_area_length);
  } else if (param_.detection_method == utils::DETECTION_METHOD::PREDICTED_OBJECT) {
    debug_data_.detection_type = "object";
  }
  if (param_.use_partition_lanelet) {
    const lanelet::LaneletMapConstPtr & ll = planner_data.route_handler_->getLaneletMapPtr();
    planning_utils::getAllPartitionLanelets(ll, partition_lanelets_);
  }
}

bool OcclusionSpotModule::modifyPathVelocity(
  Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data)
{
  auto path_msg = planning_utils::fromTrajectory(path, left_bound, right_bound);

  if (param_.is_show_processing_time) stop_watch_.tic("total_processing_time");
  debug_data_.resetData();

  // set planner data
  {
    param_.v.max_stop_jerk = planner_data.max_stop_jerk_threshold;
    param_.v.max_stop_accel = planner_data.max_stop_acceleration_threshold;
    param_.v.v_ego = planner_data.current_velocity->twist.linear.x;
    param_.v.a_ego = planner_data.current_acceleration->accel.accel.linear.x;
    param_.v.delay_time = planner_data.system_delay;
    param_.detection_area_max_length =
      planning_utils::calcJudgeLineDistWithJerkLimit(
        param_.v.v_ego, param_.v.a_ego, param_.v.non_effective_accel, param_.v.non_effective_jerk,
        planner_data.delay_response_time) +
      param_.detection_area_offset;  // To fill difference between planned and measured acc
  }
  const geometry_msgs::msg::Pose ego_pose = planner_data.current_odometry->pose;
  PathWithLaneId clipped_path;
  utils::clipPathByLength(path_msg, clipped_path, param_.detection_area_length);
  PathWithLaneId path_interpolated;
  //! never change this interpolation interval(will affect module accuracy)
  splineInterpolate(clipped_path, 1.0, path_interpolated, logger_);
  const geometry_msgs::msg::Point start_point = path_interpolated.points.at(0).point.pose.position;
  const auto ego_segment_idx = autoware::motion_utils::findNearestSegmentIndex(
    path_interpolated.points, ego_pose, param_.dist_thr, param_.angle_thr);
  if (!ego_segment_idx) return true;
  const size_t start_point_segment_idx =
    autoware::motion_utils::findNearestSegmentIndex(path_interpolated.points, start_point);
  const auto offset = autoware::motion_utils::calcSignedArcLength(
    path_interpolated.points, ego_pose.position, *ego_segment_idx, start_point,
    start_point_segment_idx);
  const double offset_from_start_to_ego = -offset;
  const bool show_time = param_.is_show_processing_time;
  if (show_time) stop_watch_.tic("processing_time");
  PathWithLaneId predicted_path;
  if (param_.pass_judge == utils::PASS_JUDGE::CURRENT_VELOCITY) {
    predicted_path = utils::applyVelocityToPath(path_interpolated, param_.v.v_ego);
  } else if (param_.pass_judge == utils::PASS_JUDGE::SMOOTH_VELOCITY) {
    if (!smoothPath(path_interpolated, predicted_path, planner_data)) {
      predicted_path = utils::applyVelocityToPath(path_interpolated, param_.v.v_ego);
      // use current ego velocity in path if optimization failure
    }
  }
  DEBUG_PRINT(show_time, "apply velocity [ms]: ", stop_watch_.toc("processing_time", true));
  const size_t ego_seg_idx = findEgoSegmentIndex(predicted_path.points, planner_data);
  if (!utils::buildDetectionAreaPolygon(
        debug_data_.detection_area_polygons, predicted_path, ego_pose, ego_seg_idx, param_)) {
    return true;  // path point is not enough
  }
  DEBUG_PRINT(show_time, "generate poly[ms]: ", stop_watch_.toc("processing_time", true));
  std::vector<utils::PossibleCollisionInfo> possible_collisions;
  // extract only close lanelet
  if (param_.use_partition_lanelet) {
    planning_utils::extractClosePartition(
      ego_pose.position, partition_lanelets_, debug_data_.close_partition);
  }
  DEBUG_PRINT(show_time, "extract[ms]: ", stop_watch_.toc("processing_time", true));
  const auto objects_ptr = planner_data.predicted_objects;
  const auto vehicles =
    utils::extractVehicles(objects_ptr, ego_pose.position, param_.detection_area_length);
  const std::vector<PredictedObject> filtered_vehicles =
    utils::filterVehiclesByDetectionArea(vehicles, debug_data_.detection_area_polygons);
  DEBUG_PRINT(show_time, "filter obj[ms]: ", stop_watch_.toc("processing_time", true));
  if (param_.detection_method == utils::DETECTION_METHOD::OCCUPANCY_GRID) {
    const auto & occ_grid_ptr = planner_data.occupancy_grid;
    if (!occ_grid_ptr) return true;  // no data
    grid_map::GridMap grid_map;
    Polygons2d stuck_vehicle_foot_prints;
    Polygons2d moving_vehicle_foot_prints;
    utils::categorizeVehicles(
      filtered_vehicles, stuck_vehicle_foot_prints, moving_vehicle_foot_prints,
      param_.stuck_vehicle_vel);
    // occ -> image
    // find out occlusion from erode occlusion candidate num iter is strength of filter
    const int num_iter = static_cast<int>(
      (param_.detection_area.min_occlusion_spot_size / occ_grid_ptr->info.resolution) - 1);
    grid_utils::denoiseOccupancyGridCV(
      occ_grid_ptr, stuck_vehicle_foot_prints, moving_vehicle_foot_prints, grid_map, param_.grid,
      param_.is_show_cv_window, num_iter, param_.use_object_info,
      param_.use_moving_object_ray_cast);
    DEBUG_PRINT(show_time, "grid [ms]: ", stop_watch_.toc("processing_time", true));
    // Note: Don't consider offset from path start to ego here
    if (!utils::generatePossibleCollisionsFromGridMap(
          possible_collisions, grid_map, path_interpolated, offset_from_start_to_ego, param_,
          debug_data_)) {
      // no occlusion spot
      return true;
    }
  } else if (param_.detection_method == utils::DETECTION_METHOD::PREDICTED_OBJECT) {
    const auto stuck_vehicles = extractStuckVehicle(filtered_vehicles, param_.stuck_vehicle_vel);
    // Note: Don't consider offset from path start to ego here
    if (!utils::generatePossibleCollisionsFromObjects(
          possible_collisions, path_interpolated, param_, offset_from_start_to_ego,
          stuck_vehicles)) {
      // no occlusion spot
      return true;
    }
  }
  DEBUG_PRINT(show_time, "occlusion [ms]: ", stop_watch_.toc("processing_time", true));
  DEBUG_PRINT(show_time, "num collision:", possible_collisions.size());
  utils::calcSlowDownPointsForPossibleCollision(0, path_interpolated, 0.0, possible_collisions);
  // Note: Consider offset from path start to ego here
  utils::handleCollisionOffset(possible_collisions, offset_from_start_to_ego);
  // apply safe velocity using ebs and pbs deceleration
  utils::applySafeVelocityConsideringPossibleCollision(
    &path_msg, possible_collisions, debug_data_.debug_poses, param_);
  debug_data_.baselink_to_front = param_.baselink_to_front;
  // these debug topics needs computation resource
  debug_data_.z = path_msg.points.front().point.pose.position.z;
  debug_data_.possible_collisions = possible_collisions;
  DEBUG_PRINT(show_time, "total [ms]: ", stop_watch_.toc("total_processing_time", true));
  planning_utils::toTrajectory(path_msg, path);
  return true;
}

MarkerArray OcclusionSpotModule::createDebugMarkerArray()
{
  const auto now = this->clock_->now();
  MarkerArray debug_marker_array;
  if (!debug_data_.possible_collisions.empty()) {
    append_marker_array(makeDebugInfoMarkers(debug_data_), &debug_marker_array, now);
  }
  if (!debug_data_.detection_area_polygons.empty()) {
    append_marker_array(
      makeSlicePolygonMarker(
        debug_data_.detection_area_polygons, "detection_area", module_id_, debug_data_.z),
      &debug_marker_array, now);
  }
  if (!debug_data_.close_partition.empty() && param_.is_show_occlusion) {
    append_marker_array(
      makePolygonMarker(debug_data_.close_partition, "close_partition", module_id_, debug_data_.z),
      &debug_marker_array, now);
  }
  if (!debug_data_.occlusion_points.empty()) {
    append_marker_array(
      debug::createPointsMarkerArray(
        debug_data_.occlusion_points, "occlusion", module_id_, now, 0.5, 0.5, 0.5, 1.0, 0.0, 0.0),
      &debug_marker_array, now);
  }
  return debug_marker_array;
}

autoware::motion_utils::VirtualWalls OcclusionSpotModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  wall.text = "occlusion_spot";
  wall.style = autoware::motion_utils::VirtualWallType::slowdown;
  for (size_t id = 0; id < debug_data_.debug_poses.size(); id++) {
    wall.pose =
      calc_offset_pose(debug_data_.debug_poses.at(id), debug_data_.baselink_to_front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

}  // namespace autoware::behavior_velocity_planner::experimental
