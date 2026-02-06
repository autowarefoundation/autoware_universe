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

#include "scene_crosswalk.hpp"

#include "../parked_vehicles_stop.hpp"
#include "occluded_crosswalk.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware/trajectory/utils/find_nearest.hpp>
#include <autoware/trajectory/utils/velocity.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>

#include <lanelet2_routing/RoutingGraphContainer.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
namespace bg = boost::geometry;
using autoware::motion_utils::calcArcLength;
using autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints;
using autoware::motion_utils::calcLateralOffset;
using autoware::motion_utils::calcLongitudinalOffsetPoint;
using autoware::motion_utils::calcLongitudinalOffsetPose;
using autoware::motion_utils::calcSignedArcLength;
using autoware::motion_utils::calcSignedArcLengthPartialSum;
using autoware::motion_utils::findNearestSegmentIndex;
using autoware::motion_utils::resamplePath;
using autoware_utils::create_point;
using autoware_utils::get_pose;
using autoware_utils::Point2d;
using autoware_utils::Polygon2d;
using autoware_utils::to_hex_string;

namespace
{
using autoware_utils::append_marker_array;
using autoware_utils::calc_offset_pose;
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;
using visualization_msgs::msg::Marker;

geometry_msgs::msg::Point32 createPoint32(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

std::vector<geometry_msgs::msg::Point> toGeometryPointVector(
  const Polygon2d & polygon, const double z)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto & p : polygon.outer()) {
    points.push_back(create_point(p.x(), p.y(), z));
  }
  return points;
}

void offsetPolygon2d(
  const geometry_msgs::msg::Pose & origin_point, const geometry_msgs::msg::Polygon & polygon,
  Polygon2d & offset_polygon)
{
  for (const auto & polygon_point : polygon.points) {
    const auto offset_pos =
      autoware_utils::calc_offset_pose(origin_point, polygon_point.x, polygon_point.y, 0.0)
        .position;
    offset_polygon.outer().push_back(Point2d(offset_pos.x, offset_pos.y));
  }
}

template <class T>
Polygon2d createMultiStepPolygon(
  const T & obj_path_points, const geometry_msgs::msg::Polygon & polygon, const size_t start_idx,
  const size_t end_idx)
{
  Polygon2d multi_step_polygon{};
  for (size_t i = start_idx; i <= end_idx; ++i) {
    offsetPolygon2d(autoware_utils::get_pose(obj_path_points.at(i)), polygon, multi_step_polygon);
  }

  Polygon2d hull_multi_step_polygon{};
  bg::convex_hull(multi_step_polygon, hull_multi_step_polygon);
  bg::correct(hull_multi_step_polygon);

  return hull_multi_step_polygon;
}

void sortCrosswalksByDistance(
  const Trajectory & ego_path, const geometry_msgs::msg::Point & ego_pos,
  lanelet::ConstLanelets & crosswalks)
{
  const auto compare = [&](const lanelet::ConstLanelet & l1, const lanelet::ConstLanelet & l2) {
    const auto l1_end_points_on_crosswalk =
      getPathEndPointsOnCrosswalk(ego_path, l1.polygon2d().basicPolygon(), ego_pos);
    const auto l2_end_points_on_crosswalk =
      getPathEndPointsOnCrosswalk(ego_path, l2.polygon2d().basicPolygon(), ego_pos);

    if (!l1_end_points_on_crosswalk || !l2_end_points_on_crosswalk) {
      return true;
    }

    const double dist_l1 = autoware::experimental::trajectory::find_nearest_index(
      ego_path, l1_end_points_on_crosswalk->first);
    const double dist_l2 = autoware::experimental::trajectory::find_nearest_index(
      ego_path, l2_end_points_on_crosswalk->first);

    return dist_l1 < dist_l2;
  };

  std::sort(crosswalks.begin(), crosswalks.end(), compare);
}

std::vector<Polygon2d> calcOverlappingPoints(const Polygon2d & polygon1, const Polygon2d & polygon2)
{
  // NOTE: If one polygon is fully inside the other polygon, the result is empty.
  std::vector<Polygon2d> intersection_polygons{};
  bg::intersection(polygon1, polygon2, intersection_polygons);
  return intersection_polygons;
}

autoware_internal_debug_msgs::msg::StringStamped createStringStampedMessage(
  const rclcpp::Time & now, const lanelet::Id module_id_,
  const std::vector<std::tuple<std::string, CollisionPoint, CollisionState>> & collision_points)
{
  autoware_internal_debug_msgs::msg::StringStamped msg;
  msg.stamp = now;
  for (const auto & collision_point : collision_points) {
    std::stringstream ss;
    ss << module_id_ << "," << std::get<0>(collision_point).substr(0, 4) << ","
       << std::get<1>(collision_point).time_to_collision << ","
       << std::get<1>(collision_point).time_to_vehicle << ","
       << static_cast<int>(std::get<2>(collision_point)) << ",";
    msg.data += ss.str();
  }
  return msg;
}

visualization_msgs::msg::MarkerArray createCrosswalkMarkers(
  const DebugData & debug_data, const rclcpp::Time & now, const lanelet::Id module_id)
{
  visualization_msgs::msg::MarkerArray msg;
  int32_t uid = planning_utils::bitShift(module_id);

  {
    for (size_t i = 0; i < debug_data.collision_points.size(); ++i) {
      const auto & collision_point = debug_data.collision_points.at(i);
      const auto & point = std::get<1>(collision_point);
      const auto & state = std::get<2>(collision_point);

      auto marker = create_default_marker(
        "map", now, "collision point state", uid + i++, Marker::TEXT_VIEW_FACING,
        create_marker_scale(0.0, 0.0, 1.0), create_marker_color(1.0, 1.0, 1.0, 0.999));
      std::ostringstream string_stream;
      string_stream << std::fixed << std::setprecision(2);
      string_stream << "(module, ttc, ttv, state)=(" << module_id << " , "
                    << point.time_to_collision << " , " << point.time_to_vehicle;
      switch (state) {
        case CollisionState::YIELD:
          string_stream << " , YIELD)";
          break;
        case CollisionState::EGO_PASS_FIRST:
          string_stream << " , EGO_PASS_FIRST)";
          break;
        case CollisionState::EGO_PASS_LATER:
          string_stream << " , EGO_PASS_LATER)";
          break;
        case CollisionState::IGNORE:
          string_stream << " , IGNORE)";
          break;
        default:
          string_stream << " , NONE)";
          break;
      }
      marker.text = string_stream.str();
      marker.pose.position = point.collision_point;
      marker.pose.position.z += 2.0 + i * 0.5;  // NOTE: so that the texts will not overlap
      msg.markers.push_back(marker);
    }
  }

  if (debug_data.range_near_point) {
    auto marker = create_default_marker(
      "map", now, "attention range near", uid, Marker::POINTS, create_marker_scale(0.25, 0.25, 0.0),
      create_marker_color(0.0, 0.0, 1.0, 0.999));
    marker.points.push_back(*debug_data.range_near_point);
    msg.markers.push_back(marker);
  }

  if (debug_data.range_far_point) {
    auto marker = create_default_marker(
      "map", now, "attention range far", uid, Marker::POINTS, create_marker_scale(0.25, 0.25, 0.0),
      create_marker_color(1.0, 1.0, 1.0, 0.999));
    marker.points.push_back(*debug_data.range_far_point);
    msg.markers.push_back(marker);
  }

  for (size_t i = 0; i < debug_data.obj_polygons.size(); ++i) {
    const auto & points = debug_data.obj_polygons.at(i);
    auto marker = create_default_marker(
      "map", now, "object polygon", uid + i, Marker::LINE_STRIP, create_marker_scale(0.1, 0.0, 0.0),
      create_marker_color(1.0, 0.0, 1.0, 0.999));
    marker.points = points;
    // marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  for (size_t i = 0; i < debug_data.ego_polygons.size(); ++i) {
    const auto & points = debug_data.ego_polygons.at(i);
    auto marker = create_default_marker(
      "map", now, "vehicle polygon", uid + i, Marker::LINE_STRIP,
      create_marker_scale(0.1, 0.0, 0.0), create_marker_color(1.0, 1.0, 0.0, 0.999));
    marker.points = points;
    // marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  // Crosswalk polygon
  if (!debug_data.crosswalk_polygon.empty()) {
    auto marker = create_default_marker(
      "map", now, "crosswalk polygon", uid, Marker::LINE_STRIP, create_marker_scale(0.1, 0.0, 0.0),
      create_marker_color(1.0, 1.0, 1.0, 0.999));
    for (const auto & p : debug_data.crosswalk_polygon) {
      marker.points.push_back(create_point(p.x, p.y, p.z));
    }
    marker.points.push_back(marker.points.front());
    marker.color = debug_data.ignore_crosswalk ? create_marker_color(1.0, 1.0, 1.0, 0.999)
                                               : create_marker_color(1.0, 0.0, 0.0, 0.999);
    msg.markers.push_back(marker);
  }

  // Collision point
  if (!debug_data.collision_points.empty()) {
    auto marker = create_default_marker(
      "map", now, "collision point", uid, Marker::POINTS, create_marker_scale(0.25, 0.25, 0.0),
      create_marker_color(1.0, 0.0, 1.0, 0.999));
    for (const auto & collision_point : debug_data.collision_points) {
      marker.points.push_back(std::get<1>(collision_point).collision_point);
    }
    msg.markers.push_back(marker);
  }

  // Slow point
  if (!debug_data.slow_poses.empty()) {
    auto marker = create_default_marker(
      "map", now, "slow point", uid, Marker::POINTS, create_marker_scale(0.25, 0.25, 0.0),
      create_marker_color(1.0, 1.0, 0.0, 0.999));
    for (const auto & p : debug_data.slow_poses) {
      marker.points.push_back(create_point(p.position.x, p.position.y, p.position.z));
    }
    msg.markers.push_back(marker);
  }

  // Stop factor point
  if (!debug_data.stop_factor_points.empty()) {
    auto marker = create_default_marker(
      "map", now, "stop factor point", uid, Marker::POINTS, create_marker_scale(0.25, 0.25, 0.0),
      create_marker_color(0.0, 0.0, 1.0, 0.999));
    for (const auto & p : debug_data.stop_factor_points) {
      marker.points.push_back(p);
    }
    msg.markers.push_back(marker);
  }

  // Stop point
  if (!debug_data.stop_poses.empty()) {
    auto marker = create_default_marker(
      "map", now, "stop point", uid, Marker::POINTS, create_marker_scale(0.25, 0.25, 0.0),
      create_marker_color(1.0, 0.0, 0.0, 0.999));
    for (const auto & p : debug_data.stop_poses) {
      marker.points.push_back(create_point(p.position.x, p.position.y, p.position.z));
    }
    msg.markers.push_back(marker);
  }

  if (!debug_data.occlusion_detection_areas.empty()) {
    auto marker = create_default_marker(
      "map", now, "occlusion_detection_areas", uid, Marker::LINE_LIST,
      create_marker_scale(0.25, 0.25, 0.0), create_marker_color(1.0, 1.0, 1.0, 0.5));
    for (const auto & area : debug_data.occlusion_detection_areas) {
      for (auto i = 0UL; i + 1 < area.size(); ++i) {
        const auto & p1 = area[i];
        const auto & p2 = area[i + 1];
        marker.points.push_back(create_point(p1.x(), p1.y(), 0.0));
        marker.points.push_back(create_point(p2.x(), p2.y(), 0.0));
      }
    }
    msg.markers.push_back(marker);
    marker = create_default_marker(
      "map", now, "crosswalk_origin", uid, Marker::SPHERE, create_marker_scale(0.25, 0.25, 0.25),
      create_marker_color(1.0, 1.0, 1.0, 0.5));
    marker.pose.position = debug_data.crosswalk_origin;
    msg.markers.push_back(marker);
  }

  // parked vehicles stop
  {
    const auto color = debug_data.parked_vehicles_stop_already_stopped
                         ? create_marker_color(1.0, 1.0, 1.0, 0.5)
                         : create_marker_color(1.0, 0.0, 0.0, 0.5);
    auto marker = create_default_marker(
      "map", now, "parked_vehicles_stop_search_area", uid, Marker::LINE_STRIP,
      create_marker_scale(0.25, 0.25, 0.0), color);
    marker.lifetime.sec = 0.0;
    marker.lifetime.nanosec = 0.0;
    for (const auto & p : debug_data.parked_vehicles_stop_search_area) {
      marker.points.push_back(create_point(p.x(), p.y(), 0.0));
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    msg.markers.push_back(marker);
  }

  return msg;
}
}  // namespace

CrosswalkModule::CrosswalkModule(
  rclcpp::Node & node, const lanelet::Id lane_id, const lanelet::Id module_id,
  const std::optional<lanelet::Id> & reg_elem_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const PlannerParam & planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterfaceWithRTC(module_id, logger, clock, time_keeper, planning_factor_interface),
  module_id_(module_id),
  planner_param_(planner_param),
  use_regulatory_element_(reg_elem_id)
{
  passed_safety_slow_point_ = false;

  if (use_regulatory_element_) {
    const auto reg_elem_ptr = std::dynamic_pointer_cast<const lanelet::autoware::Crosswalk>(
      lanelet_map_ptr->regulatoryElementLayer.get(*reg_elem_id));
    stop_lines_ = reg_elem_ptr->stopLines();
    crosswalk_ = reg_elem_ptr->crosswalkLanelet();
  } else {
    const auto stop_line = getStopLineFromMap(module_id_, lanelet_map_ptr, "crosswalk_id");
    if (stop_line) {
      stop_lines_.push_back(*stop_line);
    }
    crosswalk_ = lanelet_map_ptr->laneletLayer.get(module_id);
  }

  road_ = lanelet_map_ptr->laneletLayer.get(lane_id);

  collision_info_pub_ = node.create_publisher<autoware_internal_debug_msgs::msg::StringStamped>(
    "~/debug/collision_info", 1);
}

bool CrosswalkModule::modifyPathVelocity(
  Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data)
{
  auto path_msg = planning_utils::fromTrajectory(path, left_bound, right_bound);

  stop_watch_.tic("total_processing_time");
  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time,
    "=========== module_id: %ld ===========", module_id_);

  const auto & ego_pos = planner_data.current_odometry->pose.position;
  const auto objects_ptr = planner_data.predicted_objects;

  // Initialize debug data
  debug_data_ = DebugData(planner_data);
  for (const auto & p : crosswalk_.polygon2d().basicPolygon()) {
    debug_data_.crosswalk_polygon.push_back(create_point(p.x(), p.y(), ego_pos.z));
  }
  recordTime(1);

  // Calculate intersection between path and crosswalks
  const auto path_end_points_on_crosswalk =
    getPathEndPointsOnCrosswalk(path, crosswalk_.polygon2d().basicPolygon(), ego_pos);
  if (!path_end_points_on_crosswalk) {
    return {};
  }
  const auto & first_path_point_on_crosswalk = path_end_points_on_crosswalk->first;
  const auto & last_path_point_on_crosswalk = path_end_points_on_crosswalk->second;

  // Apply safety slow down speed if defined in Lanelet2 map
  applySlowDownByLanelet2Map(
    path, first_path_point_on_crosswalk, last_path_point_on_crosswalk, planner_data);

  // Apply safety slow down speed if the crosswalk is occluded
  applySlowDownByOcclusion(
    path, first_path_point_on_crosswalk, last_path_point_on_crosswalk, planner_data);
  recordTime(2);

  // Calculate stop point with margin
  const auto default_stop_pose =
    getDefaultStopPose(path, first_path_point_on_crosswalk, planner_data);

  // Decide to stop for crosswalk users
  const auto stop_factor_for_crosswalk_users = checkStopForCrosswalkUsers(
    path, first_path_point_on_crosswalk, last_path_point_on_crosswalk, default_stop_pose,
    planner_data);
  // Decide to stop for obstruction prevention
  const auto stop_factor_for_obstruction_preventions = checkStopForObstructionPrevention(
    path, objects_ptr->objects, first_path_point_on_crosswalk, last_path_point_on_crosswalk,
    default_stop_pose, planner_data);
  // Decide to stop for parked vehicles (only if no other stop is planned)
  const auto stop_factor_for_parked_vehicles =
    checkStopForParkedVehicles(path, first_path_point_on_crosswalk, planner_data);

  // Get nearest stop factor and reason
  const auto [nearest_stop_factor, reason] = getNearestStopFactorAndReason(
    path, stop_factor_for_crosswalk_users, stop_factor_for_obstruction_preventions,
    stop_factor_for_parked_vehicles, planner_data);
  recordTime(3);

  // Set safe or unsafe
  setSafe(!nearest_stop_factor);

  // Set distance
  // NOTE: If no stop point is inserted, distance to the virtual stop line has to be calculated.
  setDistanceToStop(path, default_stop_pose, nearest_stop_factor, planner_data);

  // plan Go/Stop
  if (isActivated()) {
    planGo(path, nearest_stop_factor, planner_data);
  } else {
    planStop(path, nearest_stop_factor, default_stop_pose, reason, planner_data);
  }
  recordTime(4);

  const auto collision_info_msg =
    createStringStampedMessage(clock_->now(), module_id_, debug_data_.collision_points);
  collision_info_pub_->publish(collision_info_msg);
  return true;
}

// NOTE: The stop point will be the returned point with the margin.
std::optional<geometry_msgs::msg::Pose> CrosswalkModule::getDefaultStopPose(
  const Trajectory & ego_path, const geometry_msgs::msg::Point & first_path_point_on_crosswalk,
  const PlannerData & planner_data) const
{
  const auto & base_link2front = planner_data.vehicle_info_.max_longitudinal_offset_m;

  // If stop lines are found in the LL2 map.
  for (const auto & stop_line : stop_lines_) {
    const auto p_stop_lines = autoware::experimental::trajectory::crossed(
      ego_path, lanelet::utils::to2D(stop_line).basicLineString());
    if (!p_stop_lines.empty()) {
      const double stop_pose_s = p_stop_lines.front() - base_link2front;
      const auto stop_pose = ego_path.compute(stop_pose_s);
      return stop_pose.point.pose;
    }
  }

  // If stop lines are not found in the LL2 map.
  const auto first_point_s =
    autoware::experimental::trajectory::find_nearest_index(ego_path, first_path_point_on_crosswalk);
  const double stop_s =
    first_point_s - planner_param_.stop_distance_from_crosswalk - base_link2front;
  const auto stop_pose = ego_path.compute(stop_s);
  return stop_pose.point.pose;
}

std::optional<StopPoseWithObjectUuids> CrosswalkModule::checkStopForCrosswalkUsers(
  const Trajectory & ego_path, const geometry_msgs::msg::Point & first_path_point_on_crosswalk,
  const geometry_msgs::msg::Point & last_path_point_on_crosswalk,
  const std::optional<geometry_msgs::msg::Pose> & default_stop_pose,
  const PlannerData & planner_data)
{
  const auto & ego_pos = planner_data.current_odometry->pose.position;

  // Calculate attention range for crosswalk
  const auto crosswalk_attention_range = getAttentionRange(
    ego_path, first_path_point_on_crosswalk, last_path_point_on_crosswalk, planner_data);

  // Get attention area, which is ego's footprints on the crosswalk
  const auto attention_area = getAttentionArea(ego_path, crosswalk_attention_range, planner_data);

  // Update object state
  // This exceptional handling should be done in update(), but is compromised by history
  const double dist_default_stop =
    default_stop_pose.has_value()
      ? autoware::experimental::trajectory::find_nearest_index(
          ego_path, default_stop_pose->position) -
          autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos)
      : 0.0;
  updateObjectState(
    dist_default_stop, ego_path, crosswalk_attention_range, attention_area, planner_data);

  // Check pedestrian for stop
  // NOTE: first stop point and its minimum distance from ego to stop
  std::optional<double> dist_nearest_cp;
  std::vector<unique_identifier_msgs::msg::UUID> object_ids;
  for (const auto & object_info : object_info_manager_.objects) {
    const auto & object = object_info.second;
    const auto & collision_point_opt = object.collision_point;
    if (collision_point_opt) {
      const auto & collision_point = collision_point_opt.value();
      const auto & collision_state = object.collision_state;
      if (collision_state != CollisionState::YIELD) {
        continue;
      }

      object_ids.emplace_back(object_info.first);

      const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);
      const auto cp_s = autoware::experimental::trajectory::find_nearest_index(
        ego_path, collision_point.collision_point);
      const auto dist_ego2cp = cp_s - ego_s;
      if (!dist_nearest_cp || dist_ego2cp < dist_nearest_cp) {
        dist_nearest_cp = dist_ego2cp;
      }
    }
  }
  if (!dist_nearest_cp) {
    return {};
  }

  const auto decided_stop_pose_opt = calcStopPose(
    ego_path, dist_nearest_cp.value(), default_stop_pose, first_path_point_on_crosswalk,
    planner_data);
  if (!decided_stop_pose_opt.has_value()) {
    return {};
  }
  return StopPoseWithObjectUuids{decided_stop_pose_opt.value(), object_ids};
}

std::optional<geometry_msgs::msg::Pose> CrosswalkModule::calcStopPose(
  const Trajectory & ego_path, double dist_nearest_cp,
  const std::optional<geometry_msgs::msg::Pose> & default_stop_pose_opt,
  const geometry_msgs::msg::Point & first_path_point_on_crosswalk, const PlannerData & planner_data)
{
  struct StopCandidate
  {
    geometry_msgs::msg::Pose pose;
    double dist;
  };

  const auto & p = planner_param_;
  const auto base_link2front = planner_data.vehicle_info_.max_longitudinal_offset_m;
  const auto & ego_pos = planner_data.current_odometry->pose.position;
  const double ego_vel_non_negative = std::max(0.0, planner_data.current_velocity->twist.linear.x);
  const double ego_acc = planner_data.current_acceleration->accel.accel.linear.x;
  const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);
  const auto first_point_s =
    autoware::experimental::trajectory::find_nearest_index(ego_path, first_path_point_on_crosswalk);

  // TODO(takagi) replace without_acc_pref_stop_opt with default_stop_pose, and
  // replace existing default_stop_pose with static_stop_pose.
  const auto without_acc_pref_stop = [&]() -> std::optional<StopCandidate> {
    // From here, first_path_point_on_crosswalk is used as x-origin
    const double current_step_pref_x_pos = [&]() {
      const double dynamic_stop_distance_from_crosswalk_front =
        dist_nearest_cp - (first_point_s - ego_s) - base_link2front -
        planner_param_.stop_distance_from_object_preferred;
      if (!default_stop_pose_opt.has_value()) {
        return dynamic_stop_distance_from_crosswalk_front;
      }
      const auto default_stop_s = autoware::experimental::trajectory::find_nearest_index(
        ego_path, default_stop_pose_opt->position);
      const double static_stop_distance_from_crosswalk_front = default_stop_s - first_point_s;
      return std::min(
        dynamic_stop_distance_from_crosswalk_front, static_stop_distance_from_crosswalk_front);
    }();

    // If (pref_stop_x_position_.getValue().has_value() && ego_vel_non_negative < 1.0) do not update
    // the lpf variable.
    if (!pref_stop_x_position_.getValue().has_value()) {
      pref_stop_x_position_.reset(current_step_pref_x_pos);
    } else if (ego_vel_non_negative > 1.0) {
      if (current_step_pref_x_pos < *pref_stop_x_position_.getValue()) {
        pref_stop_x_position_.reset(current_step_pref_x_pos);
      } else {
        pref_stop_x_position_.filter(current_step_pref_x_pos);
      }
    }

    // From here, ego_pos is used as x-origin
    const double dist = (first_point_s - ego_s) + pref_stop_x_position_.getValue().value();
    const double target_s = ego_s + dist;
    const auto pose_opt = ego_path.compute(target_s).point.pose;
    if (target_s < 0.0 || target_s > ego_path.length()) {
      if (dist < 0.0) {
        return StopCandidate{ego_path.compute(0.0).point.pose, dist};
      }
      return std::nullopt;
    }
    return StopCandidate{pose_opt, dist};
  }();
  if (!without_acc_pref_stop.has_value()) {
    RCLCPP_INFO(
      logger_,
      "without_acc_pref_stop is beyond the path horizon. Crosswalk stop will be canceled.");
    return std::nullopt;
  }

  // From here, ego_pos is used as x-origin
  const auto weak_brake_stop = [&]() -> std::optional<StopCandidate> {
    // NOTE: weak_stop_dist_opt should have a value if the parameter of the module is correctly set.
    const auto weak_stop_dist_opt = autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
      ego_vel_non_negative, 0.0, ego_acc, p.min_acc_preferred, 10.0, p.min_jerk_preferred);
    if (!weak_stop_dist_opt.has_value()) return std::nullopt;
    const double target_s = ego_s + weak_stop_dist_opt.value();
    if (target_s < 0.0 || target_s > ego_path.length()) return std::nullopt;
    const auto weak_stop_pose = ego_path.compute(target_s).point.pose;
    return StopCandidate{weak_stop_pose, weak_stop_dist_opt.value()};
  }();
  if (!weak_brake_stop.has_value()) {
    RCLCPP_INFO(
      logger_, "weak_brake_stop is beyond the path horizon. Crosswalk stop will be canceled.");
    return std::nullopt;
  }

  const auto limit_stop = [&]() -> std::optional<StopCandidate> {
    const double limit_stop_dist =
      (first_point_s - ego_s) - base_link2front - planner_param_.stop_distance_from_crosswalk_limit;
    const double target_s = ego_s + limit_stop_dist;
    if (target_s < 0.0 || target_s > ego_path.length()) {
      if (limit_stop_dist < 0.0) {
        return StopCandidate{ego_path.compute(0.0).point.pose, limit_stop_dist};
      }
      return std::nullopt;
    }
    const auto limit_stop_pose = ego_path.compute(target_s).point.pose;
    return StopCandidate{limit_stop_pose, limit_stop_dist};
  }();
  if (!limit_stop.has_value()) {
    RCLCPP_INFO(logger_, "limit_stop is beyond the path horizon. Crosswalk stop will be canceled.");
    return std::nullopt;
  }

  const auto selected_stop = [&]() {
    if (weak_brake_stop->dist < without_acc_pref_stop->dist) {
      return without_acc_pref_stop.value();
    } else if (weak_brake_stop->dist < limit_stop->dist) {
      return weak_brake_stop.value();
    } else {
      return limit_stop.value();
    }
  }();

  const double strong_brake_dist = [&]() {
    const auto strong_brake_dist_opt =
      autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
        ego_vel_non_negative, 0.0, ego_acc, p.min_acc_for_no_stop_decision, 10.0,
        p.min_jerk_for_no_stop_decision);
    return strong_brake_dist_opt ? strong_brake_dist_opt.value() : 0.0;
  }();
  if (p.enable_no_stop_decision && std::max(selected_stop.dist, 0.1) < strong_brake_dist) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "Abandon to stop. "
      "Can not stop against the nearest pedestrian with a specified deceleration. "
      "dist to stop: %f, braking distance: %f",
      selected_stop.dist, strong_brake_dist);
    debug_data_.pass_poses.push_back(selected_stop.pose);
    return std::nullopt;
  }

  return std::make_optional(selected_stop.pose);
}

std::pair<double, double> CrosswalkModule::getAttentionRange(
  const Trajectory & ego_path, const geometry_msgs::msg::Point & first_path_point_on_crosswalk,
  const geometry_msgs::msg::Point & last_path_point_on_crosswalk, const PlannerData & planner_data)
{
  stop_watch_.tic(__func__);

  const auto & ego_pos = planner_data.current_odometry->pose.position;
  const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);
  const auto first_point_s =
    autoware::experimental::trajectory::find_nearest_index(ego_path, first_path_point_on_crosswalk);
  const auto last_point_s =
    autoware::experimental::trajectory::find_nearest_index(ego_path, last_path_point_on_crosswalk);
  const auto near_attention_range =
    first_point_s - ego_s - planner_param_.crosswalk_attention_range;
  const auto far_attention_range = last_point_s - ego_s + planner_param_.crosswalk_attention_range;
  const auto [clamped_near_attention_range, clamped_far_attention_range] =
    clampAttentionRangeByNeighborCrosswalks(
      ego_path, near_attention_range, far_attention_range, planner_data);

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "%s : %f ms", __func__,
    stop_watch_.toc(__func__, true));

  return std::make_pair(
    std::max(0.0, clamped_near_attention_range), std::max(0.0, clamped_far_attention_range));
}

void CrosswalkModule::insertDecelPointWithDebugInfo(
  const geometry_msgs::msg::Point & stop_point, const float target_velocity,
  Trajectory & output) const
{
  const auto stop_s = autoware::experimental::trajectory::find_nearest_index(output, stop_point);
  output.longitudinal_velocity_mps().range(stop_s, output.length()).set(target_velocity);
  const auto stop_pose = output.compute(stop_s).point.pose;
  debug_data_.first_stop_pose = stop_pose;

  if (std::abs(target_velocity) < 1e-3) {
    debug_data_.stop_poses.push_back(stop_pose);
  } else {
    debug_data_.slow_poses.push_back(stop_pose);
  }
}

float CrosswalkModule::calcTargetVelocity(
  const geometry_msgs::msg::Point & stop_point, const Trajectory & ego_path,
  const PlannerData & planner_data) const
{
  const auto max_jerk = planner_param_.max_slow_down_jerk;
  const auto max_accel = planner_param_.max_slow_down_accel;
  const auto & ego_pos = planner_data.current_odometry->pose.position;
  const auto ego_vel = planner_data.current_velocity->twist.linear.x;

  if (ego_vel < planner_param_.no_relax_velocity) {
    return 0.0;
  }

  const auto ego_acc = planner_data.current_acceleration->accel.accel.linear.x;
  const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);
  const auto stop_s = autoware::experimental::trajectory::find_nearest_index(ego_path, stop_point);
  const double dist_deceleration = stop_s - ego_s;
  const auto feasible_velocity = planning_utils::calcDecelerationVelocityFromDistanceToTarget(
    max_jerk, max_accel, ego_acc, ego_vel, dist_deceleration);

  constexpr double margin_velocity = 0.5;  // 1.8 km/h
  float result = margin_velocity < feasible_velocity ? feasible_velocity : 0.0;
  return result;
}

std::pair<double, double> CrosswalkModule::clampAttentionRangeByNeighborCrosswalks(
  const Trajectory & ego_path, const double near_attention_range, const double far_attention_range,
  const PlannerData & planner_data)
{
  stop_watch_.tic(__func__);

  const auto & ego_pos = planner_data.current_odometry->pose.position;

  const auto near_s = near_attention_range;
  const auto far_s = far_attention_range;

  if (near_s < 0.0 || far_s > ego_path.length() || near_s > far_s) {
    return std::make_pair(near_attention_range, far_attention_range);
  }

  const auto discrete_path = ego_path.restore();
  geometry_msgs::msg::Point p_near, p_far;
  p_near = ego_path.compute(near_s).point.pose.position;
  p_far = ego_path.compute(far_s).point.pose.position;

  const auto near_idx = findNearestSegmentIndex(discrete_path, p_near);
  const auto far_idx = findNearestSegmentIndex(discrete_path, p_far) + 1;

  std::set<lanelet::Id> lane_ids;
  for (size_t i = near_idx; i < far_idx; ++i) {
    for (const auto & id : discrete_path.at(i).lane_ids) {
      lane_ids.insert(id);
    }
  }

  lanelet::ConstLanelets crosswalks{};
  constexpr int PEDESTRIAN_GRAPH_ID = 1;
  const auto lanelet_map_ptr = planner_data.route_handler_->getLaneletMapPtr();
  const auto overall_graphs_ptr = planner_data.route_handler_->getOverallGraphPtr();

  for (const auto & id : lane_ids) {
    const auto ll = lanelet_map_ptr->laneletLayer.get(id);
    const auto conflicting_crosswalks =
      overall_graphs_ptr->conflictingInGraph(ll, PEDESTRIAN_GRAPH_ID);
    for (const auto & crosswalk : conflicting_crosswalks) {
      const auto itr = std::find_if(
        crosswalks.begin(), crosswalks.end(),
        [&](const lanelet::ConstLanelet & ll) { return ll.id() == crosswalk.id(); });
      if (itr == crosswalks.end()) {
        crosswalks.push_back(crosswalk);
      }
    }
  }

  sortCrosswalksByDistance(ego_path, ego_pos, crosswalks);

  std::optional<lanelet::ConstLanelet> prev_crosswalk{std::nullopt};
  std::optional<lanelet::ConstLanelet> next_crosswalk{std::nullopt};

  if (!crosswalks.empty()) {
    for (size_t i = 0; i < crosswalks.size() - 1; ++i) {
      const auto ll_front = crosswalks.at(i);
      const auto ll_back = crosswalks.at(i + 1);

      if (ll_front.id() == module_id_ && ll_back.id() != module_id_) {
        next_crosswalk = ll_back;
      }

      if (ll_front.id() != module_id_ && ll_back.id() == module_id_) {
        prev_crosswalk = ll_front;
      }
    }
  }

  const double clamped_near_attention_range = [&]() {
    if (!prev_crosswalk) {
      return near_attention_range;
    }

    const auto path_end_points_on_prev_crosswalk =
      getPathEndPointsOnCrosswalk(ego_path, prev_crosswalk->polygon2d().basicPolygon(), ego_pos);
    if (!path_end_points_on_prev_crosswalk) {
      return near_attention_range;
    }

    const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);
    const auto prev_point_s = autoware::experimental::trajectory::find_nearest_index(
      ego_path, path_end_points_on_prev_crosswalk->first);
    const double dist_to_prev_crosswalk = prev_point_s - ego_s;
    return std::max(near_attention_range, dist_to_prev_crosswalk);
  }();

  const double clamped_far_attention_range = [&]() {
    if (!next_crosswalk) {
      return far_attention_range;
    }
    const auto path_end_points_on_next_crosswalk =
      getPathEndPointsOnCrosswalk(ego_path, next_crosswalk->polygon2d().basicPolygon(), ego_pos);
    if (!path_end_points_on_next_crosswalk) {
      return far_attention_range;
    }

    const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);
    const auto next_point_s = autoware::experimental::trajectory::find_nearest_index(
      ego_path, path_end_points_on_next_crosswalk->first);
    const double dist_to_next_crosswalk = next_point_s - ego_s;
    return std::min(far_attention_range, dist_to_next_crosswalk);
  }();

  const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);
  const auto update_p_near_s = ego_s + clamped_near_attention_range;
  const auto update_p_far_s = ego_s + clamped_far_attention_range;
  const auto update_p_near =
    update_p_near_s >= 0.0 && update_p_near_s <= ego_path.length()
      ? std::optional(ego_path.compute(update_p_near_s).point.pose.position)
      : std::nullopt;
  const auto update_p_far = update_p_far_s >= 0.0 && update_p_far_s <= ego_path.length()
                              ? std::optional(ego_path.compute(update_p_far_s).point.pose.position)
                              : std::nullopt;

  if (update_p_near && update_p_far) {
    debug_data_.range_near_point = update_p_near.value();
    debug_data_.range_far_point = update_p_far.value();
  }

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "%s : %f ms", __func__,
    stop_watch_.toc(__func__, true));

  return std::make_pair(clamped_near_attention_range, clamped_far_attention_range);
}

std::optional<double> CrosswalkModule::findEgoPassageDirectionAlongPath(
  const Trajectory & ego_path) const
{
  auto findIntersectPoint =
    [&](const lanelet::ConstLineString3d line) -> std::optional<geometry_msgs::msg::Point> {
    const auto line_2d = lanelet::utils::to2D(line);
    const auto intersect_s_values =
      autoware::experimental::trajectory::crossed(ego_path, line_2d.basicLineString());
    if (intersect_s_values.empty()) {
      return std::nullopt;
    }
    const auto intersect_pose = ego_path.compute(intersect_s_values.front()).point.pose;
    return intersect_pose.position;
  };
  const auto pt1 = findIntersectPoint(crosswalk_.leftBound());
  const auto pt2 = findIntersectPoint(crosswalk_.rightBound());

  if (!pt1 || !pt2) {
    return std::nullopt;
  }

  return std::atan2(pt2->y - pt1->y, pt2->x - pt1->x);
}

std::optional<double> CrosswalkModule::findObjectPassageDirectionAlongVehicleLane(
  const autoware_perception_msgs::msg::PredictedPath & path) const
{
  using autoware_utils::Segment2d;

  auto findIntersectPoint = [&](const lanelet::ConstLineString3d line)
    -> std::optional<std::pair<size_t, geometry_msgs::msg::Point>> {
    const auto line_start =
      autoware_utils::create_point(line.front().x(), line.front().y(), line.front().z());
    const auto line_end =
      autoware_utils::create_point(line.back().x(), line.back().y(), line.back().z());
    for (unsigned i = 0; i < path.path.size() - 1; ++i) {
      const auto & start = path.path.at(i).position;
      const auto & end = path.path.at(i + 1).position;
      if (const auto intersect = autoware_utils::intersect(line_start, line_end, start, end);
          intersect.has_value()) {
        return std::make_optional(std::make_pair(i, intersect.value()));
      }
    }
    return std::nullopt;
  };
  const auto intersect_pt1 = findIntersectPoint(crosswalk_.leftBound());
  const auto intersect_pt2 = findIntersectPoint(crosswalk_.rightBound());

  if (!intersect_pt1 || !intersect_pt2) {
    return std::nullopt;
  }
  const auto idx1 = intersect_pt1.value().first, idx2 = intersect_pt2.value().first;
  const auto & front = idx1 > idx2 ? intersect_pt2.value().second : intersect_pt1.value().second;
  const auto & back = idx1 > idx2 ? intersect_pt1.value().second : intersect_pt2.value().second;
  return std::atan2(back.y - front.y, back.x - front.x);
}

std::optional<CollisionPoint> CrosswalkModule::getCollisionPoint(
  const Trajectory & ego_path, const PredictedObject & object,
  const std::pair<double, double> & crosswalk_attention_range, const Polygon2d & attention_area,
  const PlannerData & planner_data)
{
  stop_watch_.tic(__func__);

  const auto & obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;

  const auto & ego_pos = planner_data.current_odometry->pose.position;
  const auto & ego_vel = planner_data.current_velocity->twist.linear;

  const auto obj_polygon =
    createObjectPolygon(object.shape.dimensions.x, object.shape.dimensions.y);

  double minimum_stop_dist = std::numeric_limits<double>::max();
  std::optional<CollisionPoint> nearest_collision_point{std::nullopt};
  for (const auto & obj_path : object.kinematics.predicted_paths) {
    size_t start_idx{0};
    bool is_start_idx_initialized{false};
    for (size_t i = 0; i < obj_path.path.size(); ++i) {
      // For effective computation, the point and polygon intersection is calculated first.
      const auto obj_one_step_polygon = createMultiStepPolygon(obj_path.path, obj_polygon, i, i);
      const auto one_step_intersection_polygons =
        calcOverlappingPoints(obj_one_step_polygon, attention_area);
      if (!one_step_intersection_polygons.empty()) {
        if (!is_start_idx_initialized) {
          start_idx = i;
          is_start_idx_initialized = true;
        }
        if (i != obj_path.path.size() - 1) {
          // NOTE: Even if the object path does not fully cross the ego path, the path should be
          // considered.
          continue;
        }
      }

      if (!is_start_idx_initialized) {
        continue;
      }

      // Calculate multi-step object polygon, and reset start_idx
      const size_t start_idx_with_margin = std::max(static_cast<int>(start_idx) - 1, 0);
      const size_t end_idx_with_margin = std::min(i + 1, obj_path.path.size() - 1);
      const auto object_crosswalk_passage_direction =
        findObjectPassageDirectionAlongVehicleLane(obj_path);
      const auto obj_multi_step_polygon = createMultiStepPolygon(
        obj_path.path, obj_polygon, start_idx_with_margin, end_idx_with_margin);
      is_start_idx_initialized = false;

      // Calculate intersection points between object and attention area
      const auto multi_step_intersection_polygons =
        calcOverlappingPoints(obj_multi_step_polygon, attention_area);
      if (multi_step_intersection_polygons.empty()) {
        continue;
      }

      // Calculate nearest collision point among collisions with a predicted path
      Point2d boost_intersection_center_point;
      bg::centroid(multi_step_intersection_polygons.front(), boost_intersection_center_point);
      const auto intersection_center_point = create_point(
        boost_intersection_center_point.x(), boost_intersection_center_point.y(), ego_pos.z);

      const auto dist_ego2cp =
        autoware::experimental::trajectory::find_nearest_index(ego_path, intersection_center_point);
      const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);
      const double relative_dist_ego2cp = dist_ego2cp - ego_s;
      constexpr double eps = 1e-3;
      const auto dist_obj2cp =
        calcArcLength(obj_path.path) < eps
          ? 0.0
          : calcSignedArcLength(obj_path.path, size_t(0), intersection_center_point);

      if (
        relative_dist_ego2cp < crosswalk_attention_range.first ||
        crosswalk_attention_range.second < relative_dist_ego2cp) {
        continue;
      }

      // Calculate nearest collision point among predicted paths
      if (relative_dist_ego2cp < minimum_stop_dist) {
        minimum_stop_dist = relative_dist_ego2cp;
        nearest_collision_point = createCollisionPoint(
          intersection_center_point, relative_dist_ego2cp, dist_obj2cp, ego_vel, obj_vel,
          object_crosswalk_passage_direction, planner_data);
        debug_data_.obj_polygons.push_back(
          toGeometryPointVector(obj_multi_step_polygon, ego_pos.z));
      }

      break;
    }
  }

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "%s : %f ms", __func__,
    stop_watch_.toc(__func__, true));

  return nearest_collision_point;
}

CollisionPoint CrosswalkModule::createCollisionPoint(
  const geometry_msgs::msg::Point & nearest_collision_point, const double dist_ego2cp,
  const double dist_obj2cp, const geometry_msgs::msg::Vector3 & ego_vel,
  const geometry_msgs::msg::Vector3 & obj_vel,
  const std::optional<double> object_crosswalk_passage_direction,
  const PlannerData & planner_data) const
{
  const auto estimated_velocity = std::hypot(obj_vel.x, obj_vel.y);
  const auto velocity = std::max(planner_param_.min_object_velocity, estimated_velocity);

  CollisionPoint collision_point{};
  collision_point.collision_point = nearest_collision_point;
  collision_point.crosswalk_passage_direction = object_crosswalk_passage_direction;

  // The decision of whether the ego vehicle or the pedestrian goes first is determined by the logic
  // for ego_pass_first or yield; even the decision for ego_pass_later does not affect this sense.
  // Hence, here, we use the length that would be appropriate for the ego_pass_first judge.
  collision_point.time_to_collision =
    std::max(0.0, dist_ego2cp - planner_data.vehicle_info_.min_longitudinal_offset_m) /
    std::max(ego_vel.x, planner_param_.ego_min_assumed_speed);
  collision_point.time_to_vehicle = std::max(0.0, dist_obj2cp) / velocity;

  return collision_point;
}

void CrosswalkModule::applySlowDown(
  Trajectory & output, const geometry_msgs::msg::Point & first_path_point_on_crosswalk,
  const geometry_msgs::msg::Point & last_path_point_on_crosswalk,
  const float safety_slow_down_speed, const std::string & reason, const PlannerData & planner_data)
{
  const auto & ego_pos = planner_data.current_odometry->pose.position;
  const auto ego_path = output;
  std::optional<Pose> slowdown_pose{std::nullopt};

  if (!passed_safety_slow_point_) {
    // Safety slow down distance [m]
    const double safety_slow_down_distance =
      crosswalk_.attributeOr("safety_slow_down_distance", 2.0);

    // the range until to the point where ego will have a const safety slow down speed
    const double safety_slow_margin =
      planner_data.vehicle_info_.max_longitudinal_offset_m + safety_slow_down_distance;

    const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);
    const auto first_point_s = autoware::experimental::trajectory::find_nearest_index(
      ego_path, first_path_point_on_crosswalk);
    const double safety_slow_point_range = first_point_s - ego_s - safety_slow_margin;

    const auto p_safety_slow = ego_path.compute(ego_s + safety_slow_point_range);

    insertDecelPointWithDebugInfo(
      p_safety_slow.point.pose.position, safety_slow_down_speed, output);
    slowdown_pose.emplace();
    slowdown_pose->position = p_safety_slow.point.pose.position;

    if (safety_slow_point_range < 0.0) {
      passed_safety_slow_point_ = true;
    }
  } else {
    // the range until to the point where ego will start accelerate
    const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);
    const auto last_point_s = autoware::experimental::trajectory::find_nearest_index(
      ego_path, last_path_point_on_crosswalk);
    const double safety_slow_end_point_range = last_point_s - ego_s;

    if (0.0 < safety_slow_end_point_range) {
      // insert constant ego speed until the end of the crosswalk
      output.longitudinal_velocity_mps().range(ego_s, last_point_s).set(safety_slow_down_speed);
      slowdown_pose = ego_path.compute(ego_s).point.pose;
    }
  }
  if (slowdown_pose) {
    autoware_internal_planning_msgs::msg::SafetyFactorArray safety_factor;

    const auto discrete_points = output.restore();
    planning_factor_interface_->add(
      discrete_points, planner_data.current_odometry->pose, *slowdown_pose,
      autoware_internal_planning_msgs::msg::PlanningFactor::SLOW_DOWN, safety_factor,
      true /*is_driving_forward*/, safety_slow_down_speed, 0.0 /*shift distance*/, reason);
  }
}

void CrosswalkModule::applySlowDownByLanelet2Map(
  Trajectory & output, const geometry_msgs::msg::Point & first_path_point_on_crosswalk,
  const geometry_msgs::msg::Point & last_path_point_on_crosswalk, const PlannerData & planner_data)
{
  if (!crosswalk_.hasAttribute("safety_slow_down_speed")) {
    return;
  }
  applySlowDown(
    output, first_path_point_on_crosswalk, last_path_point_on_crosswalk,
    static_cast<float>(crosswalk_.attribute("safety_slow_down_speed").asDouble().get()), "map",
    planner_data);
}

void CrosswalkModule::applySlowDownByOcclusion(
  Trajectory & output, const geometry_msgs::msg::Point & first_path_point_on_crosswalk,
  const geometry_msgs::msg::Point & last_path_point_on_crosswalk, const PlannerData & planner_data)
{
  const auto & ego_pos = planner_data.current_odometry->pose.position;
  const auto objects_ptr = planner_data.predicted_objects;

  const auto now = clock_->now();
  const auto cmp_with_time_buffer = [&](const auto & t, const auto cmp_fn) {
    return t && cmp_fn((now - *t).seconds(), planner_param_.occlusion_time_buffer);
  };
  const auto crosswalk_has_traffic_light =
    !crosswalk_.regulatoryElementsAs<const lanelet::TrafficLight>().empty();
  const auto is_crosswalk_ignored =
    (planner_param_.occlusion_ignore_with_traffic_light && crosswalk_has_traffic_light) ||
    crosswalk_.hasAttribute("skip_occluded_slowdown");
  if (!planner_param_.occlusion_enable || is_crosswalk_ignored) {
    return;
  }
  const auto ego_s = autoware::experimental::trajectory::find_nearest_index(output, ego_pos);
  const auto first_point_s =
    autoware::experimental::trajectory::find_nearest_index(output, first_path_point_on_crosswalk);
  const double dist_ego_to_crosswalk = first_point_s - ego_s;
  const auto is_ego_on_the_crosswalk =
    dist_ego_to_crosswalk <= planner_data.vehicle_info_.max_longitudinal_offset_m;
  if (is_ego_on_the_crosswalk) {
    return;
  }
  const auto detection_range =
    planner_data.vehicle_info_.max_lateral_offset_m +
    calculate_detection_range(
      planner_param_.occlusion_occluded_object_velocity, dist_ego_to_crosswalk,
      planner_data.current_velocity->twist.linear.x);
  const auto detection_areas = calculate_detection_areas(
    crosswalk_, {first_path_point_on_crosswalk.x, first_path_point_on_crosswalk.y},
    detection_range);
  debug_data_.occlusion_detection_areas = detection_areas;
  debug_data_.crosswalk_origin = first_path_point_on_crosswalk;
  if (is_crosswalk_occluded(
        *planner_data.occupancy_grid, detection_areas, objects_ptr->objects, planner_param_)) {
    if (!current_initial_occlusion_time_) {
      current_initial_occlusion_time_ = now;
    }
    if (cmp_with_time_buffer(current_initial_occlusion_time_, std::greater_equal<double>{})) {
      most_recent_occlusion_time_ = now;
    }
  } else if (!cmp_with_time_buffer(most_recent_occlusion_time_, std::greater<double>{})) {
    current_initial_occlusion_time_.reset();
  }

  if (cmp_with_time_buffer(most_recent_occlusion_time_, std::less_equal<double>{})) {
    const auto target_velocity =
      calcTargetVelocity(first_path_point_on_crosswalk, output, planner_data);
    applySlowDown(
      output, first_path_point_on_crosswalk, last_path_point_on_crosswalk,
      std::max(target_velocity, planner_param_.occlusion_slow_down_velocity), "occlusion",
      planner_data);
    debug_data_.virtual_wall_suffix = " (occluded)";
  } else {
    most_recent_occlusion_time_.reset();
  }
}

Polygon2d CrosswalkModule::getAttentionArea(
  const Trajectory & ego_path, const std::pair<double, double> & crosswalk_attention_range,
  const PlannerData & planner_data) const
{
  const auto & ego_pos = planner_data.current_odometry->pose.position;
  const auto ego_polygon = createVehiclePolygon(planner_data.vehicle_info_);

  constexpr double sample_interval = 4.0;
  const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);

  Polygon2d attention_area;

  for (double s = crosswalk_attention_range.first; s <= crosswalk_attention_range.second;
       s += sample_interval) {
    const double front_s = s;
    const double back_s = std::min(s + sample_interval, crosswalk_attention_range.second);

    if (back_s < crosswalk_attention_range.first) {
      continue;
    }

    if (crosswalk_attention_range.second < front_s) {
      break;
    }

    const double target_s_front = ego_s + front_s;
    const double target_s_back = ego_s + back_s;

    if (
      target_s_front >= 0.0 && target_s_front <= ego_path.length() && target_s_back >= 0.0 &&
      target_s_back <= ego_path.length()) {
      const auto pose_front = ego_path.compute(target_s_front).point.pose;
      const auto pose_back = ego_path.compute(target_s_back).point.pose;

      Polygon2d ego_one_step_polygon;
      offsetPolygon2d(pose_front, ego_polygon, ego_one_step_polygon);
      offsetPolygon2d(pose_back, ego_polygon, ego_one_step_polygon);
      bg::correct(ego_one_step_polygon);

      debug_data_.ego_polygons.push_back(toGeometryPointVector(ego_one_step_polygon, ego_pos.z));
      bg::model::multi_polygon<Polygon2d> unions;
      bg::union_(attention_area, ego_one_step_polygon, unions);
      if (!unions.empty()) {
        attention_area = unions.front();
        bg::correct(attention_area);
      }
    }
  }

  return attention_area;
}

std::optional<StopPoseWithObjectUuids> CrosswalkModule::checkStopForObstructionPrevention(
  const Trajectory & ego_path, const std::vector<PredictedObject> & objects,
  const geometry_msgs::msg::Point & first_path_point_on_crosswalk,
  const geometry_msgs::msg::Point & last_path_point_on_crosswalk,
  const std::optional<geometry_msgs::msg::Pose> & stop_pose, const PlannerData & planner_data)
{
  const auto & p = planner_param_;

  if (!stop_pose) {
    return {};
  }

  // skip stuck vehicle checking for crosswalk which is in intersection.
  if (!p.enable_obstruction_prevention) {
    std::string turn_direction = road_.attributeOr("turn_direction", "else");
    if (turn_direction == "right" || turn_direction == "left" || turn_direction == "straight") {
      if (!road_.regulatoryElementsAs<const lanelet::TrafficLight>().empty()) {
        return {};
      }
    }
  }
  const auto & ego_pos = planner_data.current_odometry->pose.position;
  const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);
  const auto stop_pose_s =
    autoware::experimental::trajectory::find_nearest_index(ego_path, stop_pose->position);

  for (const auto & object : objects) {
    if (!isVehicle(object)) {
      continue;
    }

    const auto & obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;
    if (p.target_vehicle_velocity < std::hypot(obj_vel.x, obj_vel.y)) {
      continue;
    }

    const auto & obj_pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto obj_s =
      autoware::experimental::trajectory::find_nearest_index(ego_path, obj_pose.position);

    const auto nearest_path_pose = ego_path.compute(obj_s).point.pose;
    const auto dx = obj_pose.position.x - nearest_path_pose.position.x;
    const auto dy = obj_pose.position.y - nearest_path_pose.position.y;
    const auto lateral_offset = std::hypot(dx, dy);

    if (p.max_target_vehicle_lateral_offset < std::abs(lateral_offset)) {
      continue;
    }

    // check if STOP is required
    const auto first_point_s = autoware::experimental::trajectory::find_nearest_index(
      ego_path, first_path_point_on_crosswalk);
    const auto last_point_s = autoware::experimental::trajectory::find_nearest_index(
      ego_path, last_path_point_on_crosswalk);
    const double crosswalk_front_to_obj_rear =
      (obj_s - first_point_s) - object.shape.dimensions.x / 2.0;
    const double crosswalk_back_to_obj_rear =
      (obj_s - last_point_s) - object.shape.dimensions.x / 2.0;
    const double required_space_length =
      planner_data.vehicle_info_.vehicle_length_m + planner_param_.required_clearance;

    if (crosswalk_front_to_obj_rear > 0.0 && crosswalk_back_to_obj_rear < required_space_length) {
      // If there exists at least one vehicle ahead, plan STOP considering min_acc, max_jerk and
      // min_jerk. Note that nearest search is not required because the stop pose independents to
      // the vehicles.
      const auto braking_distance = calcDecelDistWithJerkAndAccConstraints(
        planner_data.current_velocity->twist.linear.x, 0.0,
        planner_data.current_acceleration->accel.accel.linear.x, p.min_acc_for_target_vehicle,
        p.max_jerk_for_target_vehicle, p.min_jerk_for_target_vehicle);
      if (!braking_distance) {
        return {};
      }

      const double dist_ego2stop = stop_pose_s - ego_s;
      const double feasible_dist_ego2stop = std::max(*braking_distance, dist_ego2stop);
      const double target_s = ego_s + feasible_dist_ego2stop;

      if (target_s < 0.0 || target_s > ego_path.length()) {
        return {};
      }
      const auto feasible_stop_pose = ego_path.compute(target_s).point.pose;

      setObjectsOfInterestData(obj_pose, object.shape, ColorName::RED);
      return StopPoseWithObjectUuids{feasible_stop_pose, {object.object_id}};
    }
  }

  return {};
}

std::optional<StopPoseWithObjectUuids> CrosswalkModule::checkStopForParkedVehicles(
  const Trajectory & ego_path, const geometry_msgs::msg::Point & first_path_point_on_crosswalk,
  const PlannerData & planner_data)
{
  if (
    !planner_param_.parked_vehicles_stop_enable || isRedSignalForEgo(planner_data) ||
    isRedSignalForPedestrians(planner_data)) {
    parked_vehicles_stop_.reset();
    return std::nullopt;
  }
  const auto & ego_pose = planner_data.current_odometry->pose;
  const auto ego_s =
    autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pose.position);

  if (parked_vehicles_stop_.search_area.empty()) {  // only computed once
    const auto discrete_points = ego_path.restore();
    autoware_internal_planning_msgs::msg::PathWithLaneId discrete_path;
    discrete_path.points = discrete_points;
    const auto lanelets_on_path = planning_utils::getLaneletsOnPath(
      discrete_path, planner_data.route_handler_->getLaneletMapPtr(), ego_pose);
    const auto search_area = create_search_area(
      crosswalk_, lanelets_on_path, first_path_point_on_crosswalk,
      planner_param_.parked_vehicles_stop_search_distance);
    parked_vehicles_stop_.search_area = search_area;
  }
  debug_data_.parked_vehicles_stop_search_area = parked_vehicles_stop_.search_area;
  debug_data_.parked_vehicles_stop_already_stopped = parked_vehicles_stop_.already_stopped;

  bool is_stop_planned = false;
  if (!parked_vehicles_stop_.already_stopped && !parked_vehicles_stop_.search_area.empty()) {
    if (
      const auto zero_vel_s =
        autoware::experimental::trajectory::search_zero_velocity_position(ego_path, ego_s)) {
      const auto zero_vel_point = ego_path.compute(*zero_vel_s).point;
      if (boost::geometry::within(
            lanelet::BasicPoint2d(zero_vel_point.pose.position.x, zero_vel_point.pose.position.y),
            parked_vehicles_stop_.search_area)) {
        is_stop_planned = true;
      }
    }
  }
  const auto skip_condition = parked_vehicles_stop_.already_stopped ||
                              parked_vehicles_stop_.search_area.empty() || is_stop_planned;

  if (skip_condition) {
    // parked_vehicles_stop_.reset();
    return std::nullopt;
  }

  const auto & vehicle_info = planner_data.vehicle_info_;
  const auto ego_footprint = autoware_utils_geometry::to_footprint(
    ego_pose, vehicle_info.max_longitudinal_offset_m, vehicle_info.min_longitudinal_offset_m,
    vehicle_info.max_lateral_offset_m);
  const auto ego_within_search_area =
    boost::geometry::distance(ego_footprint, parked_vehicles_stop_.search_area) <
    planner_param_.parked_vehicles_stop_parked_ego_inside_safe_area_margin;
  const auto ego_close_to_previous_stop_pose =
    parked_vehicles_stop_.previous_stop_pose &&
    autoware_utils_geometry::calc_distance2d(*parked_vehicles_stop_.previous_stop_pose, ego_pose) <=
      planner_param_.parked_vehicles_stop_parked_ego_inside_safe_area_margin;
  const auto already_stopped_condition =
    (ego_within_search_area || ego_close_to_previous_stop_pose) &&
    planner_data.isVehicleStopped(planner_param_.parked_vehicles_stop_min_ego_stop_duration);
  if (already_stopped_condition) {
    parked_vehicles_stop_.already_stopped = true;
    parked_vehicles_stop_.reset();
    return std::nullopt;
  }

  const auto parked_vehicles = get_parked_vehicles(
    planner_data.predicted_objects->objects,
    planner_param_.parked_vehicles_stop_parked_velocity_threshold, isVehicle);
  const auto targets = parked_vehicles_stop_.update_and_add_previous_target_vehicle(
    parked_vehicles, clock_->now(),
    planner_param_.parked_vehicles_stop_vehicle_permanence_duration);
  const auto no_previous_target = !parked_vehicles_stop_.previous_target_vehicle.has_value();

  std::optional<autoware_perception_msgs::msg::PredictedObject> furthest_parked_vehicle{
    std::nullopt};
  geometry_msgs::msg::Point furthest_footprint_point{};
  double max_s_value = -1.0;

  for (const auto & target : targets) {
    const auto & obj_pose = target.kinematics.initial_pose_with_covariance.pose;
    const auto obj_point = obj_pose.position;

    if (boost::geometry::within(
          lanelet::BasicPoint2d(obj_point.x, obj_point.y), parked_vehicles_stop_.search_area)) {
      const auto obj_s =
        autoware::experimental::trajectory::find_nearest_index(ego_path, obj_point);
      if (obj_s > max_s_value) {
        max_s_value = obj_s;
        furthest_parked_vehicle = target;
        furthest_footprint_point = obj_point;
      }
    }
  }

  if (!furthest_parked_vehicle) {
    parked_vehicles_stop_.reset();
    return std::nullopt;
  }
  setObjectsOfInterestData(
    furthest_parked_vehicle->kinematics.initial_pose_with_covariance.pose,
    furthest_parked_vehicle->shape, ColorName::RED);

  const double ego_vel_non_negative = std::max(0.0, planner_data.current_velocity->twist.linear.x);
  const auto min_stop_distance = motion_utils::calcDecelDistWithJerkAndAccConstraints(
    ego_vel_non_negative, 0.0, planner_data.current_acceleration->accel.accel.linear.x,
    planner_param_.min_acc_preferred, 10.0, planner_param_.min_jerk_preferred);
  const auto default_stop_pose =
    getDefaultStopPose(ego_path, first_path_point_on_crosswalk, planner_data);

  const auto furthest_point_s =
    autoware::experimental::trajectory::find_nearest_index(ego_path, furthest_footprint_point);
  const double parked_vehicle_stop_s =
    furthest_point_s - planner_data.vehicle_info_.max_longitudinal_offset_m;
  const auto parked_vehicle_stop_pose = ego_path.compute(parked_vehicle_stop_s).point.pose;

  if (parked_vehicles_stop_.previous_stop_pose) {
    const auto prev_s = autoware::experimental::trajectory::find_nearest_index(
      ego_path, parked_vehicles_stop_.previous_stop_pose->position);
    parked_vehicles_stop_.previous_stop_pose = ego_path.compute(prev_s).point.pose;
  }

  auto stop_factor = calculate_parked_vehicles_stop_factor(
    {default_stop_pose, parked_vehicle_stop_pose}, parked_vehicles_stop_.previous_stop_pose,
    min_stop_distance, [&](const auto & p) {
      const auto p_s = autoware::experimental::trajectory::find_nearest_index(ego_path, p.position);
      return p_s - ego_s;
    });
  if (!stop_factor) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000UL,
      "[%lu][ParkedVehicleStop] could not stop without breaking the jerk and deceleration "
      "constraints",
      module_id_);
    parked_vehicles_stop_.reset();
  } else {
    stop_factor->stop_factor_points.push_back(furthest_footprint_point);
    parked_vehicles_stop_.previous_stop_pose = stop_factor->stop_pose;
    if (no_previous_target) {
      parked_vehicles_stop_.previous_target_vehicle = *furthest_parked_vehicle;
      parked_vehicles_stop_.previous_detection_time = clock_->now();
    }
  }
  return StopPoseWithObjectUuids{stop_factor->stop_pose, {}};
}

std::pair<std::optional<StopPoseWithObjectUuids>, std::string>
CrosswalkModule::getNearestStopFactorAndReason(
  const Trajectory & ego_path,
  const std::optional<StopPoseWithObjectUuids> & stop_factor_for_crosswalk_users,
  const std::optional<StopPoseWithObjectUuids> & stop_factor_for_obstruction_preventions,
  const std::optional<StopPoseWithObjectUuids> & stop_factor_for_parked_vehicles,
  const PlannerData & planner_data)
{
  // the parked vehicles stop feature is only used if there are no other crosswalk stops
  const auto use_parked_vehicles = !stop_factor_for_crosswalk_users &&
                                   !stop_factor_for_obstruction_preventions &&
                                   stop_factor_for_parked_vehicles;
  if (use_parked_vehicles) {
    return {stop_factor_for_parked_vehicles, "parked vehicles"};
  }

  const auto get_distance_to_stop = [&](const auto & stop_factor) -> std::optional<double> {
    const auto & ego_pos = planner_data.current_odometry->pose.position;
    const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);
    const auto stop_s = autoware::experimental::trajectory::find_nearest_index(
      ego_path, stop_factor->stop_pose.position);
    return stop_s - ego_s;
  };

  std::optional<StopPoseWithObjectUuids> nearest_stop_factor{std::nullopt};
  std::string reason = "";

  if (stop_factor_for_crosswalk_users) {
    nearest_stop_factor = stop_factor_for_crosswalk_users;
    reason = "";
  }
  if (stop_factor_for_obstruction_preventions) {
    if (
      !nearest_stop_factor || get_distance_to_stop(stop_factor_for_obstruction_preventions) <
                                get_distance_to_stop(nearest_stop_factor)) {
      nearest_stop_factor = stop_factor_for_obstruction_preventions;
      reason = "obstruction prevention";
    }
  }

  return {nearest_stop_factor, reason};
}

void CrosswalkModule::updateObjectState(
  const double dist_ego_to_stop, const Trajectory & ego_path,
  const std::pair<double, double> & crosswalk_attention_range, const Polygon2d & attention_area,
  const PlannerData & planner_data)
{
  const auto & p = planner_param_;
  const auto & objects_ptr = planner_data.predicted_objects;

  const auto traffic_lights_reg_elems =
    crosswalk_.regulatoryElementsAs<const lanelet::TrafficLight>();
  const bool has_traffic_light = !traffic_lights_reg_elems.empty();

  // Check if ego is yielding
  const bool is_ego_yielding = [&]() {
    const auto has_reached_stop_point = dist_ego_to_stop < p.stop_position_threshold;

    return planner_data.isVehicleStopped(p.timeout_ego_stop_for_yield) && has_reached_stop_point;
  }();

  const auto is_red_signal_for_pedestrians = isRedSignalForPedestrians(planner_data);
  auto ignore_crosswalk = is_red_signal_for_pedestrians;

  // Update object state
  object_info_manager_.init();
  for (const auto & object : objects_ptr->objects) {
    if (!isCrosswalkUserType(object)) {
      continue;
    }

    auto ignore_obj = is_red_signal_for_pedestrians;
    if (p.consider_obj_on_crosswalk_on_red_light && is_red_signal_for_pedestrians) {
      const auto is_object_on_crosswalk =
        bg::intersects(autoware_utils::to_polygon2d(object), crosswalk_.polygon2d().basicPolygon());

      if (is_object_on_crosswalk) {
        ignore_obj = false;
        ignore_crosswalk = false;
      }
    }

    if (ignore_obj) {
      continue;
    }

    const auto obj_uuid = object.object_id;
    const auto & obj_pos = object.kinematics.initial_pose_with_covariance.pose.position;
    const auto & obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;

    // calculate collision point and state
    const auto collision_point =
      getCollisionPoint(ego_path, object, crosswalk_attention_range, attention_area, planner_data);
    const std::optional<double> ego_crosswalk_passage_direction =
      findEgoPassageDirectionAlongPath(ego_path);
    object_info_manager_.update(
      obj_uuid, obj_pos, std::hypot(obj_vel.x, obj_vel.y), objects_ptr->header.stamp,
      is_ego_yielding, has_traffic_light, collision_point, object.classification.front().label, p,
      crosswalk_.polygon2d().basicPolygon(), attention_area, ego_crosswalk_passage_direction);

    const auto collision_state = object_info_manager_.getCollisionState(obj_uuid);
    if (collision_point) {
      debug_data_.collision_points.push_back(
        std::make_tuple(to_hex_string(obj_uuid), *collision_point, collision_state));
    }

    const auto getLabelColor = [](const auto collision_state) {
      if (collision_state == CollisionState::YIELD) {
        return ColorName::RED;
      } else if (
        collision_state == CollisionState::EGO_PASS_FIRST ||
        collision_state == CollisionState::EGO_PASS_LATER) {
        return ColorName::GREEN;
      } else if (collision_state == CollisionState::IGNORE) {
        return ColorName::GRAY;
      } else {
        return ColorName::AMBER;
      }
    };

    setObjectsOfInterestData(
      object.kinematics.initial_pose_with_covariance.pose, object.shape,
      getLabelColor(collision_state));
  }

  debug_data_.ignore_crosswalk = ignore_crosswalk;
  object_info_manager_.finalize(objects_ptr->header.stamp, p);
}

bool CrosswalkModule::isRedSignalForLanelet(
  const lanelet::ConstLanelet & lanelet, const PlannerData & planner_data) const
{
  const auto traffic_lights_reg_elems = lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();

  for (const auto & traffic_lights_reg_elem : traffic_lights_reg_elems) {
    const auto traffic_signal_stamped_opt =
      planner_data.getTrafficSignal(traffic_lights_reg_elem->id());
    if (!traffic_signal_stamped_opt) {
      continue;
    }
    const auto & traffic_signal_stamped = traffic_signal_stamped_opt.value();

    if (
      planner_param_.traffic_light_state_timeout <
      (clock_->now() - traffic_signal_stamped.stamp).seconds()) {
      continue;
    }

    const auto & lights = traffic_signal_stamped.signal.elements;
    if (lights.empty()) {
      continue;
    }

    for (const auto & element : lights) {
      if (element.color == TrafficLightElement::RED && element.shape == TrafficLightElement::CIRCLE)
        return true;
    }
  }
  return false;
}
bool CrosswalkModule::isRedSignalForEgo(const PlannerData & planner_data) const
{
  return isRedSignalForLanelet(road_, planner_data);
}

bool CrosswalkModule::isRedSignalForPedestrians(const PlannerData & planner_data) const
{
  return isRedSignalForLanelet(crosswalk_, planner_data);
}

bool CrosswalkModule::isVehicle(const PredictedObject & object)
{
  if (object.classification.empty()) {
    return false;
  }

  const auto & label = object.classification.front().label;

  if (label == ObjectClassification::CAR) {
    return true;
  }

  if (label == ObjectClassification::TRUCK) {
    return true;
  }

  if (label == ObjectClassification::BUS) {
    return true;
  }

  if (label == ObjectClassification::TRAILER) {
    return true;
  }

  if (label == ObjectClassification::MOTORCYCLE) {
    return true;
  }

  return false;
}

bool CrosswalkModule::isCrosswalkUserType(const PredictedObject & object) const
{
  if (object.classification.empty()) {
    return false;
  }

  const auto & label = object.classification.front().label;

  if (label == ObjectClassification::UNKNOWN && planner_param_.look_unknown) {
    return true;
  }

  if (label == ObjectClassification::BICYCLE && planner_param_.look_bicycle) {
    return true;
  }

  if (label == ObjectClassification::MOTORCYCLE && planner_param_.look_motorcycle) {
    return true;
  }

  if (label == ObjectClassification::PEDESTRIAN && planner_param_.look_pedestrian) {
    return true;
  }

  return false;
}

geometry_msgs::msg::Polygon CrosswalkModule::createObjectPolygon(
  const double width_m, const double length_m)
{
  geometry_msgs::msg::Polygon polygon{};

  polygon.points.push_back(createPoint32(length_m / 2.0, -width_m / 2.0, 0.0));
  polygon.points.push_back(createPoint32(length_m / 2.0, width_m / 2.0, 0.0));
  polygon.points.push_back(createPoint32(-length_m / 2.0, width_m / 2.0, 0.0));
  polygon.points.push_back(createPoint32(-length_m / 2.0, -width_m / 2.0, 0.0));

  return polygon;
}

geometry_msgs::msg::Polygon CrosswalkModule::createVehiclePolygon(
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const auto & i = vehicle_info;
  const auto & front_m = i.max_longitudinal_offset_m;
  const auto & width_m = i.vehicle_width_m / 2.0;
  const auto & back_m = i.rear_overhang_m;

  geometry_msgs::msg::Polygon polygon{};

  polygon.points.push_back(createPoint32(front_m, -width_m, 0.0));
  polygon.points.push_back(createPoint32(front_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-back_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-back_m, -width_m, 0.0));

  return polygon;
}

void CrosswalkModule::setDistanceToStop(
  const Trajectory & ego_path, const std::optional<geometry_msgs::msg::Pose> & default_stop_pose,
  const std::optional<StopPoseWithObjectUuids> & stop_factor, const PlannerData & planner_data)
{
  // calculate stop position
  const auto stop_pos = [&]() -> std::optional<geometry_msgs::msg::Point> {
    if (stop_factor) return stop_factor->stop_pose.position;
    if (default_stop_pose) return default_stop_pose->position;
    return std::nullopt;
  }();

  // Set distance
  if (stop_pos) {
    const auto & ego_pos = planner_data.current_odometry->pose.position;
    const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);
    const auto stop_s = autoware::experimental::trajectory::find_nearest_index(ego_path, *stop_pos);
    const double dist_ego2stop = stop_s - ego_s;
    setDistance(std::max(dist_ego2stop, 0.0));
  } else {
    setDistance(std::numeric_limits<double>::lowest());
  }
}

void CrosswalkModule::planGo(
  Trajectory & ego_path, const std::optional<StopPoseWithObjectUuids> & stop_factor,
  const PlannerData & planner_data) const
{
  if (!stop_factor.has_value()) {
    return;
  }
  // Plan slow down
  const auto target_velocity =
    calcTargetVelocity(stop_factor->stop_pose.position, ego_path, planner_data);
  insertDecelPointWithDebugInfo(
    stop_factor->stop_pose.position,
    std::max(planner_param_.min_slow_down_velocity, target_velocity), ego_path);
}

void CrosswalkModule::planStop(
  Trajectory & ego_path, const std::optional<StopPoseWithObjectUuids> & nearest_stop_factor,
  const std::optional<geometry_msgs::msg::Pose> & default_stop_pose, const std::string & reason,
  const PlannerData & planner_data) const
{
  // Calculate stop factor
  auto stop_factor = [&]() -> std::optional<StopPoseWithObjectUuids> {
    if (nearest_stop_factor) {
      return *nearest_stop_factor;
    }
    if (default_stop_pose) {
      return StopPoseWithObjectUuids{*default_stop_pose, {}};
    }
    return std::nullopt;
  }();

  if (!stop_factor) {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 5000, "stop_factor is null");
    return;
  }

  // Check if the restart should be suppressed.
  const bool suppress_restart = checkRestartSuppression(ego_path, stop_factor, planner_data);
  if (suppress_restart) {
    const auto & ego_pos = planner_data.current_odometry->pose.position;
    const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);
    const auto pose_opt = ego_path.compute(ego_s).point.pose;
    stop_factor->stop_pose = pose_opt;
  }

  const SafetyFactorArray safety_factors = createSafetyFactorArray(stop_factor);

  // Plan stop
  insertDecelPointWithDebugInfo(stop_factor->stop_pose.position, 0.0, ego_path);
  const auto discrete_points = ego_path.restore();
  planning_factor_interface_->add(
    discrete_points, planner_data.current_odometry->pose, stop_factor->stop_pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP, safety_factors,
    true /*is_driving_forward*/, 0.0 /*velocity*/, 0.0 /*shift distance*/, reason);
}

bool CrosswalkModule::checkRestartSuppression(
  const Trajectory & ego_path, const std::optional<StopPoseWithObjectUuids> & stop_factor,
  const PlannerData & planner_data) const
{
  if (!planner_data.isVehicleStopped()) {
    return false;
  }

  const auto & ego_pos = planner_data.current_odometry->pose.position;
  const auto ego_s = autoware::experimental::trajectory::find_nearest_index(ego_path, ego_pos);
  const auto stop_s = autoware::experimental::trajectory::find_nearest_index(
    ego_path, stop_factor->stop_pose.position);
  const double dist_to_stop = stop_s - ego_s;

  // NOTE: min_dist_to_stop_for_restart_suppression is supposed to be the same as
  //      the pid_longitudinal_controller's drive_state_stop_dist.
  return planner_param_.min_dist_to_stop_for_restart_suppression < dist_to_stop &&
         dist_to_stop < planner_param_.max_dist_to_stop_for_restart_suppression;
}

SafetyFactorArray CrosswalkModule::createSafetyFactorArray(
  const std::optional<StopPoseWithObjectUuids> & stop_factor) const
{
  SafetyFactorArray safety_factors;
  safety_factors.header.stamp = clock_->now();
  safety_factors.header.frame_id = "map";

  for (const auto & object_id : stop_factor.value().target_object_ids) {
    autoware_internal_planning_msgs::msg::SafetyFactor safety_factor;

    safety_factor.type = autoware_internal_planning_msgs::msg::SafetyFactor::OBJECT;
    safety_factor.object_id = object_id;

    // TODO(odashima): add a predicted path used for the decision

    if (object_info_manager_.objects.count(object_id) > 0) {
      const auto & object = object_info_manager_.objects.at(object_id);
      safety_factor.ttc_begin = object.collision_point->time_to_collision;
      // TODO(odashima): add a correct value
      safety_factor.ttc_end = object.collision_point->time_to_collision;

      const auto & position = object.position;
      safety_factor.points = {position};
    }

    safety_factor.is_safe = false;

    safety_factors.factors.push_back(safety_factor);
  }

  safety_factors.is_safe = false;
  safety_factors.detail = "";

  return safety_factors;
}

autoware::motion_utils::VirtualWalls CrosswalkModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  wall.text = "crosswalk";
  wall.ns = std::to_string(module_id_) + "_";

  wall.style = autoware::motion_utils::VirtualWallType::stop;
  for (const auto & p : debug_data_.stop_poses) {
    wall.pose = calc_offset_pose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }
  wall.style = autoware::motion_utils::VirtualWallType::slowdown;
  wall.text += debug_data_.virtual_wall_suffix;
  for (const auto & p : debug_data_.slow_poses) {
    wall.pose = calc_offset_pose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }
  wall.style = autoware::motion_utils::VirtualWallType::pass;
  wall.text += debug_data_.virtual_wall_suffix;
  for (const auto & p : debug_data_.pass_poses) {
    wall.pose = calc_offset_pose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }

  return virtual_walls;
}

visualization_msgs::msg::MarkerArray CrosswalkModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  append_marker_array(
    createCrosswalkMarkers(debug_data_, this->clock_->now(), module_id_), &debug_marker_array);

  return debug_marker_array;
}
}  // namespace autoware::behavior_velocity_planner::experimental
