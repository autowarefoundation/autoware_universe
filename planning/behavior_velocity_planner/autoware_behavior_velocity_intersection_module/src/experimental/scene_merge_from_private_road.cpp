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

#include "scene_merge_from_private_road.hpp"

#include "autoware/behavior_velocity_intersection_module/util.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
namespace bg = boost::geometry;

namespace
{
using autoware_utils::append_marker_array;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_orientation;
using autoware_utils::create_marker_scale;

visualization_msgs::msg::MarkerArray createPoseMarkerArray(
  const geometry_msgs::msg::Pose & pose, const std::string & ns, const lanelet::Id id,
  const double r, const double g, const double b)
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
}  // namespace

MergeFromPrivateRoadModule::MergeFromPrivateRoadModule(
  const lanelet::Id module_id, const lanelet::Id lane_id, const PlannerParam & planner_param,
  const std::set<lanelet::Id> & associative_ids, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  lane_id_(lane_id),
  associative_ids_(associative_ids)
{
  planner_param_ = planner_param;
  state_machine_.setState(StateMachine::State::STOP);
}

static std::optional<lanelet::ConstLanelet> getFirstConflictingLanelet(
  const lanelet::ConstLanelets & conflicting_lanelets,
  const InterpolatedPathInfo & interpolated_path_info,
  const autoware_utils::LinearRing2d & footprint, const double vehicle_length)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto [lane_start, end] = interpolated_path_info.lane_id_interval.value();
  const size_t vehicle_length_idx = static_cast<size_t>(vehicle_length / interpolated_path_info.ds);
  const size_t start =
    static_cast<size_t>(std::max<int>(0, static_cast<int>(lane_start) - vehicle_length_idx));

  for (size_t i = start; i <= end; ++i) {
    const auto & pose = path_ip.points.at(i).point.pose;
    const auto path_footprint =
      autoware_utils::transform_vector(footprint, autoware_utils::pose2transform(pose));
    for (const auto & conflicting_lanelet : conflicting_lanelets) {
      const auto polygon_2d = conflicting_lanelet.polygon2d().basicPolygon();
      const bool intersects = bg::intersects(polygon_2d, path_footprint);
      if (intersects) {
        return std::make_optional(conflicting_lanelet);
      }
    }
  }
  return std::nullopt;
}

bool MergeFromPrivateRoadModule::modifyPathVelocity(
  Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data)
{
  auto path_msg = planning_utils::fromTrajectory(path, left_bound, right_bound);

  debug_data_ = DebugData();

  const auto input_path = path_msg;

  StateMachine::State current_state = state_machine_.getState();
  RCLCPP_DEBUG(
    logger_, "lane_id = %ld, state = %s", lane_id_, StateMachine::toString(current_state).c_str());

  /* get current pose */
  geometry_msgs::msg::Pose current_pose = planner_data.current_odometry->pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data.route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data.route_handler_->getRoutingGraphPtr();

  /* spline interpolation */
  const auto interpolated_path_info_opt = util::generateInterpolatedPath(
    lane_id_, associative_ids_, path_msg, planner_param_.path_interpolation_ds, logger_);
  if (!interpolated_path_info_opt) {
    RCLCPP_DEBUG_SKIPFIRST_THROTTLE(logger_, *clock_, 1000 /* ms */, "splineInterpolate failed");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }
  const auto & interpolated_path_info = interpolated_path_info_opt.value();
  if (!interpolated_path_info.lane_id_interval) {
    RCLCPP_WARN(logger_, "Path has no interval on intersection lane %ld", lane_id_);
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }

  const double baselink2front = planner_data.vehicle_info_.max_longitudinal_offset_m;
  const auto local_footprint = planner_data.vehicle_info_.createFootprint(0.0, 0.0);
  if (!first_conflicting_lanelet_) {
    const auto conflicting_lanelets = getAttentionLanelets(planner_data);
    first_conflicting_lanelet_ = getFirstConflictingLanelet(
      conflicting_lanelets, interpolated_path_info, local_footprint, baselink2front);
  }
  if (!first_conflicting_lanelet_) {
    return false;
  }
  const auto first_conflicting_lanelet = first_conflicting_lanelet_.value();

  const auto first_conflicting_idx_opt = util::getFirstPointInsidePolygonByFootprint(
    first_conflicting_lanelet.polygon3d(), interpolated_path_info, local_footprint, baselink2front);
  if (!first_conflicting_idx_opt) {
    return false;
  }
  // ==========================================================================================
  // first_conflicting_idx is calculated considering baselink2front already, so there is no need
  // to subtract baselink2front/ds here
  // ==========================================================================================
  const auto stopline_idx_ip = static_cast<size_t>(std::max<int>(
    0, static_cast<int>(first_conflicting_idx_opt.value()) -
         static_cast<int>(planner_param_.stopline_margin / planner_param_.path_interpolation_ds)));

  const auto stopline_idx_opt = util::insertPointIndex(
    interpolated_path_info.path.points.at(stopline_idx_ip).point.pose, &path_msg,
    planner_data.ego_nearest_dist_threshold, planner_data.ego_nearest_yaw_threshold);
  if (!stopline_idx_opt) {
    RCLCPP_DEBUG(logger_, "failed to insert stopline, ignore planning.");
    return true;
  }
  const auto stopline_idx = stopline_idx_opt.value();

  debug_data_.virtual_wall_pose =
    planning_utils::getAheadPose(stopline_idx, baselink2front, path_msg);
  debug_data_.stop_point_pose = path_msg.points.at(stopline_idx).point.pose;

  /* set stop speed */
  if (state_machine_.getState() == StateMachine::State::STOP) {
    constexpr double v = 0.0;
    planning_utils::setVelocityFromIndex(stopline_idx, v, &path_msg);

    /* get stop point and stop factor */
    const auto & stop_pose = path_msg.points.at(stopline_idx).point.pose;
    planning_factor_interface_->add(
      path_msg.points, planner_data.current_odometry->pose, stop_pose,
      autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
      0.0 /*shift distance*/, "merge_from_private");

    const double signed_arc_dist_to_stop_point = autoware::motion_utils::calcSignedArcLength(
      path_msg.points, current_pose.position, path_msg.points.at(stopline_idx).point.pose.position);

    if (
      signed_arc_dist_to_stop_point < planner_param_.stop_distance_threshold &&
      planner_data.isVehicleStopped(planner_param_.stop_duration_sec)) {
      state_machine_.setState(StateMachine::State::GO);
      if (signed_arc_dist_to_stop_point < -planner_param_.stop_distance_threshold) {
        RCLCPP_ERROR(logger_, "Failed to stop near stop line but ego stopped. Change state to GO");
      }
    }

    planning_utils::toTrajectory(path_msg, path);
    return true;
  }

  planning_utils::toTrajectory(path_msg, path);
  return true;
}

lanelet::ConstLanelets MergeFromPrivateRoadModule::getAttentionLanelets(
  const PlannerData & planner_data) const
{
  const auto lanelet_map_ptr = planner_data.route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data.route_handler_->getRoutingGraphPtr();

  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const auto conflicting_lanelets =
    lanelet::utils::getConflictingLanelets(routing_graph_ptr, assigned_lanelet);
  lanelet::ConstLanelets sibling_lanelets;
  for (const auto & previous_lanelet : routing_graph_ptr->previous(assigned_lanelet)) {
    sibling_lanelets.push_back(previous_lanelet);
    for (const auto & following_lanelet : routing_graph_ptr->following(previous_lanelet)) {
      if (lanelet::utils::contains(sibling_lanelets, following_lanelet)) {
        continue;
      }
      sibling_lanelets.push_back(following_lanelet);
    }
  }

  lanelet::ConstLanelets attention_lanelets;
  for (const auto & conflicting_lanelet : conflicting_lanelets) {
    if (lanelet::utils::contains(sibling_lanelets, conflicting_lanelet)) {
      continue;
    }
    attention_lanelets.push_back(conflicting_lanelet);
  }
  return attention_lanelets;
}

visualization_msgs::msg::MarkerArray MergeFromPrivateRoadModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  const auto state = state_machine_.getState();

  int32_t uid = autoware::behavior_velocity_planner::planning_utils::bitShift(module_id_);
  const auto now = this->clock_->now();
  if (state == StateMachine::State::STOP) {
    append_marker_array(
      createPoseMarkerArray(debug_data_.stop_point_pose, "stop_point_pose", uid, 1.0, 0.0, 0.0),
      &debug_marker_array, now);
  }

  return debug_marker_array;
}

autoware::motion_utils::VirtualWalls MergeFromPrivateRoadModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  const auto state = state_machine_.getState();
  if (state == StateMachine::State::STOP) {
    autoware::motion_utils::VirtualWall wall;
    wall.style = autoware::motion_utils::VirtualWallType::stop;
    wall.pose = debug_data_.virtual_wall_pose;
    wall.text = "merge_from_private_road";
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

}  // namespace autoware::behavior_velocity_planner::experimental
