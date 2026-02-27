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

#include "scene.hpp"

#include <autoware/trajectory/utils/find_nearest.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <memory>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace
{
visualization_msgs::msg::MarkerArray createNoDrivableLaneMarkers(
  const NoDrivableLaneModule::DebugData & debug_data, const rclcpp::Time & now,
  const lanelet::Id module_id)
{
  using autoware_utils::create_default_marker;
  using autoware_utils::create_marker_color;
  using autoware_utils::create_marker_scale;
  using autoware_utils::create_point;
  using visualization_msgs::msg::Marker;

  visualization_msgs::msg::MarkerArray msg;
  const int32_t uid = planning_utils::bitShift(module_id);

  // No Drivable Lane Polygon
  if (!debug_data.no_drivable_lane_polygon.empty()) {
    auto marker = create_default_marker(
      "map", now, "no_drivable_lane polygon", uid, Marker::LINE_STRIP,
      create_marker_scale(0.1, 0.0, 0.0), create_marker_color(1.0, 1.0, 1.0, 0.999));
    for (const auto & p : debug_data.no_drivable_lane_polygon) {
      marker.points.push_back(create_point(p.x, p.y, p.z));
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  // Path - polygon intersection points
  {
    auto marker = create_default_marker(
      "map", now, "path_polygon intersection points", uid, Marker::POINTS,
      create_marker_scale(0.25, 0.25, 0.0), create_marker_color(1.0, 0.0, 0.0, 0.999));
    const auto & p_first = debug_data.first_intersection_point;
    if (p_first) {
      marker.points.push_back(create_point(p_first->x, p_first->y, p_first->z));
    }
    const auto & p_second = debug_data.second_intersection_point;
    if (p_second) {
      marker.points.push_back(create_point(p_second->x, p_second->y, p_second->z));
    }
    if (!marker.points.empty()) msg.markers.push_back(marker);
  }

  return msg;
}
}  // namespace

using autoware_utils::create_point;

NoDrivableLaneModule::NoDrivableLaneModule(
  const lanelet::Id module_id, const lanelet::Id lane_id, const PlannerParam & planner_param,
  const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  lane_id_(lane_id),
  planner_param_(planner_param),
  debug_data_(),
  state_(State::INIT)
{
}

bool NoDrivableLaneModule::modifyPathVelocity(
  Trajectory & path, [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & left_bound,
  [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & right_bound,
  const PlannerData & planner_data)
{
  const auto & ego_pos = planner_data.current_odometry->pose.position;
  const auto & lanelet_map_ptr = planner_data.route_handler_->getLaneletMapPtr();
  const auto & no_drivable_lane = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const auto & no_drivable_lane_polygon =
    lanelet::utils::to2D(no_drivable_lane).polygon2d().basicPolygon();

  const auto path_polygon_intersection =
    no_drivable_lane::getPathIntersectionWithNoDrivableLanePolygon(
      path, no_drivable_lane_polygon, 2);

  const auto ego_front_s = autoware::experimental::trajectory::find_nearest_index(path, ego_pos) +
                           planner_data.vehicle_info_.max_longitudinal_offset_m;

  initialize_debug_data(path, no_drivable_lane, ego_pos, path_polygon_intersection, planner_data);

  switch (state_) {
    case State::INIT: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "Init");
      }

      handle_init_state(ego_front_s, path_polygon_intersection);

      break;
    }

    case State::APPROACHING: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "Approaching ");
      }

      handle_approaching_state(path, ego_front_s, path_polygon_intersection, planner_data);

      break;
    }

    case State::INSIDE_NO_DRIVABLE_LANE: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "INSIDE_NO_DRIVABLE_LANE");
      }

      handle_inside_no_drivable_lane_state(path, ego_front_s, planner_data);

      break;
    }

    case State::STOPPED: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "STOPPED");
      }

      handle_stopped_state(path, ego_front_s, planner_data);

      break;
    }

    default: {
      RCLCPP_ERROR(logger_, "ERROR. Undefined case");
      return false;
    }
  }

  return true;
}

void NoDrivableLaneModule::handle_init_state(
  const double ego_front_s, const no_drivable_lane::PolygonIntersection & path_polygon_intersection)
{
  if (
    path_polygon_intersection.is_first_path_point_inside_polygon ||
    (path_polygon_intersection.first_intersection_s &&
     *path_polygon_intersection.first_intersection_s - ego_front_s <= planner_param_.stop_margin)) {
    state_ = State::INSIDE_NO_DRIVABLE_LANE;
  } else {
    state_ = State::APPROACHING;
  }
}

void NoDrivableLaneModule::handle_approaching_state(
  Trajectory & path, const double ego_front_s,
  const no_drivable_lane::PolygonIntersection & path_polygon_intersection,
  const PlannerData & planner_data)
{
  if (!path_polygon_intersection.first_intersection_s) {
    return;
  }

  const auto longitudinal_offset =
    -(planner_param_.stop_margin + planner_data.vehicle_info_.max_longitudinal_offset_m);
  const auto stop_s = *path_polygon_intersection.first_intersection_s + longitudinal_offset;

  path.longitudinal_velocity_mps().range(stop_s, path.length()).set(0.0);

  // Get stop point and stop factor
  planning_factor_interface_->add(
    path.restore(), planner_data.current_odometry->pose, path.compute(stop_s).point.pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "");

  debug_data_.stop_pose = path.compute(stop_s + debug_data_.base_link2front).point.pose;

  // Move to stopped state if stopped
  const auto distance_ego_first_intersection =
    *path_polygon_intersection.first_intersection_s - ego_front_s;
  if (
    (distance_ego_first_intersection <= planner_param_.stop_margin) &&
    planner_data.isVehicleStopped()) {
    if (planner_param_.print_debug_info) {
      RCLCPP_INFO(logger_, "APPROACHING -> STOPPED");
      RCLCPP_INFO_STREAM(
        logger_, "distance_ego_first_intersection = " << distance_ego_first_intersection);
    }

    if (distance_ego_first_intersection < 0.0) {
      RCLCPP_ERROR(
        logger_, "Failed to stop before no drivable lane but ego stopped. Change state to STOPPED");
    }

    state_ = State::STOPPED;
  }
}

void NoDrivableLaneModule::handle_inside_no_drivable_lane_state(
  Trajectory & path, const double ego_front_s, const PlannerData & planner_data)
{
  // Insert stop point
  path.longitudinal_velocity_mps()
    .range(ego_front_s - planner_data.vehicle_info_.max_longitudinal_offset_m, path.length())
    .set(0.0);

  // Get stop point and stop factor
  planning_factor_interface_->add(
    path.restore(), planner_data.current_odometry->pose, path.compute(0).point.pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "");

  debug_data_.stop_pose = path.compute(debug_data_.base_link2front).point.pose;

  // Move to stopped state if stopped
  if (planner_data.isVehicleStopped()) {
    if (planner_param_.print_debug_info) {
      RCLCPP_INFO(logger_, "APPROACHING -> STOPPED");
    }
    state_ = State::STOPPED;
  }
}

void NoDrivableLaneModule::handle_stopped_state(
  Trajectory & path, const double ego_front_s, const PlannerData & planner_data)
{
  // Insert stop pose
  const auto ego_s = ego_front_s - planner_data.vehicle_info_.max_longitudinal_offset_m;
  path.longitudinal_velocity_mps().range(ego_s, path.length()).set(0.0);

  // Get stop point and stop factor
  planning_factor_interface_->add(
    path.restore(), planner_data.current_odometry->pose, path.compute(ego_s).point.pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "");

  debug_data_.stop_pose = path.compute(ego_s + debug_data_.base_link2front).point.pose;
}

void NoDrivableLaneModule::initialize_debug_data(
  const Trajectory & path, const lanelet::Lanelet & no_drivable_lane,
  const geometry_msgs::msg::Point & ego_pos,
  const no_drivable_lane::PolygonIntersection & path_polygon_intersection,
  const PlannerData & planner_data)
{
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data.vehicle_info_.max_longitudinal_offset_m;
  if (path_polygon_intersection.first_intersection_s) {
    debug_data_.first_intersection_point =
      path.compute(*path_polygon_intersection.first_intersection_s).point.pose.position;
  }
  if (path_polygon_intersection.second_intersection_s) {
    debug_data_.second_intersection_point =
      path.compute(*path_polygon_intersection.second_intersection_s).point.pose.position;
  }

  for (const auto & p : no_drivable_lane.polygon2d().basicPolygon()) {
    debug_data_.no_drivable_lane_polygon.push_back(create_point(p.x(), p.y(), ego_pos.z));
  }
}

autoware::motion_utils::VirtualWalls NoDrivableLaneModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;

  const auto now = this->clock_->now();

  if (
    (state_ == State::APPROACHING) || (state_ == State::INSIDE_NO_DRIVABLE_LANE) ||
    (state_ == State::STOPPED)) {
    autoware::motion_utils::VirtualWall wall;
    wall.text = "no_drivable_lane";
    wall.style = autoware::motion_utils::VirtualWallType::stop;
    wall.ns = std::to_string(module_id_) + "_";
    wall.pose = debug_data_.stop_pose;
    virtual_walls.push_back(wall);
  }

  return virtual_walls;
}

visualization_msgs::msg::MarkerArray NoDrivableLaneModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;
  const auto now = this->clock_->now();

  autoware_utils::append_marker_array(
    createNoDrivableLaneMarkers(debug_data_, this->clock_->now(), module_id_), &debug_marker_array);

  return debug_marker_array;
}

}  // namespace autoware::behavior_velocity_planner
