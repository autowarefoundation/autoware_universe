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

#include "scene_no_stopping_area.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/debug.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <cmath>
#include <memory>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
namespace bg = boost::geometry;

namespace
{
constexpr auto marker_lifetime = 0.2;
using autoware::experimental::lanelet2_utils::to_ros;
using autoware_utils::append_marker_array;
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;
using no_stopping_area::DebugData;

lanelet::BasicPoint3d get_centroid_point(const lanelet::BasicPolygon3d & poly)
{
  lanelet::BasicPoint3d p_sum{0.0, 0.0, 0.0};
  for (const auto & p : poly) {
    p_sum += p;
  }
  return p_sum / poly.size();
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

      marker.pose.position = to_ros(poly.front());
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

        marker.points.push_back(to_ros(p_front));
        marker.points.push_back(to_ros(p_back));
      }
    }
    msg.markers.push_back(marker);
  }

  const auto & stop_line = no_stopping_area_reg_elem.stopLine();
  // Polygon to StopLine
  if (stop_line) {
    const lanelet::BasicPoint3d stop_line_center_point =
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
        marker.points.push_back(to_ros(centroid_point));
        marker.points.push_back(to_ros(stop_line_center_point));
      }
    }
    msg.markers.push_back(marker);
  }

  return msg;
}
}  // namespace

NoStoppingAreaModule::NoStoppingAreaModule(
  const lanelet::Id module_id, const lanelet::Id lane_id,
  const lanelet::autoware::NoStoppingArea & no_stopping_area_reg_elem,
  const PlannerParam & planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterfaceWithRTC(module_id, logger, clock, time_keeper, planning_factor_interface),
  lane_id_(lane_id),
  no_stopping_area_reg_elem_(no_stopping_area_reg_elem),
  planner_param_(planner_param),
  debug_data_()
{
  state_machine_.setState(StateMachine::State::GO);
  state_machine_.setMarginTime(planner_param_.state_clear_time);
}

bool NoStoppingAreaModule::modifyPathVelocity(
  Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data)
{
  auto path_msg = planning_utils::fromTrajectory(path, left_bound, right_bound);

  // Store original path
  const auto original_path = path_msg;
  const auto & predicted_obj_arr_ptr = planner_data.predicted_objects;
  const auto & current_pose = planner_data.current_odometry;

  // Reset data
  debug_data_ = no_stopping_area::DebugData();
  debug_data_.base_link2front = planner_data.vehicle_info_.max_longitudinal_offset_m;

  const no_stopping_area::EgoData ego_data(planner_data);

  // Get stop line geometry
  const auto stop_line = no_stopping_area::get_stop_line_geometry2d(
    original_path, no_stopping_area_reg_elem_, planner_param_.stop_line_margin,
    planner_data.vehicle_info_.vehicle_width_m);
  if (!stop_line) {
    setSafe(true);
    return true;
  }
  const auto stop_point = arc_lane_utils::createTargetPoint(
    original_path, stop_line.value(), planner_param_.stop_margin,
    planner_data.vehicle_info_.max_longitudinal_offset_m, {lane_id_});
  if (!stop_point) {
    setSafe(true);
    return true;
  }
  const auto & stop_pose = stop_point->second;
  setDistance(
    autoware::motion_utils::calcSignedArcLength(
      original_path.points, current_pose->pose.position, stop_pose.position));
  if (planning_utils::isOverLine(
        original_path, current_pose->pose, stop_pose, planner_param_.dead_line_margin)) {
    // ego can't stop in front of no stopping area -> GO or OR
    state_machine_.setState(StateMachine::State::GO);
    setSafe(true);
    return true;
  }
  const auto & vi = planner_data.vehicle_info_;
  const double margin = planner_param_.stop_line_margin;
  const double ego_space_in_front_of_stuck_vehicle =
    margin + vi.vehicle_length_m + planner_param_.stuck_vehicle_front_margin;
  const Polygon2d stuck_vehicle_detect_area =
    no_stopping_area::generate_ego_no_stopping_area_lane_polygon(
      path_msg, current_pose->pose, no_stopping_area_reg_elem_, ego_space_in_front_of_stuck_vehicle,
      planner_param_.detection_area_length, planner_param_.path_expand_width, logger_, *clock_);
  const double ego_space_in_front_of_stop_line =
    margin + planner_param_.stop_margin + vi.rear_overhang_m;
  const Polygon2d stop_line_detect_area =
    no_stopping_area::generate_ego_no_stopping_area_lane_polygon(
      path_msg, current_pose->pose, no_stopping_area_reg_elem_, ego_space_in_front_of_stop_line,
      planner_param_.detection_area_length, planner_param_.path_expand_width, logger_, *clock_);
  if (stuck_vehicle_detect_area.outer().empty() && stop_line_detect_area.outer().empty()) {
    setSafe(true);
    return true;
  }
  debug_data_.stuck_vehicle_detect_area = toGeomPoly(stuck_vehicle_detect_area);
  debug_data_.stop_line_detect_area = toGeomPoly(stop_line_detect_area);
  // Find stuck vehicle in no stopping area
  const bool is_entry_prohibited_by_stuck_vehicle =
    check_stuck_vehicles_in_no_stopping_area(stuck_vehicle_detect_area, predicted_obj_arr_ptr);
  // Find stop line in no stopping area
  const bool is_entry_prohibited_by_stop_line =
    no_stopping_area::check_stop_lines_in_no_stopping_area(
      path_msg, stop_line_detect_area, debug_data_);
  const bool is_entry_prohibited =
    is_entry_prohibited_by_stuck_vehicle || is_entry_prohibited_by_stop_line;
  if (!no_stopping_area::is_stoppable(
        pass_judge_, current_pose->pose, stop_point->second, ego_data, logger_, *clock_)) {
    state_machine_.setState(StateMachine::State::GO);
    setSafe(true);
    return false;
  }

  state_machine_.setStateWithMarginTime(
    is_entry_prohibited ? StateMachine::State::STOP : StateMachine::State::GO,
    logger_.get_child("state_machine"), *clock_);

  setSafe(state_machine_.getState() != StateMachine::State::STOP);
  if (!isActivated()) {
    // ----------------stop reason and stop point--------------------------
    no_stopping_area::insert_stop_point(path_msg, *stop_point);
    // For virtual wall
    debug_data_.stop_poses.push_back(stop_pose);

    // Create StopReason
    {
      planning_factor_interface_->add(
        path_msg.points, planner_data.current_odometry->pose, stop_point->second,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
        autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
        0.0 /*shift distance*/, "");
    }

  } else if (state_machine_.getState() == StateMachine::State::GO) {
    // reset pass judge if current state is go
    pass_judge_.is_stoppable = true;
    pass_judge_.pass_judged = false;
  }

  planning_utils::toTrajectory(path_msg, path);
  return true;
}

bool NoStoppingAreaModule::check_stuck_vehicles_in_no_stopping_area(
  const Polygon2d & poly,
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & predicted_obj_arr_ptr)
{
  // stuck points by predicted objects
  for (const auto & object : predicted_obj_arr_ptr->objects) {
    if (!no_stopping_area::is_vehicle_type(object)) {
      continue;  // not target vehicle type
    }
    const auto obj_v = std::fabs(object.kinematics.initial_twist_with_covariance.twist.linear.x);
    if (obj_v > planner_param_.stuck_vehicle_vel_thr) {
      setObjectsOfInterestData(
        object.kinematics.initial_pose_with_covariance.pose, object.shape, ColorName::GREEN);
      continue;  // not stop vehicle
    }
    // check if the footprint is in the stuck detect area
    const Polygon2d obj_footprint = autoware_utils::to_polygon2d(object);
    const bool is_in_stuck_area = !bg::disjoint(obj_footprint, poly);
    if (is_in_stuck_area) {
      RCLCPP_DEBUG(logger_, "stuck vehicle found.");
      setObjectsOfInterestData(
        object.kinematics.initial_pose_with_covariance.pose, object.shape, ColorName::RED);
      for (const auto & p : obj_footprint.outer()) {
        geometry_msgs::msg::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = 0.0;
        debug_data_.stuck_points.emplace_back(point);
      }
      return true;
    }
  }
  return false;
}

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
}  // namespace autoware::behavior_velocity_planner::experimental
