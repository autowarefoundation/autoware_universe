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

#include "scene_walkway.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
namespace bg = boost::geometry;
using autoware::motion_utils::calcLongitudinalOffsetPose;
using autoware::motion_utils::calcSignedArcLength;
using autoware::motion_utils::findNearestSegmentIndex;
using autoware_utils::create_point;
using autoware_utils::get_pose;

namespace
{
using autoware_utils::append_marker_array;
using autoware_utils::calc_offset_pose;
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;
using visualization_msgs::msg::Marker;

visualization_msgs::msg::MarkerArray createWalkwayMarkers(
  const DebugData & debug_data, const rclcpp::Time & now, const lanelet::Id module_id)
{
  int32_t uid = planning_utils::bitShift(module_id);
  visualization_msgs::msg::MarkerArray msg;

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

  {
    size_t i = 0;
    for (const auto & p : debug_data.stop_poses) {
      auto marker = create_default_marker(
        "map", now, "walkway stop judge range", uid + i++, Marker::LINE_STRIP,
        create_marker_scale(0.1, 0.1, 0.0), create_marker_color(1.0, 0.0, 0.0, 0.5));
      for (size_t j = 0; j < 50; ++j) {
        const auto x = p.position.x + debug_data.stop_judge_range * std::cos(M_PI * 2 / 50 * j);
        const auto y = p.position.y + debug_data.stop_judge_range * std::sin(M_PI * 2 / 50 * j);
        marker.points.push_back(create_point(x, y, p.position.z));
      }
      marker.points.push_back(marker.points.front());
      msg.markers.push_back(marker);
    }
  }

  return msg;
}
}  // namespace

WalkwayModule::WalkwayModule(
  const lanelet::Id module_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const PlannerParam & planner_param, const bool use_regulatory_element,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  module_id_(module_id),
  state_(State::APPROACH),
  planner_param_(planner_param),
  use_regulatory_element_(use_regulatory_element)
{
  if (use_regulatory_element_) {
    const auto reg_elem_ptr = std::dynamic_pointer_cast<const lanelet::autoware::Crosswalk>(
      lanelet_map_ptr->regulatoryElementLayer.get(module_id));
    stop_lines_ = reg_elem_ptr->stopLines();
    walkway_ = reg_elem_ptr->crosswalkLanelet();
  } else {
    const auto stop_line = getStopLineFromMap(module_id_, lanelet_map_ptr, "crosswalk_id");
    if (!!stop_line) {
      stop_lines_.push_back(*stop_line);
    }
    walkway_ = lanelet_map_ptr->laneletLayer.get(module_id);
  }
}

std::pair<double, geometry_msgs::msg::Point> WalkwayModule::getStopLine(
  const PathWithLaneId & ego_path, bool & exist_stopline_in_map,
  const geometry_msgs::msg::Point & first_path_point_on_walkway,
  const PlannerData & planner_data) const
{
  const auto & ego_pos = planner_data.current_odometry->pose.position;
  for (const auto & stop_line : stop_lines_) {
    const auto p_stop_lines =
      getLinestringIntersects(ego_path, lanelet::utils::to2D(stop_line).basicLineString(), ego_pos);
    if (p_stop_lines.empty()) {
      continue;
    }

    exist_stopline_in_map = true;

    const auto dist_ego_to_stop =
      calcSignedArcLength(ego_path.points, ego_pos, p_stop_lines.front());
    return std::make_pair(dist_ego_to_stop, p_stop_lines.front());
  }

  exist_stopline_in_map = false;

  const auto p_stop_line = first_path_point_on_walkway;
  const auto dist_ego_to_stop = calcSignedArcLength(ego_path.points, ego_pos, p_stop_line) -
                                planner_param_.stop_distance_from_crosswalk;
  return std::make_pair(dist_ego_to_stop, p_stop_line);
}

bool WalkwayModule::modifyPathVelocity(
  Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data)
{
  auto path_msg = planning_utils::fromTrajectory(path, left_bound, right_bound);

  const auto & base_link2front = planner_data.vehicle_info_.max_longitudinal_offset_m;

  debug_data_ = DebugData(planner_data);

  const auto input = path_msg;

  const auto & ego_pos = planner_data.current_odometry->pose.position;
  const auto path_end_points_on_walkway =
    getPathEndPointsOnCrosswalk(input, walkway_.polygon2d().basicPolygon(), ego_pos);
  if (!path_end_points_on_walkway) {
    return false;
  }

  const auto & first_path_point_on_walkway = path_end_points_on_walkway->first;

  if (state_ == State::APPROACH) {
    bool exist_stopline_in_map;
    const auto p_stop_line =
      getStopLine(input, exist_stopline_in_map, first_path_point_on_walkway, planner_data);

    const auto & p_stop = p_stop_line.second;
    const auto stop_distance_from_crosswalk =
      exist_stopline_in_map ? 0.0 : planner_param_.stop_distance_from_crosswalk;
    const auto margin = stop_distance_from_crosswalk + base_link2front;
    const auto stop_pose = calcLongitudinalOffsetPose(input.points, p_stop, -margin);

    if (!stop_pose) {
      return false;
    }

    const auto inserted_pose = planning_utils::insertStopPoint(stop_pose->position, path_msg);
    if (inserted_pose) {
      debug_data_.stop_poses.push_back(inserted_pose.value());
    }

    /* get stop point and stop factor */
    planning_factor_interface_->add(
      path_msg.points, planner_data.current_odometry->pose, stop_pose.value(),
      autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/,
      0.0 /*velocity*/, 0.0 /*shift distance*/, "walkway_stop");

    // use arc length to identify if ego vehicle is in front of walkway stop or not.
    const double signed_arc_dist_to_stop_point =
      calcSignedArcLength(input.points, ego_pos, stop_pose->position);

    const double distance_threshold = 1.0;
    debug_data_.stop_judge_range = distance_threshold;

    const auto stop_at_stop_point = signed_arc_dist_to_stop_point < distance_threshold &&
                                    planner_data.isVehicleStopped(planner_param_.stop_duration);

    if (stop_at_stop_point) {
      // If ego vehicle is after walkway stop and stopped then move to stop state
      state_ = State::STOP;
      if (signed_arc_dist_to_stop_point < -distance_threshold) {
        RCLCPP_ERROR(
          logger_, "Failed to stop near walkway but ego stopped. Change state to STOPPED");
      }
    }

    planning_utils::toTrajectory(path_msg, path);
    return true;
  }

  if (state_ == State::STOP) {
    if (planner_data.isVehicleStopped()) {
      state_ = State::SURPASSED;
    }
  }

  return true;
}

autoware::motion_utils::VirtualWalls WalkwayModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  wall.text = "walkway";
  wall.ns = std::to_string(module_id_) + "_";

  wall.style = autoware::motion_utils::VirtualWallType::stop;
  for (const auto & p : debug_data_.stop_poses) {
    wall.pose = calc_offset_pose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

visualization_msgs::msg::MarkerArray WalkwayModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  append_marker_array(
    createWalkwayMarkers(debug_data_, this->clock_->now(), module_id_), &debug_marker_array);

  return debug_marker_array;
}
}  // namespace autoware::behavior_velocity_planner::experimental
