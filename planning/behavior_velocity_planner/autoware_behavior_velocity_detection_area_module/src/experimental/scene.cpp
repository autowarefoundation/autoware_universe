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

#include "scene.hpp"

#include "../utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/arc_lane_util.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/debug.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
using autoware::experimental::lanelet2_utils::to_ros;
using autoware::motion_utils::calcLongitudinalOffsetPose;
using autoware::motion_utils::calcSignedArcLength;
using autoware_utils::append_marker_array;
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;
using std_msgs::msg::ColorRGBA;

namespace
{
lanelet::BasicPoint3d getCentroidPoint(const lanelet::BasicPolygon3d & poly)
{
  lanelet::BasicPoint3d p_sum{0.0, 0.0, 0.0};
  for (const auto & p : poly) {
    p_sum += p;
  }
  return p_sum / poly.size();
}
visualization_msgs::msg::MarkerArray createCorrespondenceMarkerArray(
  const lanelet::autoware::DetectionArea & detection_area_reg_elem, const rclcpp::Time & now,
  ColorRGBA & marker_color)
{
  visualization_msgs::msg::MarkerArray msg;

  const lanelet::ConstLineString3d stop_line = detection_area_reg_elem.stopLine();
  const lanelet::BasicPoint3d stop_line_center_point =
    (stop_line.front().basicPoint() + stop_line.back().basicPoint()) / 2;

  // ID
  {
    auto marker = create_default_marker(
      "map", now, "detection_area_id", detection_area_reg_elem.id(),
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, create_marker_scale(0.0, 0.0, 1.0),
      create_marker_color(1.0, 1.0, 1.0, 0.999));
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto & detection_area : detection_area_reg_elem.detectionAreas()) {
      const auto poly = detection_area.basicPolygon();

      marker.pose.position = to_ros(poly.front());
      marker.pose.position.z += 2.0;
      marker.text = std::to_string(detection_area_reg_elem.id());

      msg.markers.push_back(marker);
    }
  }

  // Polygon
  {
    auto marker = create_default_marker(
      "map", now, "detection_area_polygon", detection_area_reg_elem.id(),
      visualization_msgs::msg::Marker::LINE_LIST, create_marker_scale(0.1, 0.0, 0.0),
      create_marker_color(marker_color.r, marker_color.g, marker_color.b, marker_color.a));
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto & detection_area : detection_area_reg_elem.detectionAreas()) {
      const auto poly = detection_area.basicPolygon();

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

  // Polygon to StopLine
  {
    auto marker = create_default_marker(
      "map", now, "detection_area_correspondence", detection_area_reg_elem.id(),
      visualization_msgs::msg::Marker::LINE_LIST, create_marker_scale(0.1, 0.0, 0.0),
      create_marker_color(marker_color.r, marker_color.g, marker_color.b, marker_color.a));
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto & detection_area : detection_area_reg_elem.detectionAreas()) {
      const auto poly = detection_area.basicPolygon();
      const auto centroid_point = getCentroidPoint(poly);
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

DetectionAreaModule::DetectionAreaModule(
  const lanelet::Id module_id, const lanelet::Id lane_id,
  const lanelet::autoware::DetectionArea & detection_area_reg_elem,
  const PlannerParam & planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterfaceWithRTC(module_id, logger, clock, time_keeper, planning_factor_interface),
  lane_id_(lane_id),
  detection_area_reg_elem_(detection_area_reg_elem),
  state_(State::GO),
  planner_param_(planner_param),
  debug_data_()
{
}

void DetectionAreaModule::print_detected_obstacle(
  const std::vector<geometry_msgs::msg::Point> & obstacle_points,
  const geometry_msgs::msg::Pose & self_pose) const
{
  std::stringstream obstacles_ss;
  if (last_obstacle_found_time_) {
    rclcpp::Duration time_from_last_detection = clock_->now() - *last_obstacle_found_time_;
    constexpr double obstacle_disappear_time_threshold = 0.1;
    if (time_from_last_detection.seconds() > obstacle_disappear_time_threshold) {
      obstacles_ss << std::fixed << std::setprecision(2) << time_from_last_detection.seconds()
                   << "[s] since last obstacle cleared";
    }
  }

  if (!obstacle_points.empty()) {
    const auto p = obstacle_points[0];
    const auto p_from_ego = autoware_utils::inverse_transform_point(p, self_pose);
    obstacles_ss << "obstacle: map_coord(" << std::fixed << std::setprecision(2) << p.x << ", "
                 << p.y << ", " << p.z << "), ego_coord(" << p_from_ego.x << ", " << p_from_ego.y
                 << ", " << p_from_ego.z << ")";
  }

  logInfoThrottle(
    1000, "ego: map_coord(%.2f, %.2f, %.2f), %s", self_pose.position.x, self_pose.position.y,
    self_pose.position.z, obstacles_ss.str().c_str());
}

bool DetectionAreaModule::modifyPathVelocity(
  Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data)
{
  auto path_msg = planning_utils::fromTrajectory(path, left_bound, right_bound);

  // Store original path
  const auto original_path = path_msg;

  // Reset data
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data.vehicle_info_.max_longitudinal_offset_m;

  // Find obstacles in detection area
  bool has_obstacle = false;
  std::string detection_source;

  // Check pointcloud
  if (planner_param_.target_filtering.pointcloud) {
    const auto obstacle_points = detection_area::get_obstacle_points(
      detection_area_reg_elem_.detectionAreas(), *planner_data.no_ground_pointcloud);
    debug_data_.obstacle_points = obstacle_points;

    if (!obstacle_points.empty()) {
      has_obstacle = true;
      detection_source = "pointcloud";
    }
  }

  // Check predicted objects
  if (!has_obstacle && planner_data.predicted_objects) {
    const auto detected_object = detection_area::get_detected_object(
      detection_area_reg_elem_.detectionAreas(), *planner_data.predicted_objects,
      planner_param_.target_filtering);
    if (detected_object.has_value()) {
      has_obstacle = true;

      // Get object type name
      const auto label =
        autoware::object_recognition_utils::getHighestProbLabel(detected_object->classification);
      const auto object_type_name = detection_area::object_label_to_string(label);

      detection_source = object_type_name;
    }
  }

  // Update last obstacle found time
  if (has_obstacle) {
    last_obstacle_found_time_ = std::make_shared<const rclcpp::Time>(clock_->now());
  }

  // Get stop line geometry
  const auto stop_line =
    detection_area::get_stop_line_geometry2d(detection_area_reg_elem_, original_path);

  // Get self pose
  const auto & self_pose = planner_data.current_odometry->pose;
  const size_t current_seg_idx = findEgoSegmentIndex(path_msg.points, planner_data);

  // Get current lanelet and connected lanelets
  const auto connected_lane_ids =
    planning_utils::collectConnectedLaneIds(lane_id_, planner_data.route_handler_);
  // Get stop point
  const auto stop_point = arc_lane_utils::createTargetPoint(
    original_path, stop_line, planner_param_.stop_margin,
    planner_data.vehicle_info_.max_longitudinal_offset_m - forward_offset_to_stop_line_,
    connected_lane_ids);
  if (!stop_point) {
    return true;
  }

  const auto & stop_point_idx = stop_point->first;
  const auto & stop_pose = stop_point->second;
  const size_t stop_line_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
    path_msg.points, stop_pose.position, stop_point_idx);

  auto modified_stop_pose = stop_pose;
  size_t modified_stop_line_seg_idx = stop_line_seg_idx;

  const auto is_stopped = planner_data.isVehicleStopped(0.0);
  const auto stop_dist = calcSignedArcLength(
    path_msg.points, self_pose.position, current_seg_idx, stop_pose.position, stop_line_seg_idx);

  // Don't re-approach when the ego stops closer to the stop point than hold_stop_margin_distance
  if (is_stopped && stop_dist < planner_param_.hold_stop_margin_distance) {
    const auto ego_pos_on_path =
      calcLongitudinalOffsetPose(original_path.points, self_pose.position, 0.0);

    if (!ego_pos_on_path) {
      return false;
    }

    modified_stop_pose = ego_pos_on_path.value();
    modified_stop_line_seg_idx = current_seg_idx;
  }

  setDistance(stop_dist);

  // Check state
  const auto prev_state = state_;  // used to log the state change

  setSafe(
    detection_area::can_clear_stop_state(
      last_obstacle_found_time_, clock_->now(), planner_param_.state_clear_time));
  if (isActivated()) {
    last_obstacle_found_time_ = {};
    if (!planner_param_.suppress_pass_judge_when_stopping || !is_stopped) {
      state_ = State::GO;
      if (prev_state != State::GO) {
        logInfo("state changed: STOP -> GO");
      }
    }
    return true;
  }

  // Force ignore objects after dead_line
  if (planner_param_.use_dead_line) {
    // Use '-' for margin because it's the backward distance from stop line
    const auto dead_line_point = arc_lane_utils::createTargetPoint(
      original_path, stop_line, -planner_param_.dead_line_margin,
      planner_data.vehicle_info_.max_longitudinal_offset_m, connected_lane_ids);

    if (dead_line_point) {
      const size_t dead_line_point_idx = dead_line_point->first;
      const auto & dead_line_pose = dead_line_point->second;

      const size_t dead_line_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
        path_msg.points, dead_line_pose.position, dead_line_point_idx);

      debug_data_.dead_line_poses.push_back(dead_line_pose);

      const double dist_from_ego_to_dead_line = calcSignedArcLength(
        original_path.points, self_pose.position, current_seg_idx, dead_line_pose.position,
        dead_line_seg_idx);
      if (dist_from_ego_to_dead_line < 0.0) {
        logWarn("vehicle is over dead line");
        setSafe(true);
        return true;
      }
    }
  }

  // Ignore objects detected after stop_line if not in STOP state
  const double dist_from_ego_to_stop = calcSignedArcLength(
    original_path.points, self_pose.position, current_seg_idx, stop_pose.position,
    stop_line_seg_idx);
  if (
    state_ != State::STOP &&
    dist_from_ego_to_stop < -planner_param_.distance_to_judge_over_stop_line) {
    setSafe(true);
    return true;
  }

  // Ignore objects if braking distance is not enough
  if (planner_param_.use_pass_judge_line) {
    const auto current_velocity = planner_data.current_velocity->twist.linear.x;
    const double pass_judge_line_distance = planning_utils::calcJudgeLineDistWithAccLimit(
      current_velocity, planner_data.max_stop_acceleration_threshold,
      planner_data.delay_response_time);
    if (
      state_ != State::STOP &&
      !detection_area::has_enough_braking_distance(
        self_pose, stop_point->second, pass_judge_line_distance, current_velocity)) {
      logWarnThrottle(1000, "[detection_area] vehicle is over stop border");
      setSafe(true);
      return true;
    }
  }

  // Insert stop point
  state_ = State::STOP;
  if (prev_state != State::STOP) {
    if (planner_param_.use_max_acceleration) {
      forward_offset_to_stop_line_ = std::max(
        detection_area::feasible_stop_distance_by_max_acceleration(
          planner_data.current_velocity->twist.linear.x, planner_param_.max_acceleration) -
          stop_dist,
        0.0);

      const auto offset_segment = arc_lane_utils::findOffsetSegment(
        original_path, modified_stop_line_seg_idx, forward_offset_to_stop_line_);
      if (offset_segment) {
        modified_stop_pose = arc_lane_utils::calcTargetPose(original_path, *offset_segment);
        modified_stop_line_seg_idx = offset_segment->first;
      }
    }
    logInfo("state changed: GO -> STOP");
  }

  if (state_ == State::STOP && planner_param_.enable_detected_obstacle_logging) {
    print_detected_obstacle(debug_data_.obstacle_points, self_pose);
  }

  planning_utils::insertStopPoint(
    modified_stop_pose.position, modified_stop_line_seg_idx, path_msg);

  // For virtual wall
  debug_data_.stop_poses.push_back(stop_point->second);

  // Create StopReason
  {
    planning_factor_interface_->add(
      path_msg.points, planner_data.current_odometry->pose, stop_pose,
      autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
      0.0 /*shift distance*/, detection_source);
  }

  planning_utils::toTrajectory(path_msg, path);
  return true;
}

visualization_msgs::msg::MarkerArray DetectionAreaModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray wall_marker;
  const rclcpp::Time now = clock_->now();

  if (!debug_data_.stop_poses.empty()) {
    ColorRGBA marker_color;
    marker_color.r = 1.0;
    marker_color.g = 0.0;
    marker_color.b = 0.0;
    marker_color.a = 1.0;

    append_marker_array(
      createCorrespondenceMarkerArray(detection_area_reg_elem_, now, marker_color), &wall_marker);

    append_marker_array(
      debug::createPointsMarkerArray(
        debug_data_.obstacle_points, "obstacles", module_id_, now, 0.6, 0.6, 0.6, 1.0, 0.0, 0.0),
      &wall_marker, now);
  } else {
    ColorRGBA marker_color;
    marker_color.r = 0.0;
    marker_color.g = 1.0;
    marker_color.b = 0.0;
    marker_color.a = 1.0;

    append_marker_array(
      createCorrespondenceMarkerArray(detection_area_reg_elem_, now, marker_color), &wall_marker);
  }

  return wall_marker;
}

autoware::motion_utils::VirtualWalls DetectionAreaModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  wall.text = "detection_area";

  wall.style = autoware::motion_utils::VirtualWallType::stop;
  for (const auto & p : debug_data_.stop_poses) {
    wall.pose = autoware_utils::calc_offset_pose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }

  wall.style = autoware::motion_utils::VirtualWallType::deadline;
  for (const auto & p : debug_data_.dead_line_poses) {
    wall.pose = autoware_utils::calc_offset_pose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}
}  // namespace autoware::behavior_velocity_planner::experimental
