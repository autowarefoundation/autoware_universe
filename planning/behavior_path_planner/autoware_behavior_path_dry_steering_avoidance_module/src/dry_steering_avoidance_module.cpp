// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_dry_steering_avoidance_module/dry_steering_avoidance_module.hpp"

#include "autoware/behavior_path_planner_common/marker_utils/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <limits>
#include <memory>

namespace autoware::behavior_path_planner
{

using autoware::motion_utils::calcSignedArcLength;
using autoware_utils::calc_distance2d;
using autoware_utils::calc_offset_pose;
using lanelet::utils::getArcCoordinates;

DrySteeringAvoidanceModule::DrySteeringAvoidanceModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<DrySteeringAvoidanceParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface)
: SceneModuleInterface{name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map, planning_factor_interface},
  parameters_{parameters},
  vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo()}
{
  auto boundary_departure_checker_params = autoware::boundary_departure_checker::Param{};
  boundary_departure_checker_params.footprint_extra_margin =
    parameters_->lane_departure_check_expansion_margin;

  boundary_departure_checker_ =
    std::make_shared<autoware::boundary_departure_checker::BoundaryDepartureChecker>(
      boundary_departure_checker_params, vehicle_info_);

  geometric_planner_.setParameters(parameters_->parallel_parking_parameters);

  RCLCPP_WARN(getLogger(), "[DrySteeringAvoidance] Module constructed!");
}

bool DrySteeringAvoidanceModule::isExecutionRequested() const
{
  const bool is_stopped = isStopped();
  const bool has_object = hasStaticObjectInFront();

  // Check basic conditions - if not met, reset timer and return false
  if (!is_stopped || !has_object) {
    // Reset timer when conditions are not met
    const_cast<DrySteeringAvoidanceModule *>(this)->condition_check_started_ = false;
    return false;
  }

  // Continue if already running and conditions are still met
  if (avoidance_path_.has_value()) {
    return true;
  }

  // Check time condition (const cast for mutable state)
  const bool activation_ok =
    const_cast<DrySteeringAvoidanceModule *>(this)->checkActivationCondition();
  return activation_ok;
}

bool DrySteeringAvoidanceModule::isExecutionReady() const
{
  return true;
}

void DrySteeringAvoidanceModule::processOnEntry()
{
  resetPathCandidate();
  resetPathReference();
  debug_marker_.markers.clear();
  info_marker_.markers.clear();

  start_pose_ = getEgoPose();
  avoidance_path_ = planAvoidancePath();
  drivable_lanes_ = generateDrivableLanes();
}

void DrySteeringAvoidanceModule::processOnExit()
{
  avoidance_path_.reset();
  drivable_lanes_.clear();
  condition_check_started_ = false;
  resetPathCandidate();
  resetPathReference();
  debug_marker_.markers.clear();
  info_marker_.markers.clear();
}

void DrySteeringAvoidanceModule::updateData()
{
  // Path is planned in processOnEntry(), nothing to do here
}

BehaviorModuleOutput DrySteeringAvoidanceModule::plan()
{
  // Set debug markers
  setDebugData();

  // Return previous module output if no valid path
  if (!avoidance_path_.has_value()) {
    RCLCPP_DEBUG(getLogger(), "No avoidance path available, returning previous output");
    return getPreviousModuleOutput();
  }

  BehaviorModuleOutput output;
  output.path = getCurrentPath();
  output.reference_path = getPreviousModuleOutput().reference_path;
  output.turn_signal_info = calcTurnSignalInfo();

  // Left boundary: current lane's left, Right boundary: rightmost lane's right
  const auto & dp = planner_data_->drivable_area_expansion_parameters;
  const auto expanded_lanes = utils::expandLanelets(
    drivable_lanes_, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  DrivableAreaInfo current_drivable_area_info;
  current_drivable_area_info.drivable_lanes = expanded_lanes;
  output.drivable_area_info = utils::combineDrivableAreaInfo(
    current_drivable_area_info, getPreviousModuleOutput().drivable_area_info);

  // Update RTC status (auto-approval)
  const auto & path = output.path;
  const double start_distance =
    calcSignedArcLength(path.points, getEgoPosition(), avoidance_path_->start_pose.position);
  const double finish_distance =
    calcSignedArcLength(path.points, getEgoPosition(), avoidance_path_->end_pose.position);
  updateRTCStatus(std::max(0.0, start_distance), finish_distance);

  return output;
}

CandidateOutput DrySteeringAvoidanceModule::planCandidate() const
{
  CandidateOutput output;

  if (!avoidance_path_.has_value()) {
    return output;
  }

  output.path_candidate = getCurrentPath();
  output.start_distance_to_path_change = calcSignedArcLength(
    output.path_candidate.points, getEgoPosition(), avoidance_path_->start_pose.position);
  output.finish_distance_to_path_change = calcSignedArcLength(
    output.path_candidate.points, getEgoPosition(), avoidance_path_->end_pose.position);

  return output;
}

bool DrySteeringAvoidanceModule::canTransitSuccessState()
{
  return false;
}

bool DrySteeringAvoidanceModule::canTransitFailureState()
{
  return false;
}

bool DrySteeringAvoidanceModule::isStopped() const
{
  const double speed = getEgoSpeed();
  const bool stopped = speed < parameters_->th_stopped_velocity;
  return stopped;
}

bool DrySteeringAvoidanceModule::hasStaticObjectInFront() const
{
  return findNearestStaticObjectInCurrentLanes().has_value();
}

bool DrySteeringAvoidanceModule::checkActivationCondition()
{
  const bool condition_met = isStopped() && hasStaticObjectInFront();

  if (!condition_met) {
    condition_check_started_ = false;
    return false;
  }

  if (!condition_check_started_) {
    condition_start_time_ = clock_->now();
    condition_check_started_ = true;
    return false;
  }

  const double elapsed_time = (clock_->now() - condition_start_time_).seconds();
  const bool activated = elapsed_time >= parameters_->th_stopped_time;
  return activated;
}

std::optional<PredictedObject> DrySteeringAvoidanceModule::findNearestStaticObjectInCurrentLanes()
  const
{
  const auto & objects = planner_data_->dynamic_object;
  if (!objects) {
    return std::nullopt;
  }

  const auto current_lanes = getCurrentLanes();
  if (current_lanes.empty()) {
    RCLCPP_WARN(getLogger(), "[DrySteeringAvoidance] findNearest: current_lanes empty");
    return std::nullopt;
  }

  std::optional<PredictedObject> nearest_object;
  double min_distance = std::numeric_limits<double>::max();

  for (const auto & obj : objects->objects) {
    // Check if object is static
    const double obj_vel = std::hypot(
      obj.kinematics.initial_twist_with_covariance.twist.linear.x,
      obj.kinematics.initial_twist_with_covariance.twist.linear.y);

    const auto & obj_pose = obj.kinematics.initial_pose_with_covariance.pose;

    // Check longitudinal distance from ego
    const double longitudinal_distance = calcLongitudinalDistanceFromEgo(obj_pose, current_lanes);

    if (obj_vel > parameters_->th_static_object_velocity) {
      continue;
    }

    if (longitudinal_distance <= 0.0) {
      continue;
    }

    if (longitudinal_distance >= parameters_->object_search_forward_distance) {
      continue;
    }

    if (longitudinal_distance < min_distance) {
      min_distance = longitudinal_distance;
      nearest_object = obj;
    }
  }

  return nearest_object;
}

double DrySteeringAvoidanceModule::getObjectLateralEdgeOffset(
  const PredictedObject & object, const bool right_side) const
{
  const auto current_lanes = getCurrentLanes();
  if (current_lanes.empty()) {
    return 0.0;
  }

  const auto & obj_pose = object.kinematics.initial_pose_with_covariance.pose;

  // Get object's arc coordinates
  const auto arc_coords = getArcCoordinates(current_lanes, obj_pose);

  // Get object dimensions
  const double half_width = object.shape.dimensions.y / 2.0;

  // Calculate lateral edge offset
  if (right_side) {
    return arc_coords.distance - half_width;  // Right edge (smaller lateral value)
  } else {
    return arc_coords.distance + half_width;  // Left edge (larger lateral value)
  }
}

std::vector<Pose> DrySteeringAvoidanceModule::generateEndPoseCandidates(
  const PredictedObject & object) const
{
  std::vector<Pose> candidates;

  const auto current_lanes = getCurrentLanes();
  if (current_lanes.empty()) {
    RCLCPP_WARN(getLogger(), "[DrySteeringAvoidance] generateEndPose: current_lanes empty");
    return candidates;
  }

  const bool avoid_right = (parameters_->avoidance_direction == "right");

  // Get object's center arc coordinates on the lane
  const auto & obj_pose = object.kinematics.initial_pose_with_covariance.pose;
  const auto obj_arc = getArcCoordinates(current_lanes, obj_pose);

  // Get the object's center pose projected onto the lane centerline
  // First, get the centerline path
  const auto centerline_path = planner_data_->route_handler->getCenterLinePath(
    current_lanes, 0.0, std::numeric_limits<double>::max());

  if (centerline_path.points.empty()) {
    return candidates;
  }

  // Find the pose on centerline at object's longitudinal position
  const auto obj_centerline_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
    centerline_path.points, centerline_path.points.front().point.pose.position, obj_arc.length);

  if (!obj_centerline_pose) {
    return candidates;
  }

  // Generate 4 lateral offset candidates from object center: 1.0, 1.5, 2.0, 2.5 [m]
  const std::vector<double> lateral_offsets = {1.0, 1.5, 2.0, 2.5};

  for (const double lat_offset : lateral_offsets) {
    // Calculate total lateral offset from lane center
    // obj_arc.distance is object's lateral distance from center (positive = left, negative = right)
    // For right avoidance: goal should be further right than object
    // For left avoidance: goal should be further left than object
    double goal_lateral_from_center;
    if (avoid_right) {
      goal_lateral_from_center = obj_arc.distance - lat_offset;  // Move right (more negative)
    } else {
      goal_lateral_from_center = obj_arc.distance + lat_offset;  // Move left (more positive)
    }

    // Create goal pose by offsetting from centerline pose
    // calc_offset_pose: positive y = left in vehicle frame
    Pose end_pose = calc_offset_pose(*obj_centerline_pose, 0.0, goal_lateral_from_center, 0.0);

    candidates.push_back(end_pose);
  }

  return candidates;
}

std::optional<PullOutPath> DrySteeringAvoidanceModule::planAvoidancePath()
{
  // Clear debug data
  debug_end_pose_candidates_.clear();
  debug_target_object_.reset();

  const auto target_object = findNearestStaticObjectInCurrentLanes();
  if (!target_object) {
    return std::nullopt;
  }

  // Store for debug visualization
  debug_target_object_ = target_object;

  const auto end_pose_candidates = generateEndPoseCandidates(*target_object);
  if (end_pose_candidates.empty()) {
    RCLCPP_WARN(getLogger(), "[DrySteeringAvoidance] No end pose candidates generated");
    return std::nullopt;
  }

  // Store for debug visualization
  debug_end_pose_candidates_ = end_pose_candidates;

  const auto & start_pose = getEgoPose();
  const auto current_lanes = getCurrentLanes();
  if (current_lanes.empty()) {
    RCLCPP_WARN(getLogger(), "[DrySteeringAvoidance] current_lanes empty");
    return std::nullopt;
  }

  // Get road lanes for geometric planner
  const double backward_path_length = planner_data_->parameters.backward_path_length;
  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data_, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);

  // left_side_start: true if ego is on the left side of the target lane (avoiding to left)
  // For right avoidance: ego starts on left, moves right -> left_side_start = true
  // For left avoidance: ego starts on right, moves left -> left_side_start = false
  const bool avoid_right = (parameters_->avoidance_direction == "right");
  const bool left_side_start = avoid_right;

  // Set turning radius using vehicle info
  const double max_steer_angle =
    vehicle_info_.max_steer_angle_rad *
    parameters_->parallel_parking_parameters.geometric_pull_out_max_steer_angle_margin_scale;

  geometric_planner_.setTurningRadius(planner_data_->parameters, max_steer_angle);
  geometric_planner_.setPlannerData(planner_data_);

  // Try each end pose candidate
  size_t tried_count = 0;
  for (const auto & end_pose : end_pose_candidates) {
    tried_count++;

    // const bool found_valid_path = geometric_planner_.planPullOut(
    //   start_pose, end_pose, current_lanes, current_lanes, left_side_start,
    //   boundary_departure_checker_);

    // 本物のgoal poseを渡してみる
    const bool found_valid_path = geometric_planner_.planPullOut(
      start_pose, planner_data_->route_handler->getGoalPose(), current_lanes, current_lanes,
      left_side_start, boundary_departure_checker_);

    if (found_valid_path) {
      PullOutPath output;
      output.partial_paths = geometric_planner_.getPaths();
      output.pairs_terminal_velocity_and_accel =
        geometric_planner_.getPairsTerminalVelocityAndAccel();

      const auto arc_paths = geometric_planner_.getArcPaths();

      if (!arc_paths.empty() && !arc_paths.front().points.empty() && arc_paths.size() >= 2) {
        output.start_pose = arc_paths.front().points.front().point.pose;
        output.end_pose = arc_paths.back().points.back().point.pose;
      } else {
        continue;
      }

      return output;
    }

    // Limit log output for performance
    if (tried_count >= 10 && tried_count % 10 != 0) {
      continue;
    }
  }

  return std::nullopt;
}

PathWithLaneId DrySteeringAvoidanceModule::getCurrentPath() const
{
  if (!avoidance_path_.has_value() || avoidance_path_->partial_paths.empty()) {
    return getPreviousModuleOutput().path;
  }

  // Combine all partial paths
  PathWithLaneId combined_path;
  for (const auto & partial_path : avoidance_path_->partial_paths) {
    combined_path.points.insert(
      combined_path.points.end(), partial_path.points.begin(), partial_path.points.end());
  }

  // Remove overlap points
  combined_path.points = autoware::motion_utils::removeOverlapPoints(combined_path.points);

  return combined_path;
}

bool DrySteeringAvoidanceModule::hasReachedGoal() const
{
  if (!avoidance_path_.has_value()) {
    return false;
  }

  const double distance_to_goal = calc_distance2d(getEgoPose(), avoidance_path_->end_pose);
  constexpr double goal_threshold = 1.0;  // [m]

  return distance_to_goal < goal_threshold;
}

bool DrySteeringAvoidanceModule::hasObstacleDisappearedAtStart() const
{
  if (!avoidance_path_.has_value()) {
    return false;
  }

  // Check if ego is still at start pose and stopped
  const double distance_from_start = calc_distance2d(getEgoPose(), start_pose_);
  constexpr double start_threshold = 1.0;  // [m]

  if (distance_from_start > start_threshold || !isStopped()) {
    return false;
  }

  // Check if obstacle has disappeared
  return !hasStaticObjectInFront();
}

lanelet::ConstLanelets DrySteeringAvoidanceModule::getCurrentLanes() const
{
  return utils::getCurrentLanes(planner_data_);
}

lanelet::ConstLanelets DrySteeringAvoidanceModule::getRightLanes() const
{
  const auto current_lanes = getCurrentLanes();
  if (current_lanes.empty()) {
    return {};
  }

  const auto & route_handler = planner_data_->route_handler;
  lanelet::ConstLanelets right_lanes;

  // Get the rightmost lane using getMostRightLanelet
  for (const auto & lane : current_lanes) {
    // getMostRightLanelet: enable_same_root=true, get_shoulder_lane=true
    const auto rightmost_lane = route_handler->getMostRightLanelet(lane, true, true);
    // Add the rightmost lane if not already in the list
    if (std::find_if(right_lanes.begin(), right_lanes.end(), [&rightmost_lane](const auto & l) {
          return l.id() == rightmost_lane.id();
        }) == right_lanes.end()) {
      right_lanes.push_back(rightmost_lane);
    }
  }

  return right_lanes;
}

std::vector<DrivableLanes> DrySteeringAvoidanceModule::generateDrivableLanes() const
{
  const auto current_lanes = getCurrentLanes();
  if (current_lanes.empty()) {
    return {};
  }

  const auto & route_handler = planner_data_->route_handler;
  std::vector<DrivableLanes> drivable_lanes;

  for (const auto & lane : current_lanes) {
    DrivableLanes drivable_lane;

    // Left boundary: use current lane's left
    drivable_lane.left_lane = lane;

    // Right boundary: use getMostRightLanelet to get the rightmost lane
    // enable_same_root=true, get_shoulder_lane=true to include shoulder lanes
    const auto rightmost_lane = route_handler->getMostRightLanelet(lane, true, true);
    drivable_lane.right_lane = rightmost_lane;

    // Add middle lanes if there are any between left and right
    if (lane.id() != rightmost_lane.id()) {
      auto current_lane = lane;
      while (true) {
        const auto right_lane = route_handler->getRightLanelet(current_lane, true, true);
        if (!right_lane || right_lane->id() == rightmost_lane.id()) {
          break;
        }
        drivable_lane.middle_lanes.push_back(*right_lane);
        current_lane = *right_lane;
      }
    }

    drivable_lanes.push_back(drivable_lane);
  }

  return drivable_lanes;
}

double DrySteeringAvoidanceModule::calcLongitudinalDistanceFromEgo(
  const Pose & target_pose, const lanelet::ConstLanelets & current_lanes) const
{
  const auto ego_arc = getArcCoordinates(current_lanes, getEgoPose());
  const auto target_arc = getArcCoordinates(current_lanes, target_pose);

  return target_arc.length - ego_arc.length;
}

bool DrySteeringAvoidanceModule::isObjectInLanes(
  const PredictedObject & object, const lanelet::ConstLanelets & lanes) const
{
  const auto & obj_pose = object.kinematics.initial_pose_with_covariance.pose;

  for (const auto & lane : lanes) {
    if (lanelet::utils::isInLanelet(obj_pose, lane)) {
      return true;
    }
  }

  return false;
}

TurnSignalInfo DrySteeringAvoidanceModule::calcTurnSignalInfo() const
{
  TurnSignalInfo turn_signal_info;

  if (!avoidance_path_.has_value()) {
    return turn_signal_info;
  }

  const bool avoid_right = (parameters_->avoidance_direction == "right");

  // Set turn signal based on avoidance direction
  turn_signal_info.turn_signal.command =
    avoid_right ? autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT
                : autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT;

  turn_signal_info.desired_start_point = avoidance_path_->start_pose;
  turn_signal_info.desired_end_point = avoidance_path_->end_pose;
  turn_signal_info.required_start_point = avoidance_path_->start_pose;
  turn_signal_info.required_end_point = avoidance_path_->end_pose;

  return turn_signal_info;
}

void DrySteeringAvoidanceModule::setDebugData()
{
  using autoware_utils::append_marker_array;
  using marker_utils::createFootprintMarkerArray;
  using marker_utils::createPathMarkerArray;
  using marker_utils::createPoseMarkerArray;
  using visualization_msgs::msg::Marker;

  const auto life_time = rclcpp::Duration::from_seconds(1.5);
  auto add = [&life_time](
               visualization_msgs::msg::MarkerArray & added,
               visualization_msgs::msg::MarkerArray & target_marker_array) {
    for (auto & marker : added.markers) {
      marker.lifetime = life_time;
    }
    append_marker_array(added, &target_marker_array);
  };

  debug_marker_.markers.clear();
  info_marker_.markers.clear();

  // Visualize ego start pose
  auto start_pose_marker =
    createPoseMarkerArray(getEgoPose(), "dry_steering_start_pose", 0, 0.3, 0.9, 0.3);
  add(start_pose_marker, info_marker_);

  // Visualize ego footprint at start pose
  auto start_footprint_marker = createFootprintMarkerArray(
    getEgoPose(), vehicle_info_, "dry_steering_start_footprint", 0, 0.3, 0.9, 0.3);
  add(start_footprint_marker, debug_marker_);

  // Visualize end pose candidates
  if (!debug_end_pose_candidates_.empty()) {
    visualization_msgs::msg::MarkerArray end_pose_candidates_marker;
    for (size_t i = 0; i < debug_end_pose_candidates_.size(); ++i) {
      auto marker = autoware_utils::create_default_marker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "end_pose_candidates", static_cast<int>(i),
        Marker::ARROW, autoware_utils::create_marker_scale(0.5, 0.2, 0.2),
        autoware_utils::create_marker_color(0.0, 0.5, 1.0, 0.5));
      marker.pose = debug_end_pose_candidates_[i];
      marker.lifetime = life_time;
      end_pose_candidates_marker.markers.push_back(marker);
    }
    add(end_pose_candidates_marker, debug_marker_);
  }

  // Visualize target object
  if (debug_target_object_) {
    const auto & obj_pose = debug_target_object_->kinematics.initial_pose_with_covariance.pose;
    auto object_marker =
      createPoseMarkerArray(obj_pose, "dry_steering_target_object", 0, 1.0, 0.0, 0.0);
    add(object_marker, info_marker_);
  }

  // Visualize avoidance path end pose
  if (avoidance_path_.has_value()) {
    auto end_pose_marker =
      createPoseMarkerArray(avoidance_path_->end_pose, "dry_steering_end_pose", 0, 0.9, 0.9, 0.3);
    add(end_pose_marker, info_marker_);

    auto end_footprint_marker = createFootprintMarkerArray(
      avoidance_path_->end_pose, vehicle_info_, "dry_steering_end_footprint", 0, 0.9, 0.9, 0.3);
    add(end_footprint_marker, debug_marker_);

    // Visualize full path
    auto full_path_marker =
      createPathMarkerArray(getCurrentPath(), "dry_steering_full_path", 0, 0.0, 0.5, 0.9);
    add(full_path_marker, debug_marker_);
  }
}

}  // namespace autoware::behavior_path_planner
