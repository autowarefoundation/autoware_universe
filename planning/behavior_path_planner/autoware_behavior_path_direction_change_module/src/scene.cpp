// Copyright 2026 TIER IV, Inc.
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

#include "autoware/behavior_path_direction_change_module/scene.hpp"

#include "autoware/behavior_path_direction_change_module/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/logging.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
void logDirectionChangeDebugInfo(
  const rclcpp::Logger & logger, bool condition,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & current_reference_path,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & output_path,
  const geometry_msgs::msg::Pose & ego_pose)
{
  if (!condition) {
    return;
  }
  using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
  std::stringstream ss;

  auto print_path = [&ss](
                      const std::string & label,
                      const autoware_internal_planning_msgs::msg::PathWithLaneId & path) {
    ss << "[DirectionChange] " << label << " size=" << path.points.size() << "\n";
    for (size_t i = 0; i < path.points.size(); ++i) {
      const auto & pt = path.points[i].point;
      const double x = pt.pose.position.x;
      const double y = pt.pose.position.y;
      const double yaw = tf2::getYaw(pt.pose.orientation);
      const double yaw_deg = yaw * 180.0 / M_PI;
      const double v = pt.longitudinal_velocity_mps;
      ss << "  idx=" << i << ", x=" << x << ", y=" << y << ", yaw=" << yaw_deg << " deg"
         << ", v=" << v << " m/s";
      ss << ", lane_ids=[";
      for (size_t j = 0; j < path.points[i].lane_ids.size(); ++j) {
        if (j > 0) ss << " ";
        ss << path.points[i].lane_ids[j];
      }
      ss << "]\n";
    }
  };

  print_path("Current reference path (input)", current_reference_path);
  print_path("Output path (DirectionChange output)", output_path);

  ss << "[DirectionChange] Ego state:\n";
  const double ego_x = ego_pose.position.x;
  const double ego_y = ego_pose.position.y;
  const double ego_yaw = tf2::getYaw(ego_pose.orientation);
  const double ego_yaw_deg = ego_yaw * 180.0 / M_PI;
  ss << "  x=" << ego_x << ", y=" << ego_y << ", yaw=" << ego_yaw_deg << " deg\n";

  RCLCPP_INFO_STREAM(logger, ss.str());
}

std::string format_unique_path_lane_ids(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  std::set<int64_t> ids;
  for (const auto & pt : path.points) {
    for (const auto id : pt.lane_ids) {
      ids.insert(id);
    }
  }
  std::ostringstream ss;
  ss << "[";
  bool first = true;
  for (const auto id : ids) {
    if (!first) {
      ss << ", ";
    }
    ss << id;
    first = false;
  }
  ss << "]";
  return ss.str();
}

std::string format_lanelet_ids(const lanelet::ConstLanelets & lanelets)
{
  std::ostringstream ss;
  ss << "[";
  for (size_t i = 0; i < lanelets.size(); ++i) {
    if (i > 0) {
      ss << ", ";
    }
    ss << lanelets.at(i).id();
  }
  ss << "]";
  return ss.str();
}
}  // namespace

namespace autoware::behavior_path_planner
{
using autoware::motion_utils::calcSignedArcLength;
using autoware::motion_utils::findNearestIndex;
using autoware_utils::calc_distance2d;

DirectionChangeModule::DirectionChangeModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<DirectionChangeParameters> & parameters,
  const std::shared_ptr<DirectionChangePersistentState> & persistent_state,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> planning_factor_interface)
: SceneModuleInterface{name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map, planning_factor_interface},  // NOLINT
  parameters_{parameters},
  persistent_state_{persistent_state}
{
  path_publisher_ = node.create_publisher<autoware_internal_planning_msgs::msg::PathWithLaneId>(
    "~/output/direction_change/path", 1);
  RCLCPP_DEBUG(getLogger(), "Created path publisher: %s", path_publisher_->get_topic_name());
}

void DirectionChangeModule::initVariables()
{
  reference_path_ = PathWithLaneId();
  modified_path_ = PathWithLaneId();
  route_context_ = DirectionChangeRouteContext();
  is_route_context_initialized_ = false;
  cusp_points_.clear();
  current_segment_state_ = PathSegmentState::IDLE;
  is_ego_driving_forward_wrt_lane_ = true;
  cusp_stopped_since_.reset();
  // NOTE: from base class
  resetPathCandidate();
  resetPathReference();
}

bool DirectionChangeModule::buildRouteContextIfNeeded() const
{
  if (is_route_context_initialized_) {
    return route_context_.is_valid;
  }
  if (!planner_data_ || !planner_data_->route_handler) {
    return false;
  }

  const auto route_context = buildDirectionChangeRouteContext(planner_data_->route_handler);
  if (!route_context) {
    return false;
  }

  route_context_ = *route_context;
  is_route_context_initialized_ = true;

  RCLCPP_INFO_EXPRESSION(
    getLogger(), parameters_->print_debug_info,
    "Built direction_change route context: tagged_lanelets=%zu, tagged_centerline_points=%zu, "
    "prefix_lanelets=%zu, suffix_lanelets=%zu",
    route_context_.tagged_lanelet_ids_ordered.size(),
    route_context_.tagged_lanelet_centerline_path.points.size(),
    route_context_.prefix_lanelet_ids.size(), route_context_.suffix_lanelet_ids.size());

  return route_context_.is_valid;
}

void DirectionChangeModule::initializeCuspPointsFromTaggedCenterline()
{
  if (!route_context_.is_valid) {
    return;
  }

  cusp_points_ = detectCuspPointsOnPathWithIndices(
    route_context_.tagged_lanelet_centerline_path, parameters_->cusp_detection_angle_threshold_deg);

  if (parameters_->print_debug_info) {
    for (size_t i = 0; i < cusp_points_.size(); ++i) {
      const auto & cusp = cusp_points_.at(i);
      RCLCPP_INFO(
        getLogger(), "Tagged centerline cusp[%zu]: idx=%zu, x=%.2f, y=%.2f", i,
        cusp.tagged_centerline_index, cusp.pose.position.x, cusp.pose.position.y);
    }
  }
}

const PathWithLaneId & DirectionChangeModule::getTaggedLaneletCenterlinePath() const
{
  if (route_context_.is_valid) {
    return route_context_.tagged_lanelet_centerline_path;
  }
  return reference_path_;
}

void DirectionChangeModule::getCuspPointsFromReferencePath(
  const PathWithLaneId & reference_path, const geometry_msgs::msg::Pose & ego_pose)
{
  const auto detected =
    detectCuspPointsFromPath(reference_path, parameters_->cusp_detection_angle_threshold_deg);
  const size_t before = cusp_points_.size();
  mergeNewCuspPointsAheadOfEgo(
    cusp_points_, detected, reference_path, ego_pose,
    parameters_->cusp_detection_distance_threshold);

  if (parameters_->print_debug_info && cusp_points_.size() > before) {
    for (size_t i = before; i < cusp_points_.size(); ++i) {
      const auto & p = cusp_points_.at(i).pose.position;
      const double yaw_deg = tf2::getYaw(cusp_points_.at(i).pose.orientation) * 180.0 / M_PI;
      RCLCPP_INFO(
        getLogger(), "Tracked new cusp[%zu]: x=%.2f, y=%.2f, yaw=%.1f deg", i, p.x, p.y, yaw_deg);
    }
  }
}

void DirectionChangeModule::initializeManeuverState()
{
  const auto & maneuver_path = getTaggedLaneletCenterlinePath();
  if (!planner_data_ || !planner_data_->self_odometry || maneuver_path.points.empty()) {
    is_ego_driving_forward_wrt_lane_ = true;
    current_segment_state_ = PathSegmentState::IDLE;
    return;
  }

  const auto & ego_pose = planner_data_->self_odometry->pose.pose;
  is_ego_driving_forward_wrt_lane_ = isEgoDrivingForwardWrtLane(ego_pose, maneuver_path);

  current_segment_state_ = is_ego_driving_forward_wrt_lane_ ? PathSegmentState::FORWARD_FOLLOWING
                                                            : PathSegmentState::REVERSE_FOLLOWING;

  RCLCPP_INFO_EXPRESSION(
    getLogger(), parameters_->print_debug_info,
    "initializeManeuverState: forward_wrt_lane=%d, state=%s",
    static_cast<int>(is_ego_driving_forward_wrt_lane_),
    pathSegmentStateToString(current_segment_state_));
}

const CuspPoint * DirectionChangeModule::getFirstUnvisitedCusp() const
{
  for (const auto & cusp : cusp_points_) {
    if (!cusp.visited) {
      return &cusp;
    }
  }
  return nullptr;
}

const CuspPoint * DirectionChangeModule::getLastVisitedCusp() const
{
  const CuspPoint * last_visited = nullptr;
  for (const auto & cusp : cusp_points_) {
    if (cusp.visited) {
      last_visited = &cusp;
    }
  }
  return last_visited;
}

bool DirectionChangeModule::allCuspsVisited() const
{
  return cusp_points_.empty() || std::all_of(
                                   cusp_points_.begin(), cusp_points_.end(),
                                   [](const CuspPoint & cusp) { return cusp.visited; });
}

bool DirectionChangeModule::isManeuverCompletedForCurrentRoute() const
{
  if (
    !persistent_state_ || !persistent_state_->maneuver_completed || !planner_data_ ||
    !planner_data_->route_handler) {
    return false;
  }

  try {
    return planner_data_->route_handler->getRouteUuid() == persistent_state_->completed_route_uuid;
  } catch (...) {
    return false;
  }
}

void DirectionChangeModule::markManeuverCompletedForCurrentRoute()
{
  if (!persistent_state_ || !planner_data_ || !planner_data_->route_handler) {
    return;
  }

  try {
    persistent_state_->maneuver_completed = true;
    persistent_state_->completed_route_uuid = planner_data_->route_handler->getRouteUuid();
  } catch (...) {
  }
}

size_t DirectionChangeModule::countUnvisitedCusps() const
{
  return static_cast<size_t>(std::count_if(
    cusp_points_.begin(), cusp_points_.end(),
    [](const CuspPoint & cusp) { return !cusp.visited; }));
}

double DirectionChangeModule::calcDistanceToNextCusp(
  const PathWithLaneId & maneuver_path, const geometry_msgs::msg::Pose & ego_pose) const
{
  const auto * next_cusp = getFirstUnvisitedCusp();
  if (!next_cusp) {
    return calcDistanceToPathEnd(maneuver_path, ego_pose);
  }

  return calcDistanceAlongPathToPose(maneuver_path, ego_pose, next_cusp->pose);
}

void DirectionChangeModule::updateManeuverStateMachine(const PathWithLaneId & maneuver_path)
{
  const auto & ego_pose = planner_data_->self_odometry->pose.pose;
  const double dist_to_cusp = calcDistanceToNextCusp(maneuver_path, ego_pose);
  const double vehicle_velocity = std::abs(planner_data_->self_odometry->twist.twist.linear.x);

  if (allCuspsVisited()) {
    cusp_stopped_since_.reset();

    if (
      route_context_.is_valid && isDirectionChangeManeuverFinished(
                                   ego_pose, planner_data_->route_handler, route_context_,
                                   parameters_->th_arrived_distance, true)) {
      if (current_segment_state_ != PathSegmentState::COMPLETED) {
        current_segment_state_ = PathSegmentState::COMPLETED;
        RCLCPP_INFO(
          getLogger(),
          "Direction change maneuver finished (left tagged corridor or at goal), state=COMPLETED");
      }
      return;
    }

    if (current_segment_state_ == PathSegmentState::COMPLETED) {
      return;
    }

    if (current_segment_state_ != PathSegmentState::FORWARD_FOLLOWING) {
      is_ego_driving_forward_wrt_lane_ = true;
      current_segment_state_ = PathSegmentState::FORWARD_FOLLOWING;
    }
    return;
  }

  const PathSegmentState base_following_state = is_ego_driving_forward_wrt_lane_
                                                  ? PathSegmentState::FORWARD_FOLLOWING
                                                  : PathSegmentState::REVERSE_FOLLOWING;

  PathSegmentState new_state = current_segment_state_;

  if (dist_to_cusp > parameters_->cusp_detection_distance_threshold) {
    new_state = base_following_state;
    cusp_stopped_since_.reset();
    RCLCPP_DEBUG_EXPRESSION(
      getLogger(), parameters_->print_debug_info,
      "updateManeuverStateMachine: far from cusp, dist=%.2f, threshold=%.2f", dist_to_cusp,
      parameters_->cusp_detection_distance_threshold);
  } else {
    new_state = PathSegmentState::AT_CUSP;

    if (std::abs(vehicle_velocity) < parameters_->stop_velocity_threshold) {
      if (!cusp_stopped_since_.has_value()) {
        cusp_stopped_since_ = clock_->now();
      }
      const double stopped_duration = (clock_->now() - cusp_stopped_since_.value()).seconds();
      if (stopped_duration >= parameters_->th_stopped_time) {
        cusp_stopped_since_.reset();
        is_ego_driving_forward_wrt_lane_ = !is_ego_driving_forward_wrt_lane_;
        for (auto & cusp : cusp_points_) {
          if (cusp.visited) {
            continue;
          }
          const auto & passed = cusp.pose.position;
          RCLCPP_INFO_EXPRESSION(
            getLogger(), parameters_->print_debug_info, "Passed cusp at (%.2f, %.2f) after stop",
            passed.x, passed.y);
          cusp.visited = true;
          break;
        }
        new_state = is_ego_driving_forward_wrt_lane_ ? PathSegmentState::FORWARD_FOLLOWING
                                                     : PathSegmentState::REVERSE_FOLLOWING;
      }
    } else if (cusp_stopped_since_.has_value()) {
      cusp_stopped_since_.reset();
    }
  }

  if (new_state != current_segment_state_) {
    RCLCPP_INFO(
      getLogger(),
      "Maneuver state: %s -> %s, dist_to_cusp=%.2f, v=%.2f, forward_wrt_lane=%d, cusps_left=%zu",
      pathSegmentStateToString(current_segment_state_), pathSegmentStateToString(new_state),
      dist_to_cusp, vehicle_velocity, static_cast<int>(is_ego_driving_forward_wrt_lane_),
      countUnvisitedCusps());
    if (new_state == PathSegmentState::AT_CUSP) {
      cusp_stopped_since_.reset();
    }
    current_segment_state_ = new_state;
  }
}

void DirectionChangeModule::processOnEntry()
{
  RCLCPP_DEBUG(getLogger(), "Module entry - initializing variables");
  initVariables();
  buildRouteContextIfNeeded();
  initializeCuspPointsFromTaggedCenterline();
  updateData();
  initializeManeuverState();
}

void DirectionChangeModule::processOnExit()
{
  RCLCPP_DEBUG(getLogger(), "Module exit - resetting variables");
  initVariables();
}

void DirectionChangeModule::setParameters(
  const std::shared_ptr<DirectionChangeParameters> & parameters)
{
  parameters_ = parameters;
}

bool DirectionChangeModule::isExecutionRequested() const
{
  if (getCurrentStatus() == ModuleStatus::RUNNING) {
    return true;
  }

  buildRouteContextIfNeeded();
  return shouldActivateModule();
}

bool DirectionChangeModule::isExecutionReady() const
{
  return true;
}

bool DirectionChangeModule::isReadyForNextRequest(
  const double & min_request_time_sec, bool override_requests) const noexcept
{
  (void)min_request_time_sec;
  (void)override_requests;
  return true;
}

void DirectionChangeModule::updateData()
{
  if (
    persistent_state_ && persistent_state_->maneuver_completed && planner_data_ &&
    planner_data_->route_handler) {
    try {
      if (planner_data_->route_handler->getRouteUuid() != persistent_state_->completed_route_uuid) {
        persistent_state_->maneuver_completed = false;
      }
    } catch (...) {
    }
  }

  const auto previous_output = getPreviousModuleOutput();
  if (previous_output.path.points.empty()) {
    RCLCPP_WARN(getLogger(), "Previous module output path is empty. Cannot update data.");
    return;
  }

  if (!buildRouteContextIfNeeded()) {
    reference_path_ = previous_output.path;
    return;
  }

  if (!planner_data_->self_odometry) {
    reference_path_ = route_context_.tagged_lanelet_centerline_path;
    return;
  }

  const auto & ego_pose = planner_data_->self_odometry->pose.pose;
  const auto assembly_phase = determineReferencePathAssemblyPhase(
    planner_data_->route_handler, ego_pose, route_context_, allCuspsVisited());

  reference_path_ = assembleReferencePathWithLaneStitching(
    route_context_, previous_output.path, planner_data_->route_handler, assembly_phase);

  RCLCPP_INFO_EXPRESSION(
    getLogger(), parameters_->print_debug_info,
    "Assembled reference path: phase=%s, points=%zu, tagged_centerline_points=%zu",
    referencePathAssemblyPhaseToString(assembly_phase), reference_path_.points.size(),
    route_context_.tagged_lanelet_centerline_path.points.size());
}

bool DirectionChangeModule::shouldActivateModule() const
{
  if (!planner_data_ || !planner_data_->route_handler || !planner_data_->self_odometry) {
    return false;
  }

  if (isManeuverCompletedForCurrentRoute()) {
    RCLCPP_DEBUG_EXPRESSION(
      getLogger(), parameters_->print_debug_info,
      "shouldActivateModule: maneuver already completed for this route, module INACTIVE");
    return false;
  }

  if (!route_context_.is_valid || route_context_.tagged_lanelet_ids_ordered.empty()) {
    RCLCPP_DEBUG_EXPRESSION(
      getLogger(), parameters_->print_debug_info,
      "shouldActivateModule: route has no direction_change tagged lanelets, module INACTIVE");
    return false;
  }

  const auto & ego_pose = planner_data_->self_odometry->pose.pose;

  if (isEgoNearRouteGoal(
        ego_pose, planner_data_->route_handler, parameters_->th_arrived_distance,
        route_context_.suffix_lanelet_ids)) {
    RCLCPP_DEBUG_EXPRESSION(
      getLogger(), parameters_->print_debug_info,
      "shouldActivateModule: ego at route goal, module INACTIVE");
    return false;
  }

  const bool on_prefix = isEgoOnPrefixLanelets(
    ego_pose, planner_data_->route_handler, route_context_.prefix_lanelet_ids);
  const bool on_tagged = isEgoOnTaggedLanelets(
    ego_pose, planner_data_->route_handler, route_context_.tagged_lanelet_ids_ordered);

  if (!on_prefix && !on_tagged) {
    RCLCPP_DEBUG_EXPRESSION(
      getLogger(), parameters_->print_debug_info,
      "shouldActivateModule: ego not on prefix/tagged lanes, module INACTIVE");
    return false;
  }

  RCLCPP_DEBUG_EXPRESSION(
    getLogger(), parameters_->print_debug_info,
    "shouldActivateModule: route has tagged lanelets and ego on %s, module ACTIVE",
    on_tagged ? "tagged lane" : "prefix lane");
  return true;
}

void DirectionChangeModule::filterLaneletsAtCusp(BehaviorModuleOutput & output)
{
  if (output.path.points.size() < 2) {
    return;
  }

  const size_t end_idx = output.path.points.size() - 1;
  // Cusp points include lane_ids from the next segment; restrict to the current segment
  // lanelets so drivable area bounds are not expanded across next direction-change lanes.
  output.path.points.at(end_idx).lane_ids = output.path.points.at(end_idx - 1).lane_ids;
}

void DirectionChangeModule::updateDrivableAreaInfo(BehaviorModuleOutput & output)
{
  // TODO: This check might not be necessary, but keep it for reference.
  //    Clean once confirmation of results.
  const bool is_active_segment = current_segment_state_ == PathSegmentState::FORWARD_FOLLOWING ||
                                 current_segment_state_ == PathSegmentState::AT_CUSP ||
                                 current_segment_state_ == PathSegmentState::REVERSE_FOLLOWING;

  const auto prev_drivable_info = getPreviousModuleOutput().drivable_area_info;

  if (is_active_segment && !output.path.points.empty()) {
    const auto lanelets = utils::getLaneletsFromPath(output.path, planner_data_->route_handler);
    RCLCPP_DEBUG_EXPRESSION(
      getLogger(), parameters_->print_debug_info,
      "updateDrivableAreaInfo: path_lane_ids=%s, lanelet_ids=%s",
      format_unique_path_lane_ids(output.path).c_str(), format_lanelet_ids(lanelets).c_str());
    output.drivable_area_info.drivable_lanes = utils::generateDrivableLanes(lanelets);
    return;
  }

  output.drivable_area_info = prev_drivable_info;
}

void DirectionChangeModule::updateTurnSignalInfo(BehaviorModuleOutput & output)
{
  output.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
}

BehaviorModuleOutput DirectionChangeModule::plan()
{
  BehaviorModuleOutput output;

  buildRouteContextIfNeeded();

  const auto current_reference_path = reference_path_;
  const auto & tagged_centerline_path = getTaggedLaneletCenterlinePath();
  if (tagged_centerline_path.points.empty()) {
    RCLCPP_WARN(getLogger(), "Tagged lanelet centerline is empty in plan()");
    return output;
  }

  const auto & ego_pose = planner_data_->self_odometry->pose.pose;

  updateManeuverStateMachine(tagged_centerline_path);

  RCLCPP_INFO_EXPRESSION(
    getLogger(), parameters_->print_debug_info,
    "plan(): reference_points=%zu, tagged_centerline_points=%zu, unvisited_cusps=%zu, state=%s, "
    "forward_wrt_lane=%d",
    current_reference_path.points.size(), tagged_centerline_path.points.size(),
    countUnvisitedCusps(), pathSegmentStateToString(current_segment_state_),
    static_cast<int>(is_ego_driving_forward_wrt_lane_));

  const double dist_to_cusp = calcDistanceToNextCusp(tagged_centerline_path, ego_pose);
  const bool approaching_cusp =
    getFirstUnvisitedCusp() != nullptr &&
    dist_to_cusp <= parameters_->cusp_detection_distance_start_approaching;

  const auto appendSuffixLanesIfNeeded = [&](PathWithLaneId path_segment) {
    if (!route_context_.is_valid || route_context_.suffix_lanelet_ids.empty()) {
      return path_segment;
    }
    auto suffix_path = extractPathPointsForLaneIds(
      getPreviousModuleOutput().path, route_context_.suffix_lanelet_ids);
    if (suffix_path.points.empty()) {
      suffix_path = buildCenterlinePathForLaneIds(
        route_context_.suffix_lanelet_ids, planner_data_->route_handler);
    }
    if (suffix_path.points.empty()) {
      return path_segment;
    }
    return utils::combinePath(path_segment, suffix_path);
  };

  const auto assembly_phase = determineReferencePathAssemblyPhase(
    planner_data_->route_handler, ego_pose, route_context_, allCuspsVisited());

  const auto prependPrefixLanesIfApproaching = [&](PathWithLaneId path_segment) {
    if (
      assembly_phase != ReferencePathAssemblyPhase::APPROACHING_TAGGED_AREA ||
      route_context_.prefix_lanelet_ids.empty()) {
      return path_segment;
    }

    auto prefix_path = buildPrefixPathForStitching(
      route_context_, getPreviousModuleOutput().path, planner_data_->route_handler);
    if (prefix_path.points.empty()) {
      return path_segment;
    }

    auto combined_path = utils::combinePath(prefix_path, path_segment);
    return cropPathFromEgo(combined_path, ego_pose);
  };

  if (allCuspsVisited()) {
    PathWithLaneId final_segment = tagged_centerline_path;
    const auto * last_visited = getLastVisitedCusp();
    std::optional<size_t> start_after_cusp_index;
    if (last_visited) {
      start_after_cusp_index = last_visited->tagged_centerline_index;
    }

    if (planner_data_->route_handler) {
      try {
        const auto goal_pose = planner_data_->route_handler->getGoalPose();
        final_segment =
          slicePathToGoalFromCuspIndex(tagged_centerline_path, start_after_cusp_index, goal_pose);

        if (!is_ego_driving_forward_wrt_lane_ && parameters_->enable_goal_lateral_shift) {
          if (
            const auto shifted = applyGoalLateralShift(
              final_segment, goal_pose, *parameters_, planner_data_->route_handler)) {
            final_segment = *shifted;
          }
        }
      } catch (...) {
      }
    }

    final_segment = appendSuffixLanesIfNeeded(final_segment);
    final_segment = cropPathFromEgo(final_segment, ego_pose);
    output.path = final_segment;
    if (!is_ego_driving_forward_wrt_lane_) {
      flipPathPointOrientation(output.path);
    }
  } else {
    const auto * next_cusp = getFirstUnvisitedCusp();
    const auto * last_visited = getLastVisitedCusp();
    std::optional<size_t> start_after_cusp_index;
    if (last_visited) {
      start_after_cusp_index = last_visited->tagged_centerline_index;
    }

    output.path = slicePathBetweenCuspIndices(
      tagged_centerline_path, start_after_cusp_index, next_cusp->tagged_centerline_index);

    output.path = prependPrefixLanesIfApproaching(output.path);

    if (output.path.points.empty()) {
      RCLCPP_WARN(
        getLogger(),
        "slicePathBetweenCuspIndices returned empty path; falling back to tagged centerline tail");
      const auto ego_idx_opt = findNearestIndex(tagged_centerline_path.points, ego_pose);
      const size_t ego_idx = ego_idx_opt ? *ego_idx_opt : 0;
      const size_t end_idx =
        std::min(next_cusp->tagged_centerline_index + 1, tagged_centerline_path.points.size());
      const size_t start_idx = std::min(ego_idx, end_idx > 0 ? end_idx - 1 : 0);
      output.path.header = tagged_centerline_path.header;
      if (end_idx > start_idx) {
        output.path.points.assign(
          tagged_centerline_path.points.begin() + static_cast<std::ptrdiff_t>(start_idx),
          tagged_centerline_path.points.begin() + static_cast<std::ptrdiff_t>(end_idx));
      }
    }

    if (!is_ego_driving_forward_wrt_lane_) {
      flipPathPointOrientation(output.path);
    }

    if (approaching_cusp || current_segment_state_ == PathSegmentState::AT_CUSP) {
      setPathPointVelocityToZero(output.path, 1);
    }
  }

  modified_path_ = output.path;

  // Publish processed path for debugging
  if (path_publisher_ && !output.path.points.empty()) {
    autoware_internal_planning_msgs::msg::PathWithLaneId path_msg;
    path_msg.header.stamp = clock_->now();

    if (planner_data_ && planner_data_->route_handler) {
      path_msg.header.frame_id = planner_data_->route_handler->getRouteHeader().frame_id;
    } else {
      path_msg.header.frame_id = "map";
    }

    path_msg.points.reserve(output.path.points.size());
    for (const auto & point : output.path.points) {
      autoware_internal_planning_msgs::msg::PathPointWithLaneId path_point;
      path_point.point = point.point;
      path_point.lane_ids = point.lane_ids;
      path_msg.points.push_back(path_point);
    }

    path_publisher_->publish(path_msg);

    RCLCPP_DEBUG_EXPRESSION(
      getLogger(), parameters_->print_debug_info, "Published path to %s with %zu points, state=%s",
      path_publisher_->get_topic_name(), path_msg.points.size(),
      pathSegmentStateToString(current_segment_state_));
  }

  updateTurnSignalInfo(output);
  if (getFirstUnvisitedCusp() != nullptr) {
    filterLaneletsAtCusp(output);
  }
  updateDrivableAreaInfo(output);

  logDirectionChangeDebugInfo(
    getLogger(), parameters_->print_debug_info, reference_path_, output.path, ego_pose);

  return output;
}

BehaviorModuleOutput DirectionChangeModule::planWaitingApproval()
{
  return plan();
}

CandidateOutput DirectionChangeModule::planCandidate() const
{
  CandidateOutput output;
  output.path_candidate = reference_path_;
  return output;
}

bool DirectionChangeModule::canTransitSuccessState()
{
  if (!allCuspsVisited() || cusp_points_.empty()) {
    return false;
  }

  if (!planner_data_ || !planner_data_->self_odometry || !route_context_.is_valid) {
    return false;
  }

  const auto & ego_pose = planner_data_->self_odometry->pose.pose;

  if (!isDirectionChangeManeuverFinished(
        ego_pose, planner_data_->route_handler, route_context_, parameters_->th_arrived_distance,
        true)) {
    return false;
  }

  markManeuverCompletedForCurrentRoute();
  current_segment_state_ = PathSegmentState::COMPLETED;
  RCLCPP_INFO(
    getLogger(),
    "Direction change maneuver finished (left tagged corridor or at goal), transiting to SUCCESS");
  return true;
}

}  // namespace autoware::behavior_path_planner
