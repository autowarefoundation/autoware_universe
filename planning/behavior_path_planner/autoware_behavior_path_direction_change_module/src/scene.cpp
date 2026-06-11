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

std::string format_drivable_lanes_ids(
  const std::vector<autoware::behavior_path_planner::DrivableLanes> & drivable_lanes)
{
  std::ostringstream ss;
  ss << "[";
  for (size_t i = 0; i < drivable_lanes.size(); ++i) {
    if (i > 0) {
      ss << "; ";
    }
    const auto & dl = drivable_lanes.at(i);
    ss << "{right=" << dl.right_lane.id() << ", left=" << dl.left_lane.id() << ", middle=[";
    for (size_t j = 0; j < dl.middle_lanes.size(); ++j) {
      if (j > 0) {
        ss << ", ";
      }
      ss << dl.middle_lanes.at(j).id();
    }
    ss << "]}";
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
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> planning_factor_interface)
: SceneModuleInterface{name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map, planning_factor_interface},  // NOLINT
  parameters_{parameters}
{
  path_publisher_ = node.create_publisher<autoware_internal_planning_msgs::msg::PathWithLaneId>(
    "~/output/direction_change/path", 1);
  RCLCPP_DEBUG(getLogger(), "Created path publisher: %s", path_publisher_->get_topic_name());
}

void DirectionChangeModule::initVariables()
{
  reference_path_ = PathWithLaneId();
  modified_path_ = PathWithLaneId();
  cusp_points_.clear();
  current_segment_state_ = PathSegmentState::IDLE;
  is_ego_driving_forward_wrt_lane_ = true;
  cusp_stopped_since_.reset();
  // NOTE: from base class
  resetPathCandidate();
  resetPathReference();
}

void DirectionChangeModule::getCuspPointsFromReferencePath(
  const PathWithLaneId & reference_path, const geometry_msgs::msg::Pose & ego_pose)
{
  const auto detected = detectCuspPointsFromPath(
    reference_path, parameters_->cusp_detection_angle_threshold_deg);
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
  if (!planner_data_ || !planner_data_->self_odometry || reference_path_.points.empty()) {
    is_ego_driving_forward_wrt_lane_ = true;
    current_segment_state_ = PathSegmentState::IDLE;
    return;
  }

  const auto & ego_pose = planner_data_->self_odometry->pose.pose;
  is_ego_driving_forward_wrt_lane_ = isEgoDrivingForwardWrtLane(ego_pose, reference_path_);

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
  return cusp_points_.empty() ||
         std::all_of(cusp_points_.begin(), cusp_points_.end(), [](const CuspPoint & cusp) {
           return cusp.visited;
         });
}

size_t DirectionChangeModule::countUnvisitedCusps() const
{
  return static_cast<size_t>(std::count_if(
    cusp_points_.begin(), cusp_points_.end(), [](const CuspPoint & cusp) { return !cusp.visited; }));
}

double DirectionChangeModule::calcDistanceToNextCusp(
  const PathWithLaneId & reference_path, const geometry_msgs::msg::Pose & ego_pose) const
{
  const auto * next_cusp = getFirstUnvisitedCusp();
  if (!next_cusp) {
    return calcDistanceToPathEnd(reference_path, ego_pose);
  }

  return calcDistanceAlongPathToPose(reference_path, ego_pose, next_cusp->pose);
}

PathWithLaneId DirectionChangeModule::slicePathBetweenCusps(
  const PathWithLaneId & source_path, const geometry_msgs::msg::Pose & ego_pose,
  const std::optional<geometry_msgs::msg::Pose> & start_cusp_pose,
  const geometry_msgs::msg::Pose & end_cusp_pose) const
{
  PathWithLaneId sliced;
  if (source_path.points.empty()) {
    return sliced;
  }

  const auto ego_idx_opt = findNearestIndex(source_path.points, ego_pose);
  const size_t ego_idx = ego_idx_opt ? *ego_idx_opt : 0;

  const size_t end_cusp_idx =
    autoware::motion_utils::findNearestIndex(source_path.points, end_cusp_pose.position);

  std::optional<size_t> start_cusp_idx;
  if (start_cusp_pose) {
    start_cusp_idx = autoware::motion_utils::findNearestIndex(
      source_path.points, start_cusp_pose->position);
  }

  size_t start_idx = start_cusp_idx.value_or(0);
  if (start_cusp_idx) {
    const size_t after_cusp_idx = *start_cusp_idx + 1;
    if (after_cusp_idx < end_cusp_idx) {
      start_idx = after_cusp_idx;
    }
  }
  const size_t end_idx = end_cusp_idx;
  const bool reset_start_to_zero = start_idx >= end_idx;
  if (reset_start_to_zero) {
    start_idx = 0;
  }

  std::cout << "[Debug2 DirectionChangeModule] slicePathBetweenCusps:"
            << " ego_idx=" << ego_idx
            << " start_cusp_idx=" << (start_cusp_idx ? std::to_string(*start_cusp_idx) : "none")
            << " end_cusp_idx=" << end_cusp_idx << " start_idx=" << start_idx
            << " end_idx=" << end_idx << " reset_start_to_zero=" << reset_start_to_zero
            << " output_size=" << (end_idx > start_idx ? end_idx - start_idx : 0) << std::endl;

  if (parameters_->print_debug_info) {
    if (start_cusp_idx) {
      RCLCPP_INFO(
        getLogger(),
        "slicePathBetweenCusps: ego_idx=%zu, start_cusp_idx=%zu, end_cusp_idx=%zu, start_idx=%zu, "
        "end_idx=%zu, reset_start_to_zero=%d, output_size=%zu",
        ego_idx, *start_cusp_idx, end_cusp_idx, start_idx, end_idx,
        static_cast<int>(reset_start_to_zero), end_idx > start_idx ? end_idx - start_idx : 0);
    } else {
      RCLCPP_INFO(
        getLogger(),
        "slicePathBetweenCusps: ego_idx=%zu, start_cusp_idx=none, end_cusp_idx=%zu, start_idx=%zu, "
        "end_idx=%zu, reset_start_to_zero=%d, output_size=%zu",
        ego_idx, end_cusp_idx, start_idx, end_idx, static_cast<int>(reset_start_to_zero),
        end_idx > start_idx ? end_idx - start_idx : 0);
    }
  }

  sliced.header = source_path.header;
  sliced.points.assign(
    source_path.points.begin() + static_cast<std::ptrdiff_t>(start_idx),
    source_path.points.begin() + static_cast<std::ptrdiff_t>(end_idx));  // end cusp is not included
  return sliced;
}

PathWithLaneId DirectionChangeModule::slicePathToGoal(
  const PathWithLaneId & source_path, const geometry_msgs::msg::Pose & ego_pose,
  const std::optional<geometry_msgs::msg::Pose> & start_cusp_pose,
  const geometry_msgs::msg::Pose & goal_pose) const
{
  PathWithLaneId sliced;
  if (source_path.points.empty()) {
    return sliced;
  }

  const auto ego_idx_opt = findNearestIndex(source_path.points, ego_pose);
  const size_t ego_idx = ego_idx_opt ? *ego_idx_opt : 0;

  const size_t goal_idx =
    autoware::motion_utils::findNearestIndex(source_path.points, goal_pose.position);

  std::optional<size_t> start_cusp_idx;
  if (start_cusp_pose) {
    start_cusp_idx = autoware::motion_utils::findNearestIndex(
      source_path.points, start_cusp_pose->position);
  }

  const size_t end_idx = goal_idx + 1;
  size_t start_idx = start_cusp_idx.value_or(0);
  if (start_cusp_idx) {
    const size_t after_cusp_idx = *start_cusp_idx + 1;
    if (after_cusp_idx < end_idx) {
      start_idx = after_cusp_idx;
    }
  }
  const bool reset_start_to_zero = start_idx >= end_idx;
  if (reset_start_to_zero) {
    start_idx = 0;
  }

  std::cout << "[Debug2 DirectionChangeModule] slicePathToGoal:"
            << " ego_idx=" << ego_idx
            << " start_cusp_idx=" << (start_cusp_idx ? std::to_string(*start_cusp_idx) : "none")
            << " goal_idx=" << goal_idx << " start_idx=" << start_idx << " end_idx=" << end_idx
            << " reset_start_to_zero=" << reset_start_to_zero
            << " output_size=" << (end_idx > start_idx ? end_idx - start_idx : 0) << std::endl;

  sliced.header = source_path.header;
  sliced.points.assign(
    source_path.points.begin() + static_cast<std::ptrdiff_t>(start_idx),
    source_path.points.begin() + static_cast<std::ptrdiff_t>(end_idx));  // goal point is included
  return sliced;
}

void DirectionChangeModule::updateManeuverStateMachine(const PathWithLaneId & reference_path)
{
  const auto & ego_pose = planner_data_->self_odometry->pose.pose;
  const double dist_to_cusp = calcDistanceToNextCusp(reference_path, ego_pose);
  const double vehicle_velocity =
    std::abs(planner_data_->self_odometry->twist.twist.linear.x);

  const PathSegmentState base_following_state = is_ego_driving_forward_wrt_lane_
                                                  ? PathSegmentState::FORWARD_FOLLOWING
                                                  : PathSegmentState::REVERSE_FOLLOWING;

  PathSegmentState new_state = current_segment_state_;

  if (dist_to_cusp > parameters_->cusp_detection_distance_threshold) {
    new_state = base_following_state;
    cusp_stopped_since_.reset();
    std::cout << "[Debug2 DirectionChangeModule] updateManeuverStateMachine: cusp_detection_distance_threshold: " << parameters_->cusp_detection_distance_threshold << std::endl;
    std::cout << "[Debug2 DirectionChangeModule] updateManeuverStateMachine: distance to cusp: " << dist_to_cusp << std::endl;
    std::cout << "[Debug2 DirectionChangeModule] updateManeuverStateMachine: far From Cusp" << std::endl;
  } else {
    new_state = PathSegmentState::AT_CUSP;

    if (std::abs(vehicle_velocity) < parameters_->stop_velocity_threshold) {
      if (!cusp_stopped_since_.has_value()) {
        cusp_stopped_since_ = clock_->now();
      }
      const double stopped_duration =
        (clock_->now() - cusp_stopped_since_.value()).seconds();
      if (stopped_duration >= parameters_->th_stopped_time) {
        cusp_stopped_since_.reset();
        is_ego_driving_forward_wrt_lane_ = !is_ego_driving_forward_wrt_lane_;
        for (auto & cusp : cusp_points_) {
          if (cusp.visited) {
            continue;
          }
          const auto & passed = cusp.pose.position;
          RCLCPP_INFO_EXPRESSION(
            getLogger(), parameters_->print_debug_info,
            "Passed cusp at (%.2f, %.2f) after stop", passed.x, passed.y);
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
  const auto previous_output = getPreviousModuleOutput();
  if (previous_output.path.points.empty()) {
    RCLCPP_WARN(getLogger(), "Previous module output path is empty. Cannot update data.");
    return;
  }

  if (planner_data_ && planner_data_->route_handler) {
    const auto centerline_path = getReferencePathFromDirectionChangeLanelets(
      previous_output.path, planner_data_->route_handler, *parameters_);
    if (!centerline_path.points.empty()) {
      reference_path_ = centerline_path;
      RCLCPP_INFO_EXPRESSION(
        getLogger(), parameters_->print_debug_info,
        "Using centerline from direction_change lanelets (%zu points)",
        reference_path_.points.size());
      return;
    }
  }
  // Fallback: use previous module output when not in direction_change area, or when
  // centerline from lanelets is unavailable (no route_handler, no tagged lane_ids, or empty path).
  reference_path_ = previous_output.path;
}

bool DirectionChangeModule::shouldActivateModule() const
{
  if (reference_path_.points.empty()) {
    RCLCPP_DEBUG_EXPRESSION(
      getLogger(), parameters_->print_debug_info,
      "shouldActivateModule: Path empty, module inactive");
    return false;
  }

  if (planner_data_->route_handler) {
    std::set<int64_t> checked_lane_ids;
    for (const auto & path_point : reference_path_.points) {
      for (const auto & lane_id : path_point.lane_ids) {
        if (checked_lane_ids.find(lane_id) != checked_lane_ids.end()) {
          continue;
        }
        checked_lane_ids.insert(lane_id);

        try {
          const auto lanelet = planner_data_->route_handler->getLaneletsFromId(lane_id);
          if (hasDirectionChangeAreaTag(lanelet)) {
            // Activate only when in tag area and away from goal (avoid re-entry after completion).
            // Goal planner and start planner also follow similar checks for goal arrival
            if (planner_data_->self_odometry) {
              try {
                const auto goal_pose = planner_data_->route_handler->getGoalPose();
                const double dist_to_goal = autoware_utils::calc_distance2d(
                  planner_data_->self_odometry->pose.pose.position, goal_pose.position);
                if (dist_to_goal < parameters_->th_arrived_distance) {
                  RCLCPP_WARN(
                    getLogger(), "shouldActivateModule: at goal (dist=%.2f m), module INACTIVE",
                    dist_to_goal);
                  return false;
                }
              } catch (...) {
                // No goal or getGoalPose failed; allow activation
              }
            }
            RCLCPP_DEBUG_EXPRESSION(
              getLogger(), parameters_->print_debug_info,
              "shouldActivateModule: direction_change_lane tag found in lane_id=%ld, module ACTIVE",
              static_cast<int64_t>(lane_id));
            return true;  // Tag found and away from goal, activate module
          }
        } catch (...) {
          // Lanelet not found, skip
          continue;
        }
      }
    }
  }

  // No direction_change_lane tag found, module inactive
  RCLCPP_DEBUG_EXPRESSION(
    getLogger(), parameters_->print_debug_info,
    "shouldActivateModule: No direction_change_lane tag found, module INACTIVE");
  return false;
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
  const bool is_active_segment =
    current_segment_state_ == PathSegmentState::FORWARD_FOLLOWING ||
    current_segment_state_ == PathSegmentState::AT_CUSP ||
    current_segment_state_ == PathSegmentState::REVERSE_FOLLOWING;

  const auto prev_drivable_info = getPreviousModuleOutput().drivable_area_info;

  if (is_active_segment && !output.path.points.empty()) {
    const auto lanelets =
      utils::getLaneletsFromPath(output.path, planner_data_->route_handler);
    std::cout << "[Debug2 DirectionChangeModule] updateDrivableAreaInfo: path_lane_ids="
              << format_unique_path_lane_ids(output.path) << std::endl;
    std::cout << "[Debug2 DirectionChangeModule] updateDrivableAreaInfo: lanelets.size()="
              << lanelets.size() << " lanelet_ids=" << format_lanelet_ids(lanelets) << std::endl;
    output.drivable_area_info.drivable_lanes = utils::generateDrivableLanes(lanelets);
    std::cout << "[Debug2 DirectionChangeModule] updateDrivableAreaInfo: drivable_lanes.size()="
              << output.drivable_area_info.drivable_lanes.size()
              << " drivable_lane_ids=" << format_drivable_lanes_ids(output.drivable_area_info.drivable_lanes)
              << std::endl;
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
  // Note: updateData() is already called by SceneModuleInterface::run() before plan()
  BehaviorModuleOutput output;
  //  output.reference_path = reference_path_;

  // Copy reference_path_ to local variable for stability
  const auto current_reference_path = reference_path_;
  if (current_reference_path.points.empty()) {
    RCLCPP_WARN(getLogger(), "Reference path is empty in plan()");
    return output;
  }

  const auto & ego_pose = planner_data_->self_odometry->pose.pose;

  // Update state machine before cusp detection so passed cusps are recorded first
  updateManeuverStateMachine(current_reference_path);

  getCuspPointsFromReferencePath(current_reference_path, ego_pose);

  RCLCPP_INFO_EXPRESSION(
    getLogger(), parameters_->print_debug_info,
    "plan(): path_points=%zu, unvisited_cusps=%zu, state=%s, forward_wrt_lane=%d",
    current_reference_path.points.size(), countUnvisitedCusps(),
    pathSegmentStateToString(current_segment_state_),
    static_cast<int>(is_ego_driving_forward_wrt_lane_));

  const double dist_to_cusp = calcDistanceToNextCusp(current_reference_path, ego_pose);
  const bool approaching_cusp =
    getFirstUnvisitedCusp() != nullptr &&
    dist_to_cusp <= parameters_->cusp_detection_distance_start_approaching;

  std::cout << "[Debug2 DirectionChangeModule] plan: unvisited_cusps: " << countUnvisitedCusps()
            << std::endl;

  if (allCuspsVisited()) {
    // Final segment from last visited cusp (or path start) to goal on reference path
    PathWithLaneId final_segment = current_reference_path;
    const auto * last_visited = getLastVisitedCusp();
    std::optional<geometry_msgs::msg::Pose> start_cusp_pose;
    if (last_visited) {
      start_cusp_pose = last_visited->pose;
    }

    if (planner_data_->route_handler) {
      try {
        const auto goal_pose = planner_data_->route_handler->getGoalPose();
        final_segment =
          slicePathToGoal(current_reference_path, ego_pose, start_cusp_pose, goal_pose);

        if (!is_ego_driving_forward_wrt_lane_ && parameters_->enable_goal_lateral_shift) {
          if (const auto shifted = applyGoalLateralShift(
                final_segment, goal_pose, *parameters_, planner_data_->route_handler)) {
            final_segment = *shifted;
          }
        }
      } catch (...) {
      }
    }

    output.path = final_segment;
    if (!is_ego_driving_forward_wrt_lane_) {
      flipPathPointOrientation(output.path);
    }
  } else {
    const auto * next_cusp = getFirstUnvisitedCusp();
    const auto * last_visited = getLastVisitedCusp();
    std::optional<geometry_msgs::msg::Pose> start_cusp_pose;
    if (last_visited) {
      start_cusp_pose = last_visited->pose;
    }
    output.path = slicePathBetweenCusps(
      current_reference_path, ego_pose, start_cusp_pose, next_cusp->pose);

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
  if (!allCuspsVisited()) {
    return false;
  }

  if (
    current_segment_state_ != PathSegmentState::REVERSE_FOLLOWING &&
    current_segment_state_ != PathSegmentState::FORWARD_FOLLOWING) {
    return false;
  }

  if (!planner_data_ || !planner_data_->self_odometry || modified_path_.points.empty()) {
    return false;
  }

  const auto & ego_pose = planner_data_->self_odometry->pose.pose;
  const auto ego_nearest_idx_opt = findNearestIndex(modified_path_.points, ego_pose);
  if (!ego_nearest_idx_opt || *ego_nearest_idx_opt >= modified_path_.points.size()) {
    return false;
  }

  const double remaining_distance = calcSignedArcLength(
    modified_path_.points, *ego_nearest_idx_opt, modified_path_.points.size() - 1);

  if (remaining_distance < parameters_->th_arrived_distance) {
    current_segment_state_ = PathSegmentState::COMPLETED;
    RCLCPP_DEBUG(
      getLogger(), "Ego completed final segment, state=COMPLETED, module can transit to SUCCESS");
    return true;
  }
  return false;
}

}  // namespace autoware::behavior_path_planner
