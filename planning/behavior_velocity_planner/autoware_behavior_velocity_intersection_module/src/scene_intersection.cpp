// Copyright 2020 Tier IV, Inc.
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

#include "autoware/behavior_velocity_intersection_module/scene_intersection.hpp"

#include "autoware/behavior_velocity_intersection_module/util.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>  // for toGeomPoly
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>  // for toPolygon2d
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>

#include <boost/geometry/algorithms/within.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
IntersectionModule::IntersectionModule(
  const int64_t module_id, const int64_t lane_id, const PlannerParam & planner_param,
  const std::set<lanelet::Id> & associative_ids, const std::string & turn_direction,
  const bool has_traffic_light, rclcpp::Node & node, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_for_occlusion)
: SceneModuleInterfaceWithRTC(module_id, logger, clock, time_keeper, planning_factor_interface),
  planning_factor_interface_for_occlusion_(planning_factor_interface_for_occlusion),
  planner_param_(planner_param),
  lane_id_(lane_id),
  associative_ids_(associative_ids),
  turn_direction_(turn_direction),
  has_traffic_light_(has_traffic_light),
  occlusion_uuid_(autoware_utils::generate_uuid())
{
  {
    collision_state_machine_.setMarginTime(
      planner_param_.collision_detection.collision_detection_hold_time);
  }
  {
    before_creep_state_machine_.setMarginTime(
      planner_param_.occlusion.temporal_stop_time_before_peeking);
    before_creep_state_machine_.setState(StateMachine::State::STOP);
  }
  {
    occlusion_stop_state_machine_.setMarginTime(
      planner_param_.occlusion.occlusion_detection_hold_time);
    occlusion_stop_state_machine_.setState(StateMachine::State::GO);
  }
  {
    temporal_stop_before_attention_state_machine_.setMarginTime(
      planner_param_.occlusion.occlusion_detection_hold_time);
    temporal_stop_before_attention_state_machine_.setState(StateMachine::State::STOP);
  }
  {
    static_occlusion_timeout_state_machine_.setMarginTime(
      planner_param_.occlusion.static_occlusion_with_traffic_light_timeout);
    static_occlusion_timeout_state_machine_.setState(StateMachine::State::STOP);
  }

  ego_ttc_pub_ = node.create_publisher<autoware_internal_debug_msgs::msg::Float64MultiArrayStamped>(
    "~/debug/intersection/ego_ttc", 1);
  object_ttc_pub_ =
    node.create_publisher<autoware_internal_debug_msgs::msg::Float64MultiArrayStamped>(
      "~/debug/intersection/object_ttc", 1);
}

bool IntersectionModule::can_smoothly_stop_at(
  const PathWithLaneId & path, const size_t closest_idx, const size_t target_stop_idx,
  const PlannerData & planner_data) const
{
  const double braking_distance = planning_utils::calcJudgeLineDistWithJerkLimit(
    planner_data.current_velocity->twist.linear.x,
    planner_data.current_acceleration->accel.accel.linear.x, planner_param_.common.max_accel,
    planner_param_.common.max_jerk, planner_param_.common.delay_response_time);

  return autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, target_stop_idx) +
           planner_param_.common.stopline_overshoot_margin >
         braking_distance;
}

bool IntersectionModule::modifyPathVelocity(
  Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data)
{
  auto path_msg = planning_utils::fromTrajectory(path, left_bound, right_bound);

  debug_data_ = DebugData();

  initializeRTCStatus();

  const auto decision_result = modifyPathVelocityDetail(&path_msg, planner_data);
  prev_decision_result_ = decision_result;

  {
    const std::string decision_type =
      "intersection" + std::to_string(module_id_) + " : " +
      formatDecisionResult(decision_result, activated_, occlusion_activated_);
    internal_debug_data_.decision_type = decision_type;
  }

  prepareRTCStatus(decision_result, path_msg);

  reactRTCApproval(decision_result, &path_msg, planner_data);

  planning_utils::toTrajectory(path_msg, path);
  return true;
}

void IntersectionModule::initializeRTCStatus()
{
  setSafe(true);
  setDistance(std::numeric_limits<double>::lowest());
  // occlusion
  occlusion_safety_ = true;
  occlusion_stop_distance_ = std::numeric_limits<double>::lowest();
  occlusion_first_stop_required_ = false;
  // activated_ and occlusion_activated_ must be set from manager's RTC callback
}

static std::string formatOcclusionType(const IntersectionModule::OcclusionType & type)
{
  if (std::holds_alternative<IntersectionModule::NotOccluded>(type)) {
    return "NotOccluded and the best occlusion clearance is " +
           std::to_string(std::get<IntersectionModule::NotOccluded>(type).best_clearance_distance);
  }
  if (std::holds_alternative<IntersectionModule::StaticallyOccluded>(type)) {
    return "StaticallyOccluded and the best occlusion clearance is " +
           std::to_string(
             std::get<IntersectionModule::StaticallyOccluded>(type).best_clearance_distance);
  }
  if (std::holds_alternative<IntersectionModule::DynamicallyOccluded>(type)) {
    return "DynamicallyOccluded and the best occlusion clearance is " +
           std::to_string(
             std::get<IntersectionModule::DynamicallyOccluded>(type).best_clearance_distance);
  }
  return "RTCOccluded";
}

DecisionResult IntersectionModule::modifyPathVelocityDetail(
  PathWithLaneId * path, const PlannerData & planner_data)
{
  const auto prepare_data = prepareIntersectionData(path, planner_data);
  if (!prepare_data) {
    return prepare_data.err();
  }
  const auto [interpolated_path_info, intersection_stoplines, path_lanelets] = prepare_data.ok();
  const auto & intersection_lanelets = intersection_lanelets_.value();

  // NOTE: this level is based on the updateTrafficSignalObservation() which is latest
  const auto traffic_prioritized_level = getTrafficPrioritizedLevel(planner_data);
  const bool is_prioritized =
    traffic_prioritized_level == TrafficPrioritizedLevel::FULLY_PRIORITIZED;

  const auto closest_idx = intersection_stoplines.closest_idx;
  // ==========================================================================================
  // stuck detection
  //
  // stuck vehicle detection is viable even if attention area is empty
  // so this needs to be checked before attention area validation
  // ==========================================================================================
  const auto is_stuck_status =
    isStuckStatus(*path, intersection_stoplines, path_lanelets, planner_data);
  if (is_stuck_status) {
    if (can_smoothly_stop_at(
          *path, closest_idx, is_stuck_status->stuck_stopline_idx, planner_data)) {
      return is_stuck_status.value();
    }
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "stuck vehicle detected, but give up stop to avoid will_overrun MRM");
  }

  // ==========================================================================================
  // basic data validation
  //
  // if attention area is empty, collision/occlusion detection is impossible
  //
  // if attention area is not null but default stop line is not available, ego/backward-path has
  // already passed the stop line, so ego is already in the middle of the intersection, or the
  // end of the ego path has just entered the entry of this intersection
  //
  // occlusion stop line is generated from the intersection of ego footprint along the path with the
  // attention area, so if this is null, ego has already passed the intersection, or the end of the
  // ego path has just entered the entry of this intersection
  // ==========================================================================================
  if (!intersection_lanelets.first_attention_area()) {
    return InternalError{"attention area is empty"};
  }
  const auto first_attention_area = intersection_lanelets.first_attention_area().value();
  const auto default_stopline_idx_opt = intersection_stoplines.default_stopline;
  if (!default_stopline_idx_opt) {
    return InternalError{"default stop line is null"};
  }
  const auto default_stopline_idx = default_stopline_idx_opt.value();
  const auto first_attention_stopline_idx = intersection_stoplines.first_attention_stopline;

  const auto collision_stopline_idx_opt = intersection_stoplines.collision_stopline;
  if (!collision_stopline_idx_opt) {
    return InternalError{"collision stop line is null"};
  }
  const auto collision_stopline_idx = collision_stopline_idx_opt.value();

  const auto occlusion_peeking_stopline_idx_opt = intersection_stoplines.occlusion_peeking_stopline;
  if (!occlusion_peeking_stopline_idx_opt) {
    return InternalError{"occlusion stop line is null"};
  }
  const auto occlusion_stopline_idx = occlusion_peeking_stopline_idx_opt.value();

  // ==========================================================================================
  // classify the objects to attention_area/intersection_area and update their position, velocity,
  // belonging attention lanelet, distance to corresponding stopline
  // ==========================================================================================
  updateObjectInfoManagerArea(planner_data);

  // ==========================================================================================
  // occlusion_status is type of occlusion,
  // is_occlusion_cleared_with_margin indicates if occlusion is physically detected
  // is_occlusion_state indicates if occlusion is detected. OR occlusion is not detected but RTC for
  // intersection_occlusion is disapproved, which means ego is virtually occluded
  //
  // so is_occlusion_cleared_with_margin should be sent to RTC as module decision
  // and is_occlusion_status should be only used to decide ego action
  // !is_occlusion_state == !physically_occluded && !externally_occluded, so even if occlusion is
  // not detected but not approved, SAFE is not sent.
  // ==========================================================================================
  const auto [occlusion_status, is_occlusion_cleared_with_margin, is_occlusion_state] =
    getOcclusionStatus(traffic_prioritized_level, interpolated_path_info, planner_data);

  const auto [is_over_pass_judge_line, safely_passed_judge_line] =
    isOverPassJudgeLinesStatus(*path, is_occlusion_state, intersection_stoplines, planner_data);

  // ==========================================================================================
  // calculate the expected vehicle speed and obtain the spatiotemporal profile of ego to the
  // exit of intersection
  // ==========================================================================================
  autoware_internal_debug_msgs::msg::Float64MultiArrayStamped ego_ttc_time_array;
  const auto time_distance_array = calcIntersectionPassingTime(
    *path, is_prioritized, intersection_stoplines, &ego_ttc_time_array, planner_data);

  // ==========================================================================================
  // run collision checking for each objects considering traffic light level. Also if ego just
  // passed each pass judge line for the first time, save current collision status for late
  // diagnosis
  // ==========================================================================================
  autoware_internal_debug_msgs::msg::Float64MultiArrayStamped object_ttc_time_array;
  updateObjectInfoManagerCollision(
    path_lanelets, time_distance_array, traffic_prioritized_level, safely_passed_judge_line,
    &object_ttc_time_array, planner_data);
  {
    const auto & debug = planner_param_.debug.ttc;
    if (
      std::find(debug.begin(), debug.end(), lane_id_) != debug.end() ||
      std::find(debug.begin(), debug.end(), -1) != debug.end()) {
      ego_ttc_pub_->publish(ego_ttc_time_array);
      object_ttc_pub_->publish(object_ttc_time_array);
    }
  }

  safety_factor_array_.factors.clear();
  safety_factor_array_.header.stamp = clock_->now();
  safety_factor_array_.header.frame_id = "map";
  for (const auto & object_info : object_info_manager_.attentionObjects()) {
    const auto & unsafe_info = object_info->unsafe_info();
    if (!unsafe_info) {
      continue;
    }
    setObjectsOfInterestData(
      object_info->predicted_object().kinematics.initial_pose_with_covariance.pose,
      object_info->predicted_object().shape, ColorName::RED);

    autoware_internal_planning_msgs::msg::SafetyFactor safety_factor;
    safety_factor.object_id = object_info->predicted_object().object_id;
    safety_factor.type = autoware_internal_planning_msgs::msg::SafetyFactor::OBJECT;

    // TODO(odashima): add a predicted path used for decision.
    // safety_factor.predicted_path =
    safety_factor.ttc_begin = unsafe_info->interval_time.first;
    safety_factor.ttc_end = unsafe_info->interval_time.second;
    safety_factor.is_safe = false;

    safety_factor.points = {
      object_info->predicted_object().kinematics.initial_pose_with_covariance.pose.position};
    safety_factor_array_.factors.push_back(safety_factor);
  }

  safety_factor_array_.is_safe = std::all_of(
    safety_factor_array_.factors.begin(), safety_factor_array_.factors.end(),
    [](const auto & factor) { return factor.is_safe; });

  const auto [has_collision, too_late_detect_objects, misjudge_objects] =
    detectCollision(is_over_pass_judge_line);
  collision_state_machine_.setStateWithMarginTime(
    has_collision ? StateMachine::State::STOP : StateMachine::State::GO,
    logger_.get_child("collision state_machine"), *clock_);
  const bool has_collision_with_margin =
    collision_state_machine_.getState() == StateMachine::State::STOP;
  const std::string safety_diag =
    generateDetectionBlameDiagnosis(too_late_detect_objects, misjudge_objects);
  const std::string occlusion_diag = formatOcclusionType(occlusion_status);

  const bool enable_conservative_yield_merging = planner_param_.conservative_merging.enable_yield;
  if (is_permanent_go_) {
    if (
      has_collision_with_margin && !has_traffic_light_ && enable_conservative_yield_merging &&
      intersection_stoplines.maximum_footprint_overshoot_line) {
      if (can_smoothly_stop_at(
            *path, closest_idx, intersection_stoplines.maximum_footprint_overshoot_line.value(),
            planner_data)) {
        // NOTE(soblin): intersection_stoplines.maximum_footprint_overshoot_line.value() is not used
        // as stop line. in this case, ego tries to stop at current position
        const auto stop_line_idx = closest_idx;
        return NonOccludedCollisionStop{
          closest_idx, stop_line_idx, occlusion_stopline_idx, occlusion_diag};
      }
    }
    if (has_collision) {
      const std::string evasive_diag = generateEgoRiskEvasiveDiagnosis(
        *path, closest_idx, time_distance_array, too_late_detect_objects, misjudge_objects,
        planner_data);
      debug_data_.too_late_stop_wall_pose = path->points.at(closest_idx).point.pose;
      return OverPassJudge{safety_diag, evasive_diag};
    }
    return OverPassJudge{
      "no collision is detected", "ego can safely pass the intersection at this rate"};
  }

  // ==========================================================================================
  // pseudo collision detection on green light
  // ==========================================================================================
  const auto is_green_pseudo_collision_status =
    isGreenPseudoCollisionStatus(closest_idx, collision_stopline_idx, intersection_stoplines);
  if (is_green_pseudo_collision_status) {
    if (can_smoothly_stop_at(*path, closest_idx, collision_stopline_idx, planner_data)) {
      return is_green_pseudo_collision_status.value();
    }
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "green pseudo collision detected, but give up stop to avoid overrun");
  }

  // ==========================================================================================
  // yield stuck detection
  // ==========================================================================================
  const auto yield_stuck_status =
    isYieldStuckStatus(*path, interpolated_path_info, intersection_stoplines, planner_data);
  if (yield_stuck_status) {
    if (can_smoothly_stop_at(
          *path, closest_idx, yield_stuck_status->stuck_stopline_idx, planner_data)) {
      return yield_stuck_status.value();
    }
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "yield stuck detected, but give up stop to avoid overrun");
  }

  if (is_prioritized) {
    return FullyPrioritized{
      has_collision_with_margin, closest_idx, collision_stopline_idx, occlusion_stopline_idx,
      safety_diag};
  }

  // Safe
  if (!is_occlusion_state && !has_collision_with_margin) {
    return Safe{closest_idx, collision_stopline_idx, occlusion_stopline_idx, occlusion_diag};
  }

  const bool collision_stop_feasible =
    can_smoothly_stop_at(*path, closest_idx, collision_stopline_idx, planner_data);
  const bool collision_stop_tolerable =
    collision_stop_feasible || previous_stop_pose_.collision_stopline_pose;

  // Only collision
  if (!is_occlusion_state && has_collision_with_margin) {
    if (collision_stop_tolerable) {
      if (!collision_stop_feasible) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 5000,
          "keep non-occluded collision (infeasible) stop from previous iteration");
      }
      return NonOccludedCollisionStop{
        closest_idx, collision_stopline_idx, occlusion_stopline_idx, occlusion_diag};
    }
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "non-occluded collision detected, but give up stop to avoid overrun");
    collision_state_machine_.setState(StateMachine::State::GO);
    return InternalError{
      "non-occluded collision detected, but give up stop to avoid will_overrun MRM"};
  }

  // Occluded
  // utility functions
  auto fromEgoDist = [&](const size_t index) {
    return autoware::motion_utils::calcSignedArcLength(path->points, closest_idx, index);
  };
  auto stoppedForDurationOrPassed = [&](const size_t pos, StateMachine & state_machine) {
    static constexpr double approached_dist_threshold = 1.0;
    const double dist_stopline = fromEgoDist(pos);
    const bool approached_stopline = (dist_stopline < approached_dist_threshold);
    const bool over_stopline = (dist_stopline < -approached_dist_threshold);
    const bool is_stopped_duration = planner_data.isVehicleStopped();
    if (over_stopline || (is_stopped_duration && approached_stopline)) {
      state_machine.setStateWithMarginTime(
        StateMachine::State::GO, logger_.get_child("stoppedForDurationOrPassed"), *clock_);
    }
    return state_machine.getState() == StateMachine::State::GO;
  };
  auto stoppedAtPosition = [&](const size_t pos, const double duration) {
    const double dist_stopline = fromEgoDist(pos);
    const bool approached_dist_stopline =
      (std::fabs(dist_stopline) < planner_param_.common.stopline_overshoot_margin);
    const bool over_stopline = (dist_stopline < -planner_param_.common.stopline_overshoot_margin);
    const bool is_stopped = planner_data.isVehicleStopped(duration);
    return over_stopline || (is_stopped && approached_dist_stopline);
  };

  const bool stopped_at_default_line_or_passed =
    stoppedForDurationOrPassed(default_stopline_idx, before_creep_state_machine_);

  if (!stopped_at_default_line_or_passed) {
    if (can_smoothly_stop_at(*path, closest_idx, default_stopline_idx, planner_data)) {
      return FirstWaitBeforeOcclusion{
        is_occlusion_cleared_with_margin, closest_idx, default_stopline_idx, occlusion_stopline_idx,
        occlusion_diag};
    }
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "want to do FirstWaitBeforeOcclusion, but give up stop to avoid overrun");
  }
  // ==========================================================================================
  // ego stop at occlusion_stopline before entering attention area if there is no traffic light
  // ==========================================================================================
  const bool occlusion_stop_feasible =
    can_smoothly_stop_at(*path, closest_idx, occlusion_stopline_idx, planner_data);
  const bool occlusion_stop_tolerable =
    occlusion_stop_feasible || previous_stop_pose_.occlusion_peeking_stopline_pose;

  if (!has_traffic_light_) {
    const bool temporal_stop_before_creep_required = !stoppedForDurationOrPassed(
      occlusion_stopline_idx, temporal_stop_before_attention_state_machine_);

    if (has_collision_with_margin && !collision_stop_tolerable) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "collision detected in OccludedAbsenceTrafficLight, but give up stop to avoid overrun");
    }

    if (!occlusion_stop_tolerable) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "occlusion detected in OccludedAbsenceTrafficLight, but give up stop to avoid overrun");
    }
    return OccludedAbsenceTrafficLight{
      is_occlusion_cleared_with_margin,
      has_collision_with_margin,
      temporal_stop_before_creep_required,
      closest_idx,
      occlusion_stopline_idx,
      first_attention_stopline_idx,
      occlusion_diag,
      collision_stop_tolerable,
      occlusion_stop_tolerable};
  }

  // ==========================================================================================
  // following remaining block is "has_traffic_light_"
  //
  // if ego is stuck by static occlusion in the presence of traffic light, start timeout count
  // ==========================================================================================
  const bool is_static_occlusion = std::holds_alternative<StaticallyOccluded>(occlusion_status);
  const bool is_stuck_by_static_occlusion =
    stoppedAtPosition(
      occlusion_stopline_idx, planner_param_.occlusion.temporal_stop_time_before_peeking) &&
    is_static_occlusion;
  if (has_collision_with_margin) {
    // if collision is detected, timeout is reset
    static_occlusion_timeout_state_machine_.setState(StateMachine::State::STOP);
  } else if (is_stuck_by_static_occlusion) {
    static_occlusion_timeout_state_machine_.setStateWithMarginTime(
      StateMachine::State::GO, logger_.get_child("static_occlusion"), *clock_);
  }
  const bool release_static_occlusion_stuck =
    (static_occlusion_timeout_state_machine_.getState() == StateMachine::State::GO);
  if (!has_collision_with_margin && release_static_occlusion_stuck) {
    return Safe{closest_idx, collision_stopline_idx, occlusion_stopline_idx, occlusion_diag};
  }
  // occlusion_status is either STATICALLY_OCCLUDED or DYNAMICALLY_OCCLUDED
  const double max_timeout = planner_param_.occlusion.static_occlusion_with_traffic_light_timeout +
                             planner_param_.occlusion.occlusion_detection_hold_time;
  const std::optional<double> static_occlusion_timeout =
    is_stuck_by_static_occlusion
      ? std::make_optional<double>(
          max_timeout - static_occlusion_timeout_state_machine_.getDuration() -
          occlusion_stop_state_machine_.getDuration())
      : (is_static_occlusion ? std::make_optional<double>(max_timeout) : std::nullopt);

  if (has_collision_with_margin && !collision_stop_tolerable) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "collision detected in OccludedCollisionStop, but give up stop to avoid overrun");
  }
  if (!occlusion_stop_tolerable) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "occlusion detected in OccludedAbsenceTrafficLight, but give up stop to avoid overrun");
  }

  if (has_collision_with_margin) {
    return OccludedCollisionStop{is_occlusion_cleared_with_margin, closest_idx,
                                 collision_stopline_idx,           occlusion_stopline_idx,
                                 static_occlusion_timeout,         occlusion_diag,
                                 collision_stop_tolerable,         occlusion_stop_tolerable};
  }
  return PeekingTowardOcclusion{is_occlusion_cleared_with_margin, closest_idx,
                                collision_stopline_idx,           occlusion_stopline_idx,
                                static_occlusion_timeout,         occlusion_diag,
                                occlusion_stop_tolerable};
}

// template-specification based visitor pattern
// https://en.cppreference.com/w/cpp/utility/variant/visit
template <class... Ts>
struct VisitorSwitch : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
VisitorSwitch(Ts...) -> VisitorSwitch<Ts...>;

template <typename T>
void prepareRTCByDecisionResult(
  [[maybe_unused]] const T & result,
  [[maybe_unused]] const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  [[maybe_unused]] bool * default_safety, [[maybe_unused]] double * default_distance,
  [[maybe_unused]] bool * occlusion_safety, [[maybe_unused]] double * occlusion_distance)
{
  static_assert("Unsupported type passed to prepareRTCByDecisionResult");
}

template <>
void prepareRTCByDecisionResult(
  [[maybe_unused]] const InternalError & result,
  [[maybe_unused]] const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  [[maybe_unused]] bool * default_safety, [[maybe_unused]] double * default_distance,
  [[maybe_unused]] bool * occlusion_safety, [[maybe_unused]] double * occlusion_distance)
{
}

template <>
void prepareRTCByDecisionResult(
  [[maybe_unused]] const OverPassJudge & result,
  [[maybe_unused]] const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  [[maybe_unused]] bool * default_safety, [[maybe_unused]] double * default_distance,
  [[maybe_unused]] bool * occlusion_safety, [[maybe_unused]] double * occlusion_distance)
{
}

template <>
void prepareRTCByDecisionResult(
  const StuckStop & result, const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  bool * default_safety, double * default_distance, bool * occlusion_safety,
  [[maybe_unused]] double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "StuckStop");
  const auto closest_idx = result.closest_idx;
  const auto stopline_idx = result.stuck_stopline_idx;
  *default_safety = false;
  *default_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, stopline_idx);
  *occlusion_safety = true;
}

template <>
void prepareRTCByDecisionResult(
  const YieldStuckStop & result, const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  bool * default_safety, double * default_distance, bool * occlusion_safety,
  [[maybe_unused]] double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "YieldStuckStop");
  const auto closest_idx = result.closest_idx;
  const auto stopline_idx = result.stuck_stopline_idx;
  *default_safety = false;
  *default_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, stopline_idx);
  *occlusion_safety = true;
}

template <>
void prepareRTCByDecisionResult(
  const NonOccludedCollisionStop & result,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "NonOccludedCollisionStop");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  *default_safety = false;
  *default_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  const auto occlusion_stopline = result.occlusion_stopline_idx;
  *occlusion_safety = true;
  *occlusion_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const FirstWaitBeforeOcclusion & result,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "FirstWaitBeforeOcclusion");
  const auto closest_idx = result.closest_idx;
  const auto first_stopline_idx = result.first_stopline_idx;
  const auto occlusion_stopline_idx = result.occlusion_stopline_idx;
  *default_safety = false;
  *default_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, first_stopline_idx);
  *occlusion_safety = result.is_actually_occlusion_cleared;
  *occlusion_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const PeekingTowardOcclusion & result,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "PeekingTowardOcclusion");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  const auto occlusion_stopline_idx = result.occlusion_stopline_idx;
  *default_safety = true;
  *default_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  *occlusion_safety = result.is_actually_occlusion_cleared;
  *occlusion_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const OccludedAbsenceTrafficLight & result,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "OccludedAbsenceTrafficLight");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.closest_idx;
  *default_safety = !result.collision_detected;
  *default_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  *occlusion_safety = result.is_actually_occlusion_cleared;
  *occlusion_distance = 0;
  return;
}

template <>
void prepareRTCByDecisionResult(
  const OccludedCollisionStop & result,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "OccludedCollisionStop");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  const auto occlusion_stopline_idx = result.occlusion_stopline_idx;
  *default_safety = false;
  *default_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  *occlusion_safety = result.is_actually_occlusion_cleared;
  *occlusion_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const Safe & result, const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  bool * default_safety, double * default_distance, bool * occlusion_safety,
  double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "Safe");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  const auto occlusion_stopline_idx = result.occlusion_stopline_idx;
  *default_safety = true;
  *default_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  *occlusion_safety = true;
  *occlusion_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const FullyPrioritized & result,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "FullyPrioritized");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  const auto occlusion_stopline_idx = result.occlusion_stopline_idx;
  *default_safety = !result.collision_detected;
  *default_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  *occlusion_safety = true;
  *occlusion_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
}

void IntersectionModule::prepareRTCStatus(
  const DecisionResult & decision_result,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  bool default_safety = true;
  double default_distance = std::numeric_limits<double>::lowest();
  std::visit(
    VisitorSwitch{[&](const auto & decision) {
      prepareRTCByDecisionResult(
        decision, path, &default_safety, &default_distance, &occlusion_safety_,
        &occlusion_stop_distance_);
    }},
    decision_result);
  setSafe(default_safety);
  setDistance(default_distance);
  occlusion_first_stop_required_ =
    std::holds_alternative<FirstWaitBeforeOcclusion>(decision_result);
}

template <typename T>
void reactRTCApprovalByDecisionResult(
  [[maybe_unused]] const bool rtc_default_approved,
  [[maybe_unused]] const bool rtc_occlusion_approved, [[maybe_unused]] const T & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  [[maybe_unused]] const double baselink2front,
  [[maybe_unused]] autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  [[maybe_unused]] planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  [[maybe_unused]] planning_factor_interface::PlanningFactorInterface *
    planning_factor_interface_for_occlusion,
  [[maybe_unused]] IntersectionModule::DebugData * debug_data,
  [[maybe_unused]] IntersectionStopLines::PreviousStopPose * previous_stop_pose)
{
  static_assert("Unsupported type passed to reactRTCByDecisionResult");
}

template <>
void reactRTCApprovalByDecisionResult(
  [[maybe_unused]] const bool rtc_default_approved,
  [[maybe_unused]] const bool rtc_occlusion_approved,
  [[maybe_unused]] const InternalError & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  [[maybe_unused]] const double baselink2front,
  [[maybe_unused]] autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  [[maybe_unused]] planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  [[maybe_unused]] planning_factor_interface::PlanningFactorInterface *
    planning_factor_interface_for_occlusion,
  [[maybe_unused]] IntersectionModule::DebugData * debug_data,
  [[maybe_unused]] IntersectionStopLines::PreviousStopPose * previous_stop_pose)
{
}

template <>
void reactRTCApprovalByDecisionResult(
  [[maybe_unused]] const bool rtc_default_approved,
  [[maybe_unused]] const bool rtc_occlusion_approved,
  [[maybe_unused]] const OverPassJudge & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  [[maybe_unused]] const double baselink2front,
  [[maybe_unused]] autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  [[maybe_unused]] planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  [[maybe_unused]] planning_factor_interface::PlanningFactorInterface *
    planning_factor_interface_for_occlusion,
  [[maybe_unused]] IntersectionModule::DebugData * debug_data,
  [[maybe_unused]] IntersectionStopLines::PreviousStopPose * previous_stop_pose)
{
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const StuckStop & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  [[maybe_unused]] planning_factor_interface::PlanningFactorInterface *
    planning_factor_interface_for_occlusion,
  IntersectionModule::DebugData * debug_data,
  [[maybe_unused]] IntersectionStopLines::PreviousStopPose * previous_stop_pose)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "StuckStop, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  const auto closest_idx = decision_result.closest_idx;
  if (!rtc_default_approved) {
    // use default_rtc uuid for stuck vehicle detection
    const auto stopline_idx = decision_result.stuck_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);

    const auto stop_pose = path->points.at(stopline_idx).point.pose;
    // NOTE(soblin): following process is not intentionally off for Stuck/YieldStuck
    // previous_stop_pose->collision_stopline_pose = stop_pose;

    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      planning_factor_interface->add(
        path->points, path->points.at(closest_idx).point.pose, stop_pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP, safety_factor_array,
        true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/, "stuck stop");
    }
  }
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const YieldStuckStop & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  [[maybe_unused]] planning_factor_interface::PlanningFactorInterface *
    planning_factor_interface_for_occlusion,
  IntersectionModule::DebugData * debug_data,
  [[maybe_unused]] IntersectionStopLines::PreviousStopPose * previous_stop_pose)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "YieldStuckStop, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  const auto closest_idx = decision_result.closest_idx;
  if (!rtc_default_approved) {
    // use default_rtc uuid for stuck vehicle detection
    const auto stopline_idx = decision_result.stuck_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);

    const auto stop_pose = path->points.at(stopline_idx).point.pose;
    // NOTE(soblin): following process is not intentionally off for Stuck/YieldStuck
    // previous_stop_pose->collision_stopline_pose = stop_pose;

    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      planning_factor_interface->add(
        path->points, path->points.at(closest_idx).point.pose, stop_pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP, safety_factor_array,
        true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/, "yield stuck");
    }
  }
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const NonOccludedCollisionStop & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  planning_factor_interface::PlanningFactorInterface * planning_factor_interface_for_occlusion,
  IntersectionModule::DebugData * debug_data,
  IntersectionStopLines::PreviousStopPose * previous_stop_pose)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "NonOccludedCollisionStop, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stopline_idx = decision_result.collision_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);

    const auto stop_pose = path->points.at(stopline_idx).point.pose;
    previous_stop_pose->collision_stopline_pose = stop_pose;

    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      planning_factor_interface->add(
        path->points, path->points.at(decision_result.closest_idx).point.pose, stop_pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP, safety_factor_array,
        true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/, "collision stop");
    }
  }
  if (!rtc_occlusion_approved) {
    const auto stopline_idx = decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);

    const auto stop_pose = path->points.at(stopline_idx).point.pose;
    previous_stop_pose->occlusion_peeking_stopline_pose = stop_pose;

    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      planning_factor_interface_for_occlusion->add(
        path->points, path->points.at(decision_result.closest_idx).point.pose, stop_pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
        autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
        0.0 /*shift distance*/, "");
    }
  }
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const FirstWaitBeforeOcclusion & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  planning_factor_interface::PlanningFactorInterface * planning_factor_interface_for_occlusion,
  IntersectionModule::DebugData * debug_data,
  IntersectionStopLines::PreviousStopPose * previous_stop_pose)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "FirstWaitBeforeOcclusion, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stopline_idx = decision_result.first_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);

    const auto stop_pose = path->points.at(stopline_idx).point.pose;
    // NOTE(soblin): following process is not intentionally off for FirstWaitBeforeOcclusion
    // previous_stop_pose->collision_stopline_pose = stop_pose;
    previous_stop_pose->collision_stopline_pose = stop_pose;

    debug_data->occlusion_first_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      planning_factor_interface->add(
        path->points, path->points.at(decision_result.closest_idx).point.pose, stop_pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP, safety_factor_array,
        true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/, "first wait");
    }
  }
  if (!rtc_occlusion_approved) {
    const auto stopline_idx = decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);

    const auto stop_pose = path->points.at(stopline_idx).point.pose;
    // NOTE(soblin): following process is not intentionally off for FirstWaitBeforeOcclusion
    previous_stop_pose->occlusion_peeking_stopline_pose = stop_pose;

    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      planning_factor_interface_for_occlusion->add(
        path->points, path->points.at(decision_result.closest_idx).point.pose, stop_pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
        autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
        0.0 /*shift distance*/, "");
    }
  }
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const PeekingTowardOcclusion & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  [[maybe_unused]] planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  planning_factor_interface::PlanningFactorInterface * planning_factor_interface_for_occlusion,
  IntersectionModule::DebugData * debug_data,
  IntersectionStopLines::PreviousStopPose * previous_stop_pose)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "PeekingTowardOcclusion, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  // NOTE: creep_velocity should be inserted first at closest_idx if !rtc_default_approved
  if (!rtc_occlusion_approved && decision_result.occlusion_stop_tolerable) {
    const size_t occlusion_peeking_stopline = decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(occlusion_peeking_stopline, 0.0, path);

    const auto stop_pose = path->points.at(occlusion_peeking_stopline).point.pose;
    previous_stop_pose->occlusion_peeking_stopline_pose = stop_pose;

    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(occlusion_peeking_stopline, baselink2front, *path);
    debug_data->static_occlusion_with_traffic_light_timeout =
      decision_result.static_occlusion_timeout;
    {
      planning_factor_interface_for_occlusion->add(
        path->points, path->points.at(decision_result.closest_idx).point.pose, stop_pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
        autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
        0.0 /*shift distance*/, "");
    }
  }
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const OccludedCollisionStop & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  planning_factor_interface::PlanningFactorInterface * planning_factor_interface_for_occlusion,
  IntersectionModule::DebugData * debug_data,
  IntersectionStopLines::PreviousStopPose * previous_stop_pose)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "OccludedCollisionStop, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved && decision_result.collision_stop_tolerable) {
    const auto stopline_idx = decision_result.collision_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);

    const auto stop_pose = path->points.at(stopline_idx).point.pose;
    previous_stop_pose->collision_stopline_pose = stop_pose;

    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      planning_factor_interface->add(
        path->points, path->points.at(decision_result.closest_idx).point.pose, stop_pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP, safety_factor_array,
        true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/, "collision");
    }
  }
  if (!rtc_occlusion_approved && decision_result.occlusion_stop_tolerable) {
    const auto stopline_idx = decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);

    const auto stop_pose = path->points.at(stopline_idx).point.pose;
    previous_stop_pose->occlusion_peeking_stopline_pose = stop_pose;

    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    debug_data->static_occlusion_with_traffic_light_timeout =
      decision_result.static_occlusion_timeout;
    {
      planning_factor_interface_for_occlusion->add(
        path->points, path->points.at(decision_result.closest_idx).point.pose, stop_pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
        autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
        0.0 /*shift distance*/, "");
    }
  }
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const OccludedAbsenceTrafficLight & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  [[maybe_unused]] planning_factor_interface::PlanningFactorInterface *
    planning_factor_interface_for_occlusion,
  IntersectionModule::DebugData * debug_data,
  IntersectionStopLines::PreviousStopPose * previous_stop_pose)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "OccludedAbsenceTrafficLight, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved && decision_result.collision_stop_tolerable) {
    const auto stopline_idx = decision_result.closest_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);

    const auto stop_pose = path->points.at(stopline_idx).point.pose;
    previous_stop_pose->collision_stopline_pose = stop_pose;

    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      planning_factor_interface->add(
        path->points, path->points.at(decision_result.closest_idx).point.pose, stop_pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP, safety_factor_array,
        true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/, "collision");
    }
  }
  if (
    !rtc_occlusion_approved &&
    (decision_result.temporal_stop_before_attention_required ||
     planner_param.occlusion.request_approval_wo_traffic_light) &&
    decision_result.occlusion_stop_tolerable) {
    const auto closest_idx = decision_result.closest_idx;

    // NOTE(soblin): to avoid "will_overrun_stop_point", creep velocity is needed
    const auto peeking_limit_line = decision_result.peeking_limit_line_idx;
    for (auto i = closest_idx; i <= peeking_limit_line; ++i) {
      planning_utils::setVelocityFromIndex(
        i, planner_param.occlusion.creep_velocity_without_traffic_light, path);
    }
    const auto stopline_idx = decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);

    const auto stop_pose = path->points.at(stopline_idx).point.pose;
    previous_stop_pose->occlusion_peeking_stopline_pose = stop_pose;

    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      planning_factor_interface_for_occlusion->add(
        path->points, path->points.at(decision_result.closest_idx).point.pose, stop_pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
        autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
        0.0 /*shift distance*/, "");
    }
  }
  if (!rtc_occlusion_approved && !decision_result.temporal_stop_before_attention_required) {
    const auto closest_idx = decision_result.closest_idx;
    const auto peeking_limit_line = decision_result.peeking_limit_line_idx;
    for (auto i = closest_idx; i <= peeking_limit_line; ++i) {
      planning_utils::setVelocityFromIndex(
        i, planner_param.occlusion.creep_velocity_without_traffic_light, path);
    }
    debug_data->absence_traffic_light_creep_wall =
      planning_utils::getAheadPose(closest_idx, baselink2front, *path);
  }
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved, const Safe & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  planning_factor_interface::PlanningFactorInterface * planning_factor_interface_for_occlusion,
  IntersectionModule::DebugData * debug_data,
  IntersectionStopLines::PreviousStopPose * previous_stop_pose)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "Safe, approval = (default: %d, occlusion: %d)", rtc_default_approved, rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stopline_idx = decision_result.collision_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);

    const auto stop_pose = path->points.at(stopline_idx).point.pose;
    previous_stop_pose->collision_stopline_pose = stop_pose;

    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      planning_factor_interface->add(
        path->points, path->points.at(decision_result.closest_idx).point.pose, stop_pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP, safety_factor_array,
        true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/, "");
    }
  }
  if (!rtc_occlusion_approved) {
    const auto stopline_idx = decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);

    const auto stop_pose = path->points.at(stopline_idx).point.pose;
    previous_stop_pose->occlusion_peeking_stopline_pose = stop_pose;

    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      planning_factor_interface_for_occlusion->add(
        path->points, path->points.at(decision_result.closest_idx).point.pose, stop_pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
        autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
        0.0 /*shift distance*/, "");
    }
  }
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const FullyPrioritized & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  [[maybe_unused]] planning_factor_interface::PlanningFactorInterface *
    planning_factor_interface_for_occlusion,
  IntersectionModule::DebugData * debug_data,
  [[maybe_unused]] IntersectionStopLines::PreviousStopPose * previous_stop_pose)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "FullyPrioritized, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stopline_idx = decision_result.collision_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      planning_factor_interface->add(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP, safety_factor_array,
        true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/, "prioritized");
    }
  }
}

void IntersectionModule::reactRTCApproval(
  const DecisionResult & decision_result,
  autoware_internal_planning_msgs::msg::PathWithLaneId * path, const PlannerData & planner_data)
{
  const double baselink2front = planner_data.vehicle_info_.max_longitudinal_offset_m;

  IntersectionStopLines::PreviousStopPose current_stop_pose{};

  std::visit(
    VisitorSwitch{[&](const auto & decision) {
      reactRTCApprovalByDecisionResult(
        activated_, occlusion_activated_, decision, planner_param_, baselink2front, path,
        safety_factor_array_, planning_factor_interface_.get(),
        planning_factor_interface_for_occlusion_.get(), &debug_data_, &current_stop_pose);
    }},
    decision_result);

  // NOTE(soblin): this is to refresh `previous_stop_pose_`
  previous_stop_pose_ = current_stop_pose;
}

bool IntersectionModule::isGreenSolidOn() const
{
  using TrafficSignalElement = autoware_perception_msgs::msg::TrafficLightElement;

  if (!last_tl_valid_observation_) {
    return false;
  }
  const auto & tl_info = last_tl_valid_observation_.value();
  for (auto && tl_light : tl_info.signal.elements) {
    if (
      tl_light.color == TrafficSignalElement::GREEN &&
      tl_light.shape == TrafficSignalElement::CIRCLE) {
      return true;
    }
  }
  return false;
}

IntersectionModule::TrafficPrioritizedLevel IntersectionModule::getTrafficPrioritizedLevel(
  const PlannerData & planner_data) const
{
  using TrafficSignalElement = autoware_perception_msgs::msg::TrafficLightElement;

  auto corresponding_arrow = [&](const TrafficSignalElement & element) {
    if (turn_direction_ == "straight" && element.shape == TrafficSignalElement::UP_ARROW) {
      return true;
    }
    if (turn_direction_ == "left" && element.shape == TrafficSignalElement::LEFT_ARROW) {
      return true;
    }
    if (turn_direction_ == "right" && element.shape == TrafficSignalElement::RIGHT_ARROW) {
      return true;
    }
    return false;
  };

  // ==========================================================================================
  // if no traffic light information has been available, it is UNKNOWN state which is treated as
  // NOT_PRIORITIZED
  //
  // if there has been any information available in the past more than once, the last valid
  // information is kept and used for planning
  // ==========================================================================================
  auto level = TrafficPrioritizedLevel::NOT_PRIORITIZED;
  if (last_tl_valid_observation_) {
    auto color = TrafficSignalElement::GREEN;
    const auto & tl_info = last_tl_valid_observation_.value();
    bool has_amber_signal{false};
    for (auto && tl_light : tl_info.signal.elements) {
      if (
        tl_light.color == TrafficSignalElement::AMBER &&
        tl_light.shape == TrafficSignalElement::CIRCLE) {
        has_amber_signal = true;
      }
      if (
        (tl_light.color == TrafficSignalElement::RED &&
         tl_light.shape == TrafficSignalElement::CIRCLE) ||
        (tl_light.color == TrafficSignalElement::GREEN && corresponding_arrow(tl_light))) {
        // NOTE: Return here since the red signal has the highest priority.
        level = TrafficPrioritizedLevel::FULLY_PRIORITIZED;
        color = TrafficSignalElement::RED;
        break;
      }
    }
    if (has_amber_signal) {
      level = TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED;
      color = TrafficSignalElement::AMBER;
    }
    if (tl_id_and_point_) {
      const auto [tl_id, point] = tl_id_and_point_.value();
      debug_data_.traffic_light_observation =
        std::make_tuple(planner_data.current_odometry->pose, point, tl_id, color);
    }
  }
  return level;
}

void IntersectionModule::updateTrafficSignalObservation(const PlannerData & planner_data)
{
  const auto lanelet_map_ptr = planner_data.route_handler_->getLaneletMapPtr();
  const auto & lane = lanelet_map_ptr->laneletLayer.get(lane_id_);

  if (!tl_id_and_point_) {
    for (auto && tl_reg_elem :
         lane.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>()) {
      for (const auto & light : tl_reg_elem->trafficLights()) {
        if (!light.isLineString()) {
          RCLCPP_WARN_ONCE(
            logger_,
            "traffic light(%ld) of AutowareTrafficLight regulatory-element(%ld) is not LineString",
            light.id(), tl_reg_elem->id());
        }
        const auto & tl_linestring = static_cast<lanelet::ConstLineString3d>(light);
        tl_id_and_point_ = std::make_pair(tl_reg_elem->id(), tl_linestring.front());
        break;
      }
      if (tl_id_and_point_) {
        break;
      }
    }
  }
  if (!tl_id_and_point_) {
    // this lane has no traffic light
    return;
  }
  const auto [tl_id, point] = tl_id_and_point_.value();
  const auto tl_info_opt =
    planner_data.getTrafficSignal(tl_id, true /* traffic light module keeps last observation*/);
  if (!tl_info_opt) {
    // the info of this traffic light is not available
    return;
  }
  last_tl_valid_observation_ = tl_info_opt.value();
  internal_debug_data_.tl_observation = tl_info_opt.value();
}

IntersectionModule::PassJudgeStatus IntersectionModule::isOverPassJudgeLinesStatus(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const bool is_occlusion_state,
  const IntersectionStopLines & intersection_stoplines, const PlannerData & planner_data)
{
  const auto & current_pose = planner_data.current_odometry->pose;
  const auto closest_idx = intersection_stoplines.closest_idx;
  const auto original_pass_judge_line_idx = intersection_stoplines.pass_judge_line;
  const auto occlusion_stopline_idx = intersection_stoplines.occlusion_peeking_stopline.value();
  const size_t pass_judge_line_idx = [&]() {
    if (planner_param_.occlusion.enable) {
      // ==========================================================================================
      // if ego passed the original_pass_judge_line while it is peeking to occlusion, then its
      // position is changed to occlusion_stopline_idx, because otherwise peeking is terminated.
      // even if occlusion is cleared by peeking, its position should be occlusion_stopline_idx as
      // before
      // ==========================================================================================
      if (passed_judge_line_while_peeking_) {
        return occlusion_stopline_idx;
      }
      const bool is_over_original_pass_judge_line =
        util::isOverTargetIndex(path, closest_idx, current_pose, original_pass_judge_line_idx);
      if (is_occlusion_state && is_over_original_pass_judge_line) {
        passed_judge_line_while_peeking_ = true;
        return occlusion_stopline_idx;
      }
      // ==========================================================================================
      // Otherwise it is original_pass_judge_line
      // ==========================================================================================
      return original_pass_judge_line_idx;
    }
    return original_pass_judge_line_idx;
  }();

  // ==========================================================================================
  // at intersection without traffic light, this module ignores occlusion even if occlusion is
  // detected for real, so if collision is not detected in that context, that should be interpreted
  // as "was_safe"
  // ==========================================================================================
  const bool was_safe = [&]() {
    if (std::holds_alternative<Safe>(prev_decision_result_)) {
      return true;
    }
    if (std::holds_alternative<FullyPrioritized>(prev_decision_result_)) {
      const auto & prev_decision = std::get<FullyPrioritized>(prev_decision_result_);
      return !prev_decision.collision_detected;
    }
    if (std::holds_alternative<OccludedAbsenceTrafficLight>(prev_decision_result_)) {
      const auto & state = std::get<OccludedAbsenceTrafficLight>(prev_decision_result_);
      return !state.collision_detected;
    }
    return false;
  }();

  const bool is_over_pass_judge_line =
    util::isOverTargetIndex(path, closest_idx, current_pose, pass_judge_line_idx);
  bool safely_passed_judge_line_first_time = false;
  if (is_over_pass_judge_line && was_safe && !safely_passed_judge_line_time_) {
    safely_passed_judge_line_time_ = std::make_pair(clock_->now(), current_pose);
    safely_passed_judge_line_first_time = true;
  }
  const double baselink2front = planner_data.vehicle_info_.max_longitudinal_offset_m;
  debug_data_.pass_judge_wall_pose =
    planning_utils::getAheadPose(pass_judge_line_idx, baselink2front, path);
  debug_data_.passed_pass_judge = safely_passed_judge_line_time_.has_value();

  if ((is_over_pass_judge_line && was_safe) || is_permanent_go_) {
    // ==========================================================================================
    // this body is active if ego is
    // - over the default stopline AND
    // - over the pass judge line
    // - previously safe
    // ,
    // which means ego can stop even if it is over the 1st pass judge line but
    // - before default stopline OR
    // - or previously unsafe
    // .
    //
    // in order for ego to continue peeking or collision detection when occlusion is detected
    // after ego passed the 1st pass judge line, it needs to be
    // - before the default stopline OR
    // - previously unsafe
    // ==========================================================================================
    is_permanent_go_ = true;
  }
  return {is_over_pass_judge_line, safely_passed_judge_line_first_time};
}

}  // namespace autoware::behavior_velocity_planner
