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

#include "scene_roundabout.hpp"

#include <autoware/behavior_velocity_intersection_module/util.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>  // for toGeomPoly
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
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

RoundaboutModule::RoundaboutModule(
  const int64_t module_id, std::shared_ptr<const lanelet::autoware::Roundabout> roundabout,
  const int64_t lane_id, [[maybe_unused]] std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const std::set<lanelet::Id> & associative_ids,
  rclcpp::Node & node, const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterfaceWithRTC(module_id, logger, clock, time_keeper, planning_factor_interface),
  roundabout_reg_elem_(roundabout),
  planner_param_(planner_param),
  lane_id_(lane_id),
  associative_ids_(associative_ids)
{
  collision_state_machine_.setMarginTime(
    planner_param_.collision_detection.collision_detection_hold_time);
  ego_ttc_pub_ = node.create_publisher<autoware_internal_debug_msgs::msg::Float64MultiArrayStamped>(
    "~/debug/roundabout/ego_ttc", 1);
  object_ttc_pub_ =
    node.create_publisher<autoware_internal_debug_msgs::msg::Float64MultiArrayStamped>(
      "~/debug/roundabout/object_ttc", 1);
}

bool RoundaboutModule::modifyPathVelocity(PathWithLaneId * path)
{
  debug_data_ = DebugData();

  initializeRTCStatus();

  const auto decision_result = modifyPathVelocityDetail(path);
  prev_decision_result_ = decision_result;

  {
    const std::string decision_type = "roundabout" + std::to_string(module_id_) + " : " +
                                      formatDecisionResult(decision_result, activated_);
    internal_debug_data_.decision_type = decision_type;
  }

  prepareRTCStatus(decision_result, *path);

  reactRTCApproval(decision_result, path);

  return true;
}

void RoundaboutModule::initializeRTCStatus()
{
  setSafe(true);
  setDistance(std::numeric_limits<double>::lowest());
}

DecisionResult RoundaboutModule::modifyPathVelocityDetail(PathWithLaneId * path)
{
  const auto prepare_data = prepareRoundaboutData(path);
  if (!prepare_data) {
    return prepare_data.err();
  }
  const auto [interpolated_path_info, roundabout_stoplines, path_lanelets] = prepare_data.ok();
  const auto & roundabout_lanelets = roundabout_lanelets_.value();

  const auto closest_idx = roundabout_stoplines.closest_idx;
  const auto can_smoothly_stop_at = [&](const auto & stop_line_idx) {
    const double max_accel = planner_data_->max_stop_acceleration_threshold;
    const double max_jerk = planner_data_->max_stop_jerk_threshold;
    const double delay_response_time = planner_data_->delay_response_time;
    const double velocity = planner_data_->current_velocity->twist.linear.x;
    const double acceleration = planner_data_->current_acceleration->accel.accel.linear.x;
    const double braking_dist = planning_utils::calcJudgeLineDistWithJerkLimit(
      velocity, acceleration, max_accel, max_jerk, delay_response_time);
    return autoware::motion_utils::calcSignedArcLength(path->points, closest_idx, stop_line_idx) >
           braking_dist;
  };

  // ==========================================================================================
  // basic data validation
  //
  // if attention area is empty, collision detection is impossible
  //
  // if attention area is not null but default stop line is not available, ego/backward-path has
  // already passed the stop line, so ego is already in the middle of the roundabout, or the
  // end of the ego path has just entered the entry of this roundabout
  // ==========================================================================================
  if (!roundabout_lanelets.first_attention_area()) {
    return InternalError{"attention area is empty"};
  }

  const auto default_stopline_idx_opt = roundabout_stoplines.default_stopline;
  if (!default_stopline_idx_opt) {
    return InternalError{"default stop line is null"};
  }
  const auto default_stopline_idx = default_stopline_idx_opt.value();
  const auto first_attention_stopline_idx_opt = roundabout_stoplines.first_attention_stopline;
  if (!first_attention_stopline_idx_opt) {
    return InternalError{"first attention stop line is null"};
  }

  // ==========================================================================================
  // classify the objects to attention_area/roundabout_area and update their position, velocity,
  // belonging attention lanelet, distance to corresponding stopline
  // ==========================================================================================
  updateObjectInfoManagerArea();

  const auto [is_over_1st_pass_judge_line, safely_passed_1st_judge_line] =
    isOverPassJudgeLinesStatus(*path, roundabout_stoplines);

  // ==========================================================================================
  // calculate the expected vehicle speed and obtain the spatiotemporal profile of ego to the
  // exit of roundabout
  // ==========================================================================================
  autoware_internal_debug_msgs::msg::Float64MultiArrayStamped ego_ttc_time_array;
  const auto time_distance_array =
    calcRoundaboutPassingTime(*path, roundabout_stoplines, &ego_ttc_time_array);

  // ==========================================================================================
  // run collision checking for each objects. Also if ego just
  // passed each pass judge line for the first time, save current collision status for late
  // diagnosis
  // ==========================================================================================
  autoware_internal_debug_msgs::msg::Float64MultiArrayStamped object_ttc_time_array;
  updateObjectInfoManagerCollision(
    path_lanelets, time_distance_array, safely_passed_1st_judge_line, &object_ttc_time_array);
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

  const auto [has_collision, collision_position, too_late_detect_objects, misjudge_objects] =
    detectCollision(is_over_1st_pass_judge_line);
  collision_state_machine_.setStateWithMarginTime(
    has_collision ? StateMachine::State::STOP : StateMachine::State::GO,
    logger_.get_child("collision state_machine"), *clock_);
  const bool has_collision_with_margin =
    collision_state_machine_.getState() == StateMachine::State::STOP;
  const std::string safety_diag =
    generateDetectionBlameDiagnosis(too_late_detect_objects, misjudge_objects);

  if (is_permanent_go_) {
    if (has_collision_with_margin) {
      if (can_smoothly_stop_at(roundabout_stoplines.first_attention_stopline.value())) {
        // NOTE: in this case, ego tries to stop at current position
        const auto stop_line_idx = closest_idx;
        return CollisionStop{closest_idx, stop_line_idx};
      }
    }
    if (has_collision) {
      const std::string evasive_diag = generateEgoRiskEvasiveDiagnosis(
        *path, closest_idx, time_distance_array, too_late_detect_objects, misjudge_objects);
      debug_data_.too_late_stop_wall_pose = path->points.at(default_stopline_idx).point.pose;
      return OverPassJudge{safety_diag, evasive_diag};
    }
    return OverPassJudge{
      "no collision is detected", "ego can safely pass the roundabout at this rate"};
  }

  if (!has_collision_with_margin) {
    return Safe{closest_idx, default_stopline_idx};
  }
  return CollisionStop{closest_idx, default_stopline_idx};
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
  [[maybe_unused]] bool * default_safety, [[maybe_unused]] double * default_distance)
{
  static_assert("Unsupported type passed to prepareRTCByDecisionResult");
  return;
}

template <>
void prepareRTCByDecisionResult(
  [[maybe_unused]] const InternalError & result,
  [[maybe_unused]] const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  [[maybe_unused]] bool * default_safety, [[maybe_unused]] double * default_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "InternalError");
  *default_safety = false;
  return;
}

template <>
void prepareRTCByDecisionResult(
  [[maybe_unused]] const OverPassJudge & result,
  [[maybe_unused]] const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  [[maybe_unused]] bool * default_safety, [[maybe_unused]] double * default_distance)
{
  return;
}

template <>
void prepareRTCByDecisionResult(
  const CollisionStop & result, const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  bool * default_safety, double * default_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "CollisionStop");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  *default_safety = false;
  *default_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const Safe & result, const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  bool * default_safety, double * default_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "Safe");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  *default_safety = true;
  *default_distance =
    autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  return;
}

void RoundaboutModule::prepareRTCStatus(
  const DecisionResult & decision_result,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  bool default_safety = true;
  double default_distance = std::numeric_limits<double>::lowest();
  std::visit(
    VisitorSwitch{[&](const auto & decision) {
      prepareRTCByDecisionResult(decision, path, &default_safety, &default_distance);
    }},
    decision_result);
  setSafe(default_safety);
  setDistance(default_distance);
}

template <typename T>
void reactRTCApprovalByDecisionResult(
  [[maybe_unused]] const bool rtc_default_approved, [[maybe_unused]] const T & decision_result,
  [[maybe_unused]] const RoundaboutModule::PlannerParam & planner_param,
  [[maybe_unused]] const double baselink2front,
  [[maybe_unused]] autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  [[maybe_unused]] planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  [[maybe_unused]] RoundaboutModule::DebugData * debug_data)
{
  static_assert("Unsupported type passed to reactRTCByDecisionResult");
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  [[maybe_unused]] const bool rtc_default_approved,
  [[maybe_unused]] const InternalError & decision_result,
  [[maybe_unused]] const RoundaboutModule::PlannerParam & planner_param,
  [[maybe_unused]] const double baselink2front,
  [[maybe_unused]] autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  [[maybe_unused]] planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  [[maybe_unused]] RoundaboutModule::DebugData * debug_data)
{
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  [[maybe_unused]] const bool rtc_default_approved,
  [[maybe_unused]] const OverPassJudge & decision_result,
  [[maybe_unused]] const RoundaboutModule::PlannerParam & planner_param,
  [[maybe_unused]] const double baselink2front,
  [[maybe_unused]] autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  [[maybe_unused]] planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  [[maybe_unused]] RoundaboutModule::DebugData * debug_data)
{
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const CollisionStop & decision_result,
  [[maybe_unused]] const RoundaboutModule::PlannerParam & planner_param,
  const double baselink2front, autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  RoundaboutModule::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "CollisionStop, approval = (default: %d)", rtc_default_approved);
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
        true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/, "collision stop");
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const Safe & decision_result,
  [[maybe_unused]] const RoundaboutModule::PlannerParam & planner_param,
  const double baselink2front, autoware_internal_planning_msgs::msg::PathWithLaneId * path,
  autoware_internal_planning_msgs::msg::SafetyFactorArray & safety_factor_array,
  planning_factor_interface::PlanningFactorInterface * planning_factor_interface,
  RoundaboutModule::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"), "Safe, approval = (default: %d)",
    rtc_default_approved);
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
        true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/, "");
    }
  }
  return;
}

void RoundaboutModule::reactRTCApproval(
  const DecisionResult & decision_result,
  autoware_internal_planning_msgs::msg::PathWithLaneId * path)
{
  const double baselink2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  std::visit(
    VisitorSwitch{[&](const auto & decision) {
      reactRTCApprovalByDecisionResult(
        activated_, decision, planner_param_, baselink2front, path, safety_factor_array_,
        planning_factor_interface_.get(), &debug_data_);
    }},
    decision_result);
  return;
}

RoundaboutModule::PassJudgeStatus RoundaboutModule::isOverPassJudgeLinesStatus(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const RoundaboutStopLines & roundabout_stoplines)
{
  const auto & current_pose = planner_data_->current_odometry->pose;
  const auto closest_idx = roundabout_stoplines.closest_idx;
  const auto default_stopline_idx = roundabout_stoplines.default_stopline.value();
  const size_t pass_judge_line_idx = roundabout_stoplines.first_pass_judge_line;

  const bool was_safe = std::holds_alternative<Safe>(prev_decision_result_);

  const bool is_over_1st_pass_judge_line =
    util::isOverTargetIndex(path, closest_idx, current_pose, pass_judge_line_idx);
  bool safely_passed_1st_judge_line_first_time = false;
  if (is_over_1st_pass_judge_line && was_safe && !safely_passed_1st_judge_line_time_) {
    safely_passed_1st_judge_line_time_ = std::make_pair(clock_->now(), current_pose);
    safely_passed_1st_judge_line_first_time = true;
  }
  const double baselink2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  debug_data_.first_pass_judge_wall_pose =
    planning_utils::getAheadPose(pass_judge_line_idx, baselink2front, path);
  debug_data_.passed_first_pass_judge = safely_passed_1st_judge_line_time_.has_value();

  const bool is_over_default_stopline =
    util::isOverTargetIndex(path, closest_idx, current_pose, default_stopline_idx);

  const bool over_default_stopline_for_pass_judge =
    is_over_default_stopline || planner_param_.common.enable_pass_judge_before_default_stopline;
  if (
    (over_default_stopline_for_pass_judge && is_over_1st_pass_judge_line && was_safe) ||
    is_permanent_go_) {
    // ==========================================================================================
    // this body is active if ego is
    // - over the default stopline AND
    // - over the 1st pass judge line AND
    // - previously safe
    // ,
    // which means ego can stop even if it is over the 1st pass judge line but
    // - before default stopline OR
    // - or previously unsafe
    // .
    // ==========================================================================================
    is_permanent_go_ = true;
  }
  return {is_over_1st_pass_judge_line, safely_passed_1st_judge_line_first_time};
}

}  // namespace autoware::behavior_velocity_planner
