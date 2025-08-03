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

#include "autoware/behavior_velocity_blind_spot_module/scene.hpp"

#include "autoware/behavior_velocity_blind_spot_module/time_to_collision.hpp"
#include "autoware/behavior_velocity_blind_spot_module/util.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/object_recognition_utils/predicted_path_utils.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <std_msgs/msg/string.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace bg = boost::geometry;

std::string format_blind_spot_decision(const BlindSpotDecision & decision, const lanelet::Id id)
{
  std::stringstream ss;
  ss << "lane_id: " << id << std::endl;
  if (std::holds_alternative<InternalError>(decision)) {
    const auto & err = std::get<InternalError>(decision);
    ss << "InternalError: " << err.error;
  }
  if (std::holds_alternative<OverPassJudge>(decision)) {
    const auto & state = std::get<OverPassJudge>(decision);
    ss << "OverPassJudge: " << state.report;
  }
  if (std::holds_alternative<Unsafe>(decision)) {
    ss << "Unsafe";
  }
  if (std::holds_alternative<Safe>(decision)) {
    ss << "Safe";
  }
  return ss.str();
}

BlindSpotModule::BlindSpotModule(
  const int64_t module_id, const int64_t lane_id, const TurnDirection turn_direction,
  [[maybe_unused]] const std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface,
  const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_state_pub)
: SceneModuleInterfaceWithRTC(module_id, logger, clock, time_keeper, planning_factor_interface),
  lane_id_(lane_id),
  planner_param_{planner_param},
  turn_direction_(turn_direction),
  decision_state_pub_(decision_state_pub)
{
  state_machine_.setMarginTime(planner_param_.collision_judge_debounce);
}

void BlindSpotModule::initializeRTCStatus()
{
  setSafe(true);
  setDistance(std::numeric_limits<double>::lowest());
}

BlindSpotDecision BlindSpotModule::modifyPathVelocityDetail(PathWithLaneId * path)
{
  if (is_over_pass_judge_line_) {
    return OverPassJudge{"already over the pass judge line for conflict_area"};
  }
  const auto & input_path = *path;

  /* set stop-line and stop-judgement-line for base_link */
  const auto interpolated_path_info_opt =
    generateInterpolatedPathInfo(lane_id_, input_path, logger_);
  if (!interpolated_path_info_opt) {
    return InternalError{"failed to interpolate path"};
  }

  const auto & interpolated_path_info = interpolated_path_info_opt.value();
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id_);

  const auto lane_ids_upto_intersection = find_lane_ids_upto(*path, lane_id_);
  if (!blind_side_lanelets_before_turning_ || !road_lanelets_before_turning_merged_) {
    const auto road_and_blind_lanelets_opt = generate_blind_side_lanelets_before_turning(
      planner_data_->route_handler_, turn_direction_, planner_param_.backward_attention_length,
      lane_ids_upto_intersection, lane_id_);
    if (road_and_blind_lanelets_opt) {
      const auto & [road_lanelets, blind_side_lanelets_before_turning] =
        road_and_blind_lanelets_opt.value();
      road_lanelets_before_turning_merged_ = lanelet::utils::combineLaneletsShape(road_lanelets);
      blind_side_lanelets_before_turning_ = blind_side_lanelets_before_turning;
    }
  }
  if (!blind_side_lanelets_before_turning_ || !road_lanelets_before_turning_merged_) {
    return InternalError{"there are no lanelets before turning"};
  }

  const auto & road_lanelets_before_turning_merged = road_lanelets_before_turning_merged_.value();
  const auto & blind_spot_lanelets_before_turning = blind_side_lanelets_before_turning_.value();
  const auto & last_blind_spot_lanelet_before_turning = blind_spot_lanelets_before_turning.back();
  if (!virtual_blind_lane_boundary_after_turning_) {
    virtual_blind_lane_boundary_after_turning_ = generate_virtual_blind_side_boundary_after_turning(
      last_blind_spot_lanelet_before_turning, turn_direction_,
      lanelet::utils::getLaneletLength3d(assigned_lanelet));
  }
  const auto & virtual_blind_lane_boundary_after_turning =
    virtual_blind_lane_boundary_after_turning_.value();
  debug_data_.virtual_blind_lane_boundary_after_turning = virtual_blind_lane_boundary_after_turning;

  const auto ego_width = planner_data_->vehicle_info_.vehicle_width_m;

  // NOTE: this scale is to avoid regarding ego vehicle which is stopping at stopline touches the
  // virtual_ego_straight_path_after_turning
  static constexpr double ego_width_scale = 1.1;
  const auto virtual_ego_straight_path_after_turning_opt =
    generate_virtual_ego_straight_path_after_turning(
      assigned_lanelet, routing_graph_ptr, *path, turn_direction_, ego_width * ego_width_scale);
  if (!virtual_ego_straight_path_after_turning_opt) {
    return InternalError{"failed to generate virtual_ego_straight_path_after_turning"};
  }

  const auto & virtual_ego_straight_path_after_turning =
    virtual_ego_straight_path_after_turning_opt.value();
  debug_data_.virtual_ego_straight_path_after_turning = virtual_ego_straight_path_after_turning;

  const auto attention_area_opt = generate_attention_area(
    road_lanelets_before_turning_merged, blind_spot_lanelets_before_turning,
    virtual_blind_lane_boundary_after_turning, virtual_ego_straight_path_after_turning,
    assigned_lanelet, *path, turn_direction_, ego_width);
  if (!attention_area_opt) {
    return InternalError{"failed to generate attention_area"};
  }

  const auto & attention_area = attention_area_opt.value();
  debug_data_.attention_area = attention_area;
  const auto ego_intersection_path_lanelet_opt =
    generate_ego_path_polygon(interpolated_path_info, ego_width);
  if (!ego_intersection_path_lanelet_opt) {
    return InternalError{"failed to generate path_polygon"};
  }

  const auto & ego_intersection_path_lanelet = ego_intersection_path_lanelet_opt.value();
  debug_data_.path_polygon = ego_intersection_path_lanelet.polygon3d();

  const double time_to_restart =
    activated_ ? 0.0 : (planner_param_.collision_judge_debounce - state_machine_.getDuration());
  const auto ego_future_profile = calculate_future_profile(
    *path, planner_param_.minimum_default_velocity, time_to_restart, planner_data_, lane_id_);
  if (ego_future_profile.empty()) {
    return InternalError{"failed to compute ego predicted trajectory"};
  }

  const auto & first_line = (turn_direction_ == TurnDirection::Left)
                              ? virtual_ego_straight_path_after_turning
                              : virtual_blind_lane_boundary_after_turning;
  const auto & second_line = (turn_direction_ == TurnDirection::Left)
                               ? virtual_blind_lane_boundary_after_turning
                               : virtual_ego_straight_path_after_turning;
  const auto ego_passage_time_interval_opt = compute_time_interval_for_passing_line(
    ego_future_profile, planner_data_->vehicle_info_.createFootprint(0.0, 0.0), first_line,
    second_line);
  if (!ego_passage_time_interval_opt) {
    return InternalError{"failed to compute time interval for ego passing conflict area"};
  }

  const auto & ego_passage_time_interval = ego_passage_time_interval_opt.value();
  debug_data_.ego_passage_interval = ego_passage_time_interval;

  const auto attention_objects =
    filter_attention_objects(lanelet::utils::to2D(attention_area).basicPolygon());

  const auto unsafe_objects = collect_unsafe_objects(
    attention_objects, ego_intersection_path_lanelet, ego_passage_time_interval);
  debug_data_.unsafe_objects.emplace(unsafe_objects);
  const bool is_safe_now = unsafe_objects.empty();

  // provide hold time for STOP -> GO
  state_machine_.setStateWithMarginTime(
    !is_safe_now ? StateMachine::State::STOP : StateMachine::State::GO,
    logger_.get_child("collision state_machine"), *clock_);
  const bool is_safe = (state_machine_.getState() == StateMachine::State::GO);

  const double delay_response_time = planner_data_->delay_response_time;
  const double velocity = planner_data_->current_velocity->twist.linear.x;
  const double acceleration = planner_data_->current_acceleration->accel.accel.linear.x;

  const double critical_braking_distance = planning_utils::calcJudgeLineDistWithJerkLimit(
    velocity, acceleration, planner_param_.brake.critical.deceleration,
    planner_param_.brake.critical.jerk, delay_response_time);

  const auto stop_points_opt = generate_stop_points(
    interpolated_path_info, planner_data_->vehicle_info_.createFootprint(0.0, 0.0),
    planner_data_->vehicle_info_.vehicle_length_m, assigned_lanelet,
    virtual_ego_straight_path_after_turning, planner_data_->current_odometry->pose,
    critical_braking_distance, planner_param_.critical_stopline_margin,
    planner_data_->ego_nearest_dist_threshold, planner_data_->ego_nearest_yaw_threshold, path);
  if (!stop_points_opt) {
    return InternalError{"failed to generate stop points"};
  }

  const auto & stop_points = stop_points_opt.value();
  const auto default_stopline = stop_points.default_stopline;
  const auto instant_stopline = stop_points.instant_stopline;
  const auto critical_stopline = stop_points.critical_stopline;

  const auto closest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    path->points, planner_data_->current_odometry->pose, planner_data_->ego_nearest_dist_threshold,
    planner_data_->ego_nearest_yaw_threshold);
  auto can_smoothly_stop_at =
    [&](const auto & stop_line_idx, const double deceleration, const double jerk_for_deceleration) {
      const double braking_dist = planning_utils::calcJudgeLineDistWithJerkLimit(
        velocity, acceleration, deceleration, jerk_for_deceleration, delay_response_time);
      return autoware::motion_utils::calcSignedArcLength(path->points, closest_idx, stop_line_idx) >
             braking_dist;
    };

  if (is_safe) {
    // NOTE: over_pass_judge judgement is only allowed when Safe is ensured
    if (!can_smoothly_stop_at(
          critical_stopline, planner_param_.brake.critical.deceleration,
          planner_param_.brake.critical.jerk)) {
      is_over_pass_judge_line_ = true;
    }
    return Safe{default_stopline.value_or(critical_stopline)};
  }

  if (is_safe_now) {
    return Unsafe{default_stopline.value_or(critical_stopline)};
  }

  const auto & most_unsafe_object = unsafe_objects.front();
  const auto most_nearest_collision_time = most_unsafe_object.critical_time;
  debug_data_.critical_time = most_nearest_collision_time;

  const auto [deceleration, jerk] = compute_decel_and_jerk_from_ttc(most_nearest_collision_time);

  autoware_internal_planning_msgs::msg::SafetyFactorArray safety_factor;
  safety_factor.factors.push_back(most_unsafe_object.to_safety_factor());
  // 1st, try to stop at the traffic light stopline smoothly
  if (default_stopline && can_smoothly_stop_at(default_stopline.value(), deceleration, jerk)) {
    planning_factor_interface_->add(
      path->points, planner_data_->current_odometry->pose,
      path->points.at(default_stopline.value()).point.pose,
      autoware_internal_planning_msgs::msg::PlanningFactor::STOP, safety_factor,
      true /*is_driving_forward*/, 0.0 /* speed */, 0.0 /*shift distance*/, "");
    return Unsafe{default_stopline.value()};
  }

  // otherwise, try to stop at instant_stopline before critical_stopline. if it is impossible, ego
  // would stop inside the conflict_area in vain
  if (
    instant_stopline <= critical_stopline &&
    can_smoothly_stop_at(critical_stopline, deceleration, jerk)) {
    planning_factor_interface_->add(
      path->points, planner_data_->current_odometry->pose,
      path->points.at(instant_stopline).point.pose,
      autoware_internal_planning_msgs::msg::PlanningFactor::STOP, safety_factor,
      true /*is_driving_forward*/, 0.0 /* speed */, 0.0 /*shift distance*/, "");
    return Unsafe{instant_stopline};
  }

  return OverPassJudge{"the situation is unsafe, but too late to stop before conflicting_area"};
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

void BlindSpotModule::setRTCStatus(
  const BlindSpotDecision & decision,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  std::visit(
    VisitorSwitch{[&](const auto & sub_decision) { setRTCStatusByDecision(sub_decision, path); }},
    decision);
}

void BlindSpotModule::reactRTCApproval(const BlindSpotDecision & decision, PathWithLaneId * path)
{
  std::visit(
    VisitorSwitch{
      [&](const auto & sub_decision) { reactRTCApprovalByDecision(sub_decision, path); }},
    decision);
}

bool BlindSpotModule::modifyPathVelocity(PathWithLaneId * path)
{
  debug_data_ = DebugData();

  initializeRTCStatus();
  const auto decision = modifyPathVelocityDetail(path);
  {
    std_msgs::msg::String msg;
    msg.data = format_blind_spot_decision(decision, lane_id_);
    decision_state_pub_->publish(msg);
  }

  const auto & input_path = *path;
  setRTCStatus(decision, input_path);
  reactRTCApproval(decision, path);

  return true;
}

std::vector<UnsafeObject> BlindSpotModule::collect_unsafe_objects(
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & attention_objects,
  const lanelet::ConstLanelet & ego_path_lanelet,
  const std::pair<double, double> & ego_passage_time_interval) const
{
  const auto & first_line = (turn_direction_ == TurnDirection::Left)
                              ? ego_path_lanelet.leftBound()
                              : ego_path_lanelet.rightBound();
  const auto & second_line = (turn_direction_ == TurnDirection::Left)
                               ? ego_path_lanelet.rightBound()
                               : ego_path_lanelet.leftBound();
  const auto entry_line = get_entry_line(ego_path_lanelet);

  std::vector<UnsafeObject> unsafe_objects;
  for (const auto & attention_object : attention_objects) {
    const auto object_passage_intervals =
      compute_time_interval_for_passing_line(attention_object, first_line, entry_line, second_line);
    for (const auto & object_passage_interval : object_passage_intervals) {
      const auto & [object_entry, object_exit, predicted_path] = object_passage_interval;
      if (const auto collision_time = get_unsafe_time_if_critical(
            ego_passage_time_interval, {object_entry, object_exit}, planner_param_.ttc_start_margin,
            planner_param_.ttc_end_margin);
          collision_time) {
        unsafe_objects.emplace_back(
          attention_object, collision_time.value(), predicted_path,
          std::make_pair(object_entry, object_exit));
      }
    }
  }
  // sort the objects so that front() has nearest critical time
  std::sort(unsafe_objects.begin(), unsafe_objects.end(), [](const auto & a, const auto & b) {
    return a.critical_time < b.critical_time;
  });
  return unsafe_objects;
}

std::vector<autoware_perception_msgs::msg::PredictedObject>
BlindSpotModule::filter_attention_objects(const lanelet::BasicPolygon2d & attention_area) const
{
  std::vector<autoware_perception_msgs::msg::PredictedObject> result;
  for (const auto & object : planner_data_->predicted_objects->objects) {
    if (!isTargetObjectType(object)) {
      continue;
    }
    const auto & position = object.kinematics.initial_pose_with_covariance.pose.position;
    // NOTE: use position of the object because vru object polygon around blind_spot is unstable
    if (boost::geometry::within(
          autoware_utils_geometry::Point2d{position.x, position.y}, attention_area)) {
      result.push_back(object);
    }
  }
  return result;
}

bool BlindSpotModule::isTargetObjectType(
  const autoware_perception_msgs::msg::PredictedObject & object) const
{
  if (
    object.classification.at(0).label == autoware_perception_msgs::msg::ObjectClassification::CAR ||
    object.classification.at(0).label ==
      autoware_perception_msgs::msg::ObjectClassification::TRUCK ||
    object.classification.at(0).label ==
      autoware_perception_msgs::msg::ObjectClassification::TRAILER ||
    object.classification.at(0).label ==
      autoware_perception_msgs::msg::ObjectClassification::BICYCLE ||
    object.classification.at(0).label ==
      autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN ||
    object.classification.at(0).label ==
      autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE) {
    return true;
  }
  return false;
}

std::pair<double, double> BlindSpotModule::compute_decel_and_jerk_from_ttc(const double ttc) const
{
  const auto & critical_decel = planner_param_.brake.critical.deceleration;
  const auto & critical_jerk = planner_param_.brake.critical.jerk;
  const auto & semi_critical_decel = planner_param_.brake.semi_critical.deceleration;
  const auto & semi_critical_jerk = planner_param_.brake.semi_critical.jerk;

  if (ttc < planner_param_.brake_for_ttc.critical_threshold_ub) {
    return {critical_decel, critical_jerk};
  }

  if (ttc > planner_param_.brake_for_ttc.semi_critical_threshold_lb) {
    return {semi_critical_decel, semi_critical_jerk};
  }

  const double a = ttc - planner_param_.brake_for_ttc.critical_threshold_ub;
  const double b = planner_param_.brake_for_ttc.semi_critical_threshold_lb;
  const auto lerp = [&](const double x, const double y) { return (b * x + a * y) / (a + b); };

  return {lerp(critical_decel, semi_critical_decel), lerp(critical_jerk, semi_critical_jerk)};
}

}  // namespace autoware::behavior_velocity_planner
