// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_lane_change_module/manager.hpp"

#include "autoware/behavior_path_lane_change_module/interface.hpp"
#include "autoware_utils/ros/parameter.hpp"
#include "autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

void LaneChangeModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {""});
  initParams(node);
}

LCParamPtr LaneChangeModuleManager::set_params(rclcpp::Node * node, const std::string & node_name)
{
  using autoware_utils::get_or_declare_parameter;

  LaneChangeParameters p{};

  const auto parameter = [](std::string && name) { return "lane_change." + name; };

  // trajectory generation
  {
    p.trajectory.max_prepare_duration =
      get_or_declare_parameter<double>(*node, parameter("trajectory.max_prepare_duration"));
    p.trajectory.min_prepare_duration =
      get_or_declare_parameter<double>(*node, parameter("trajectory.min_prepare_duration"));
    p.trajectory.lateral_jerk =
      get_or_declare_parameter<double>(*node, parameter("trajectory.lateral_jerk"));
    p.trajectory.min_longitudinal_acc =
      get_or_declare_parameter<double>(*node, parameter("trajectory.min_longitudinal_acc"));
    p.trajectory.max_longitudinal_acc =
      get_or_declare_parameter<double>(*node, parameter("trajectory.max_longitudinal_acc"));
    p.trajectory.th_prepare_length_diff =
      get_or_declare_parameter<double>(*node, parameter("trajectory.th_prepare_length_diff"));
    p.trajectory.th_lane_changing_length_diff =
      get_or_declare_parameter<double>(*node, parameter("trajectory.th_lane_changing_length_diff"));
    p.trajectory.min_lane_changing_velocity =
      get_or_declare_parameter<double>(*node, parameter("trajectory.min_lane_changing_velocity"));
    p.trajectory.lane_changing_decel_factor =
      get_or_declare_parameter<double>(*node, parameter("trajectory.lane_changing_decel_factor"));
    p.trajectory.th_prepare_curvature =
      get_or_declare_parameter<double>(*node, parameter("trajectory.th_prepare_curvature"));
    p.trajectory.lon_acc_sampling_num =
      get_or_declare_parameter<int>(*node, parameter("trajectory.lon_acc_sampling_num"));
    p.trajectory.lat_acc_sampling_num =
      get_or_declare_parameter<int>(*node, parameter("trajectory.lat_acc_sampling_num"));

    const auto max_acc = get_or_declare_parameter<double>(*node, "normal.max_acc");
    p.trajectory.min_lane_changing_velocity = std::min(
      p.trajectory.min_lane_changing_velocity, max_acc * p.trajectory.max_prepare_duration);

    // validation of trajectory parameters
    if (p.trajectory.lon_acc_sampling_num < 1 || p.trajectory.lat_acc_sampling_num < 1) {
      RCLCPP_FATAL_STREAM(
        node->get_logger().get_child(node_name),
        "lane_change_sampling_num must be positive integer. Given longitudinal parameter: "
          << p.trajectory.lon_acc_sampling_num
          << "Given lateral parameter: " << p.trajectory.lat_acc_sampling_num << std::endl
          << "Terminating the program...");
      exit(EXIT_FAILURE);
    }

    // lateral acceleration map
    const auto lateral_acc_velocity = get_or_declare_parameter<std::vector<double>>(
      *node, parameter("lateral_acceleration.velocity"));
    const auto min_lateral_acc = get_or_declare_parameter<std::vector<double>>(
      *node, parameter("lateral_acceleration.min_values"));
    const auto max_lateral_acc = get_or_declare_parameter<std::vector<double>>(
      *node, parameter("lateral_acceleration.max_values"));
    if (
      lateral_acc_velocity.size() != min_lateral_acc.size() ||
      lateral_acc_velocity.size() != max_lateral_acc.size()) {
      RCLCPP_ERROR(
        node->get_logger().get_child(node_name),
        "Lane change lateral acceleration map has invalid size.");
      exit(EXIT_FAILURE);
    }
    for (size_t i = 0; i < lateral_acc_velocity.size(); ++i) {
      p.trajectory.lat_acc_map.add(
        lateral_acc_velocity.at(i), min_lateral_acc.at(i), max_lateral_acc.at(i));
    }
  }

  // turn signal
  p.min_length_for_turn_signal_activation =
    get_or_declare_parameter<double>(*node, parameter("min_length_for_turn_signal_activation"));

  // lane change regulations
  p.regulate_on_crosswalk =
    get_or_declare_parameter<bool>(*node, parameter("regulation.crosswalk"));
  p.regulate_on_intersection =
    get_or_declare_parameter<bool>(*node, parameter("regulation.intersection"));
  p.regulate_on_traffic_light =
    get_or_declare_parameter<bool>(*node, parameter("regulation.traffic_light"));

  // ego vehicle stuck detection
  p.th_stop_velocity =
    get_or_declare_parameter<double>(*node, parameter("stuck_detection.velocity"));
  p.th_stop_time = get_or_declare_parameter<double>(*node, parameter("stuck_detection.stop_time"));

  // safety
  {
    p.safety.enable_loose_check_for_cancel =
      get_or_declare_parameter<bool>(*node, parameter("safety_check.allow_loose_check_for_cancel"));
    p.safety.enable_target_lane_bound_check = get_or_declare_parameter<bool>(
      *node, parameter("safety_check.enable_target_lane_bound_check"));
    p.safety.th_stopped_object_velocity = get_or_declare_parameter<double>(
      *node, parameter("safety_check.stopped_object_velocity_threshold"));
    p.safety.lane_expansion_left_offset =
      get_or_declare_parameter<double>(*node, parameter("safety_check.lane_expansion.left_offset"));
    p.safety.lane_expansion_right_offset = get_or_declare_parameter<double>(
      *node, parameter("safety_check.lane_expansion.right_offset"));

    // collision check
    p.safety.collision_check.enable_for_prepare_phase_in_general_lanes =
      get_or_declare_parameter<bool>(
        *node, parameter("collision_check.enable_for_prepare_phase.general_lanes"));
    p.safety.collision_check.enable_for_prepare_phase_in_intersection =
      get_or_declare_parameter<bool>(
        *node, parameter("collision_check.enable_for_prepare_phase.intersection"));
    p.safety.collision_check.enable_for_prepare_phase_in_turns = get_or_declare_parameter<bool>(
      *node, parameter("collision_check.enable_for_prepare_phase.turns"));
    p.safety.collision_check.check_current_lane =
      get_or_declare_parameter<bool>(*node, parameter("collision_check.check_current_lanes"));
    p.safety.collision_check.check_other_lanes =
      get_or_declare_parameter<bool>(*node, parameter("collision_check.check_other_lanes"));
    p.safety.collision_check.use_all_predicted_paths =
      get_or_declare_parameter<bool>(*node, parameter("collision_check.use_all_predicted_paths"));
    p.safety.collision_check.prediction_time_resolution = get_or_declare_parameter<double>(
      *node, parameter("collision_check.prediction_time_resolution"));
    p.safety.collision_check.th_yaw_diff =
      get_or_declare_parameter<double>(*node, parameter("collision_check.yaw_diff_threshold"));
    p.safety.collision_check.th_incoming_object_yaw =
      get_or_declare_parameter<double>(*node, parameter("collision_check.th_incoming_object_yaw"));

    // rss check
    auto set_rss_params = [&](auto & params, const std::string & prefix) {
      params.longitudinal_distance_min_threshold = get_or_declare_parameter<double>(
        *node, parameter(prefix + ".longitudinal_distance_min_threshold"));
      params.longitudinal_velocity_delta_time = get_or_declare_parameter<double>(
        *node, parameter(prefix + ".longitudinal_velocity_delta_time"));
      params.front_vehicle_deceleration =
        get_or_declare_parameter<double>(*node, parameter(prefix + ".expected_front_deceleration"));
      params.rear_vehicle_deceleration =
        get_or_declare_parameter<double>(*node, parameter(prefix + ".expected_rear_deceleration"));
      params.rear_vehicle_reaction_time =
        get_or_declare_parameter<double>(*node, parameter(prefix + ".rear_vehicle_reaction_time"));
      params.rear_vehicle_safety_time_margin = get_or_declare_parameter<double>(
        *node, parameter(prefix + ".rear_vehicle_safety_time_margin"));
      params.lateral_distance_max_threshold = get_or_declare_parameter<double>(
        *node, parameter(prefix + ".lateral_distance_max_threshold"));
      params.extended_polygon_policy = get_or_declare_parameter<std::string>(
        *node, parameter(prefix + ".extended_polygon_policy"));
    };
    set_rss_params(p.safety.rss_params, "safety_check.execution");
    set_rss_params(p.safety.rss_params_for_parked, "safety_check.parked");
    set_rss_params(p.safety.rss_params_for_abort, "safety_check.cancel");
    set_rss_params(p.safety.rss_params_for_stuck, "safety_check.stuck");

    // target object types
    const std::string ns = "lane_change.target_object.";
    p.safety.target_object_types.check_car = get_or_declare_parameter<bool>(*node, ns + "car");
    p.safety.target_object_types.check_truck = get_or_declare_parameter<bool>(*node, ns + "truck");
    p.safety.target_object_types.check_bus = get_or_declare_parameter<bool>(*node, ns + "bus");
    p.safety.target_object_types.check_trailer =
      get_or_declare_parameter<bool>(*node, ns + "trailer");
    p.safety.target_object_types.check_unknown =
      get_or_declare_parameter<bool>(*node, ns + "unknown");
    p.safety.target_object_types.check_bicycle =
      get_or_declare_parameter<bool>(*node, ns + "bicycle");
    p.safety.target_object_types.check_motorcycle =
      get_or_declare_parameter<bool>(*node, ns + "motorcycle");
    p.safety.target_object_types.check_pedestrian =
      get_or_declare_parameter<bool>(*node, ns + "pedestrian");
  }

  // lane change parameters
  p.time_limit = get_or_declare_parameter<double>(*node, parameter("time_limit"));
  p.backward_lane_length =
    get_or_declare_parameter<double>(*node, parameter("backward_lane_length"));
  p.backward_length_buffer_for_end_of_lane =
    get_or_declare_parameter<double>(*node, parameter("backward_length_buffer_for_end_of_lane"));
  p.backward_length_buffer_for_blocking_object = get_or_declare_parameter<double>(
    *node, parameter("backward_length_buffer_for_blocking_object"));
  p.backward_length_from_intersection =
    get_or_declare_parameter<double>(*node, parameter("backward_length_from_intersection"));
  p.enable_stopped_vehicle_buffer =
    get_or_declare_parameter<bool>(*node, parameter("enable_stopped_vehicle_buffer"));

  if (p.backward_length_buffer_for_end_of_lane < 1.0) {
    RCLCPP_WARN_STREAM(
      node->get_logger().get_child(node_name),
      "Lane change buffer must be more than 1 meter. Modifying the buffer.");
  }

  // lane change delay
  p.delay.enable = get_or_declare_parameter<bool>(*node, parameter("delay_lane_change.enable"));
  p.delay.check_only_parked_vehicle =
    get_or_declare_parameter<bool>(*node, parameter("delay_lane_change.check_only_parked_vehicle"));
  p.delay.min_road_shoulder_width =
    get_or_declare_parameter<double>(*node, parameter("delay_lane_change.min_road_shoulder_width"));
  p.delay.th_parked_vehicle_shift_ratio = get_or_declare_parameter<double>(
    *node, parameter("delay_lane_change.th_parked_vehicle_shift_ratio"));

  // trajectory generation near terminal using frenet planner
  p.frenet.enable = get_or_declare_parameter<bool>(*node, parameter("frenet.enable"));
  p.frenet.use_entire_remaining_distance =
    get_or_declare_parameter<bool>(*node, parameter("frenet.use_entire_remaining_distance"));
  p.frenet.th_yaw_diff_deg =
    get_or_declare_parameter<double>(*node, parameter("frenet.th_yaw_diff"));
  p.frenet.th_curvature_smoothing =
    get_or_declare_parameter<double>(*node, parameter("frenet.th_curvature_smoothing"));
  p.frenet.th_average_curvature =
    get_or_declare_parameter<double>(*node, parameter("frenet.th_average_curvature"));

  // lane change cancel
  p.cancel.enable_on_prepare_phase =
    get_or_declare_parameter<bool>(*node, parameter("cancel.enable_on_prepare_phase"));
  p.cancel.enable_on_lane_changing_phase =
    get_or_declare_parameter<bool>(*node, parameter("cancel.enable_on_lane_changing_phase"));
  p.cancel.delta_time = get_or_declare_parameter<double>(*node, parameter("cancel.delta_time"));
  p.cancel.duration = get_or_declare_parameter<double>(*node, parameter("cancel.duration"));
  p.cancel.max_lateral_jerk =
    get_or_declare_parameter<double>(*node, parameter("cancel.max_lateral_jerk"));
  p.cancel.overhang_tolerance =
    get_or_declare_parameter<double>(*node, parameter("cancel.overhang_tolerance"));
  p.cancel.th_unsafe_hysteresis =
    get_or_declare_parameter<int>(*node, parameter("cancel.unsafe_hysteresis_threshold"));
  p.cancel.deceleration_sampling_num =
    get_or_declare_parameter<int>(*node, parameter("cancel.deceleration_sampling_num"));

  // finish judge parameters
  p.lane_change_finish_judge_buffer =
    get_or_declare_parameter<double>(*node, parameter("lane_change_finish_judge_buffer"));
  p.th_finish_judge_lateral_diff =
    get_or_declare_parameter<double>(*node, parameter("finish_judge_lateral_threshold"));
  const auto finish_judge_lateral_angle_deviation =
    get_or_declare_parameter<double>(*node, parameter("finish_judge_lateral_angle_deviation"));
  p.th_finish_judge_yaw_diff = autoware_utils::deg2rad(finish_judge_lateral_angle_deviation);

  // debug marker
  p.publish_debug_marker = get_or_declare_parameter<bool>(*node, parameter("publish_debug_marker"));

  // terminal lane change path
  p.terminal_path.enable = get_or_declare_parameter<bool>(*node, parameter("terminal_path.enable"));
  p.terminal_path.disable_near_goal =
    get_or_declare_parameter<bool>(*node, parameter("terminal_path.disable_near_goal"));
  p.terminal_path.stop_at_boundary =
    get_or_declare_parameter<bool>(*node, parameter("terminal_path.stop_at_boundary"));

  // validation of safety check parameters
  // if loose check is not enabled, lane change module will keep on chattering and canceling, and
  // false positive situation might  occur
  if (!p.safety.enable_loose_check_for_cancel) {
    if (
      p.safety.rss_params.front_vehicle_deceleration >
        p.safety.rss_params_for_abort.front_vehicle_deceleration ||
      p.safety.rss_params.rear_vehicle_deceleration >
        p.safety.rss_params_for_abort.rear_vehicle_deceleration ||
      p.safety.rss_params.rear_vehicle_reaction_time >
        p.safety.rss_params_for_abort.rear_vehicle_reaction_time ||
      p.safety.rss_params.rear_vehicle_safety_time_margin >
        p.safety.rss_params_for_abort.rear_vehicle_safety_time_margin ||
      p.safety.rss_params.lateral_distance_max_threshold >
        p.safety.rss_params_for_abort.lateral_distance_max_threshold ||
      p.safety.rss_params.longitudinal_distance_min_threshold >
        p.safety.rss_params_for_abort.longitudinal_distance_min_threshold ||
      p.safety.rss_params.longitudinal_velocity_delta_time >
        p.safety.rss_params_for_abort.longitudinal_velocity_delta_time) {
      RCLCPP_FATAL_STREAM(
        node->get_logger().get_child(node_name),
        "abort parameter might be loose... Terminating the program...");
      exit(EXIT_FAILURE);
    }
  }
  if (p.cancel.delta_time < 1.0) {
    RCLCPP_WARN_STREAM(
      node->get_logger().get_child(node_name),
      "cancel.delta_time: " << p.cancel.delta_time
                            << ", is too short. This could cause a danger behavior.");
  }

  return std::make_shared<lane_change::Parameters>(p);
}

void LaneChangeModuleManager::initParams(rclcpp::Node * node)
{
  parameters_ = set_params(node, name());
}

std::unique_ptr<SceneModuleInterface> LaneChangeModuleManager::createNewSceneModuleInstance()
{
  return std::make_unique<LaneChangeInterface>(
    name_, *node_, parameters_, rtc_interface_ptr_map_,
    objects_of_interest_marker_interface_ptr_map_, planning_factor_interface_,
    std::make_unique<NormalLaneChange>(parameters_, LaneChangeModuleType::NORMAL, direction_));
}

void LaneChangeModuleManager::updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  auto p = parameters_;

  {
    const std::string ns = "lane_change.";
    auto time_limit = p->time_limit;
    update_param<double>(parameters, ns + "time_limit", time_limit);
    if (time_limit >= 10.0) {
      p->time_limit = time_limit;
    } else {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "WARNING! Parameter 'time_limit' is not updated because the value (%.3f ms) is not valid, "
        "keep current value (%.3f ms)",
        time_limit, p->time_limit);
    }
    update_param<double>(parameters, ns + "backward_lane_length", p->backward_lane_length);
    update_param<double>(
      parameters, ns + "backward_length_buffer_for_end_of_lane",
      p->backward_length_buffer_for_end_of_lane);
    update_param<double>(
      parameters, ns + "backward_length_buffer_for_blocking_object",
      p->backward_length_buffer_for_blocking_object);
    update_param<double>(
      parameters, ns + "lane_change_finish_judge_buffer", p->lane_change_finish_judge_buffer);
    update_param<bool>(
      parameters, ns + "enable_stopped_vehicle_buffer", p->enable_stopped_vehicle_buffer);
    update_param<double>(
      parameters, ns + "finish_judge_lateral_threshold", p->th_finish_judge_lateral_diff);
    update_param<bool>(parameters, ns + "publish_debug_marker", p->publish_debug_marker);
    update_param<double>(
      parameters, ns + "min_length_for_turn_signal_activation",
      p->min_length_for_turn_signal_activation);
  }

  {
    const std::string ns = "lane_change.trajectory.";
    update_param<double>(
      parameters, ns + "max_prepare_duration", p->trajectory.max_prepare_duration);
    update_param<double>(
      parameters, ns + "min_prepare_duration", p->trajectory.min_prepare_duration);
    update_param<double>(parameters, ns + "lateral_jerk", p->trajectory.lateral_jerk);
    update_param<double>(
      parameters, ns + "min_lane_changing_velocity", p->trajectory.min_lane_changing_velocity);
    update_param<double>(
      parameters, ns + "min_longitudinal_acc", p->trajectory.min_longitudinal_acc);
    update_param<double>(
      parameters, ns + "max_longitudinal_acc", p->trajectory.max_longitudinal_acc);
    update_param<double>(
      parameters, ns + "lane_changing_decel_factor", p->trajectory.lane_changing_decel_factor);
    update_param<double>(
      parameters, ns + "th_prepare_curvature", p->trajectory.th_prepare_curvature);
    int longitudinal_acc_sampling_num = p->trajectory.lon_acc_sampling_num;
    update_param<int>(parameters, ns + "lon_acc_sampling_num", longitudinal_acc_sampling_num);
    if (longitudinal_acc_sampling_num > 0) {
      p->trajectory.lon_acc_sampling_num = longitudinal_acc_sampling_num;
    } else {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "WARNING! Parameter 'lon_acc_sampling_num' is not updated because the value (%d) is not "
        "positive",
        longitudinal_acc_sampling_num);
    }

    int lateral_acc_sampling_num = p->trajectory.lat_acc_sampling_num;
    update_param<int>(parameters, ns + "lat_acc_sampling_num", lateral_acc_sampling_num);
    if (lateral_acc_sampling_num > 0) {
      p->trajectory.lat_acc_sampling_num = lateral_acc_sampling_num;
    } else {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "WARNING! Parameter 'lat_acc_sampling_num' is not updated because the value (%d) is not "
        "positive",
        lateral_acc_sampling_num);
    }

    update_param<double>(
      parameters, ns + "th_prepare_length_diff", p->trajectory.th_prepare_length_diff);
    update_param<double>(
      parameters, ns + "th_lane_changing_length_diff", p->trajectory.th_lane_changing_length_diff);
  }

  {
    const std::string ns = "lane_change.frenet.";
    update_param<bool>(parameters, ns + "enable", p->frenet.enable);
    update_param<bool>(
      parameters, ns + "use_entire_remaining_distance", p->frenet.use_entire_remaining_distance);
    update_param<double>(parameters, ns + "th_yaw_diff", p->frenet.th_yaw_diff_deg);
    update_param<double>(
      parameters, ns + "th_curvature_smoothing", p->frenet.th_curvature_smoothing);
    update_param<double>(parameters, ns + "th_average_curvature", p->frenet.th_average_curvature);
  }

  {
    const std::string ns = "lane_change.safety_check.lane_expansion.";
    update_param<double>(parameters, ns + "left_offset", p->safety.lane_expansion_left_offset);
    update_param<double>(parameters, ns + "right_offset", p->safety.lane_expansion_right_offset);
  }

  {
    const std::string ns = "lane_change.lateral_acceleration.";
    std::vector<double> velocity = p->trajectory.lat_acc_map.base_vel;
    std::vector<double> min_values = p->trajectory.lat_acc_map.base_min_acc;
    std::vector<double> max_values = p->trajectory.lat_acc_map.base_max_acc;

    update_param<std::vector<double>>(parameters, ns + "velocity", velocity);
    update_param<std::vector<double>>(parameters, ns + "min_values", min_values);
    update_param<std::vector<double>>(parameters, ns + "max_values", max_values);
    if (
      velocity.size() >= 2 && velocity.size() == min_values.size() &&
      velocity.size() == max_values.size()) {
      LateralAccelerationMap lat_acc_map;
      for (size_t i = 0; i < velocity.size(); ++i) {
        lat_acc_map.add(velocity.at(i), min_values.at(i), max_values.at(i));
      }
      p->trajectory.lat_acc_map = lat_acc_map;
    } else {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Mismatched size for lateral acceleration. Expected size: %lu, but velocity: %lu, "
        "min_values: %lu, max_values: %lu",
        std::max(2ul, velocity.size()), velocity.size(), min_values.size(), max_values.size());
    }
  }

  {
    const std::string ns = "lane_change.collision_check.";
    update_param<bool>(
      parameters, ns + "enable_for_prepare_phase.general_lanes",
      p->safety.collision_check.enable_for_prepare_phase_in_general_lanes);
    update_param<bool>(
      parameters, ns + "enable_for_prepare_phase.intersection",
      p->safety.collision_check.enable_for_prepare_phase_in_intersection);
    update_param<bool>(
      parameters, ns + "enable_for_prepare_phase.turns",
      p->safety.collision_check.enable_for_prepare_phase_in_turns);
    update_param<bool>(
      parameters, ns + "check_current_lanes", p->safety.collision_check.check_current_lane);
    update_param<bool>(
      parameters, ns + "check_other_lanes", p->safety.collision_check.check_other_lanes);
    update_param<bool>(
      parameters, ns + "use_all_predicted_paths",
      p->safety.collision_check.use_all_predicted_paths);
    update_param<double>(
      parameters, ns + "prediction_time_resolution",
      p->safety.collision_check.prediction_time_resolution);
    update_param<double>(
      parameters, ns + "yaw_diff_threshold", p->safety.collision_check.th_yaw_diff);

    auto th_incoming_object_yaw = p->safety.collision_check.th_incoming_object_yaw;
    update_param<double>(parameters, ns + "th_incoming_object_yaw", th_incoming_object_yaw);
    if (th_incoming_object_yaw >= M_PI_2) {
      p->safety.collision_check.th_incoming_object_yaw = th_incoming_object_yaw;
    } else {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "The value of th_incoming_object_yaw (%.3f rad) is less than the minimum possible value "
        "(%.3f "
        "rad).",
        th_incoming_object_yaw, M_PI_2);
    }
  }

  {
    const std::string ns = "lane_change.target_object.";
    update_param<bool>(parameters, ns + "car", p->safety.target_object_types.check_car);
    update_param<bool>(parameters, ns + "truck", p->safety.target_object_types.check_truck);
    update_param<bool>(parameters, ns + "bus", p->safety.target_object_types.check_bus);
    update_param<bool>(parameters, ns + "trailer", p->safety.target_object_types.check_trailer);
    update_param<bool>(parameters, ns + "unknown", p->safety.target_object_types.check_unknown);
    update_param<bool>(parameters, ns + "bicycle", p->safety.target_object_types.check_bicycle);
    update_param<bool>(
      parameters, ns + "motorcycle", p->safety.target_object_types.check_motorcycle);
    update_param<bool>(
      parameters, ns + "pedestrian", p->safety.target_object_types.check_pedestrian);
  }

  {
    const std::string ns = "lane_change.regulation.";
    update_param<bool>(parameters, ns + "crosswalk", p->regulate_on_crosswalk);
    update_param<bool>(parameters, ns + "intersection", p->regulate_on_intersection);
    update_param<bool>(parameters, ns + "traffic_light", p->regulate_on_traffic_light);
  }

  {
    const std::string ns = "lane_change.stuck_detection.";
    update_param<double>(parameters, ns + "velocity", p->th_stop_velocity);
    update_param<double>(parameters, ns + "stop_time", p->th_stop_time);
  }

  auto update_rss_params = [&parameters, this](const std::string & prefix, auto & params) {
    using autoware_utils::update_param;
    update_param<double>(
      parameters, prefix + "longitudinal_distance_min_threshold",
      params.longitudinal_distance_min_threshold);
    update_param<double>(
      parameters, prefix + "longitudinal_velocity_delta_time",
      params.longitudinal_velocity_delta_time);
    update_param<double>(
      parameters, prefix + "expected_front_deceleration", params.front_vehicle_deceleration);
    update_param<double>(
      parameters, prefix + "expected_rear_deceleration", params.rear_vehicle_deceleration);
    update_param<double>(
      parameters, prefix + "rear_vehicle_reaction_time", params.rear_vehicle_reaction_time);
    update_param<double>(
      parameters, prefix + "rear_vehicle_safety_time_margin",
      params.rear_vehicle_safety_time_margin);
    update_param<double>(
      parameters, prefix + "lateral_distance_max_threshold", params.lateral_distance_max_threshold);

    auto extended_polygon_policy = params.extended_polygon_policy;
    update_param<std::string>(
      parameters, prefix + "extended_polygon_policy", extended_polygon_policy);
    if (extended_polygon_policy == "rectangle" || extended_polygon_policy == "along_path") {
      params.extended_polygon_policy = extended_polygon_policy;
    } else {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Policy %s not supported or there's typo. Make sure you choose either 'rectangle' or "
        "'along_path'",
        extended_polygon_policy.c_str());
    }
  };

  update_rss_params("lane_change.safety_check.execution.", p->safety.rss_params);
  update_rss_params("lane_change.safety_check.parked.", p->safety.rss_params_for_parked);
  update_rss_params("lane_change.safety_check.cancel.", p->safety.rss_params_for_abort);
  update_rss_params("lane_change.safety_check.stuck.", p->safety.rss_params_for_stuck);

  {
    const std::string ns = "lane_change.delay_lane_change.";
    update_param<bool>(parameters, ns + "enable", p->delay.enable);
    update_param<bool>(
      parameters, ns + "check_only_parked_vehicle", p->delay.check_only_parked_vehicle);
    update_param<double>(
      parameters, ns + "min_road_shoulder_width", p->delay.min_road_shoulder_width);
    update_param<double>(
      parameters, ns + "th_parked_vehicle_shift_ratio", p->delay.th_parked_vehicle_shift_ratio);
  }

  {
    const std::string ns = "lane_change.terminal_path.";
    update_param<bool>(parameters, ns + "enable", p->terminal_path.enable);
    update_param<bool>(parameters, ns + "disable_near_goal", p->terminal_path.disable_near_goal);
    update_param<bool>(parameters, ns + "stop_at_boundary", p->terminal_path.stop_at_boundary);
  }

  {
    const std::string ns = "lane_change.cancel.";
    bool enable_on_prepare_phase = p->cancel.enable_on_prepare_phase;
    update_param<bool>(parameters, ns + "enable_on_prepare_phase", enable_on_prepare_phase);
    if (!enable_on_prepare_phase) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "WARNING! Lane Change cancel function is disabled.");
      p->cancel.enable_on_prepare_phase = enable_on_prepare_phase;
    }

    bool enable_on_lane_changing_phase = p->cancel.enable_on_lane_changing_phase;
    update_param<bool>(
      parameters, ns + "enable_on_lane_changing_phase", enable_on_lane_changing_phase);
    if (!enable_on_lane_changing_phase) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "WARNING! Lane Change abort function is disabled.");
      p->cancel.enable_on_lane_changing_phase = enable_on_lane_changing_phase;
    }

    int deceleration_sampling_num = p->cancel.deceleration_sampling_num;
    update_param<int>(parameters, ns + "deceleration_sampling_num", deceleration_sampling_num);
    if (deceleration_sampling_num > 0) {
      p->cancel.deceleration_sampling_num = deceleration_sampling_num;
    } else {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Parameter 'deceleration_sampling_num' is not updated because the value (%d) is not "
        "positive",
        deceleration_sampling_num);
    }

    update_param<double>(parameters, ns + "delta_time", p->cancel.delta_time);
    update_param<double>(parameters, ns + "duration", p->cancel.duration);
    update_param<double>(parameters, ns + "max_lateral_jerk", p->cancel.max_lateral_jerk);
    update_param<double>(parameters, ns + "overhang_tolerance", p->cancel.overhang_tolerance);
    update_param<int>(
      parameters, ns + "unsafe_hysteresis_threshold", p->cancel.th_unsafe_hysteresis);
  }

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::LaneChangeRightModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::LaneChangeLeftModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
