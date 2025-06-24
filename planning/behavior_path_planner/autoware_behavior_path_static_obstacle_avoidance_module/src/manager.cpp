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

#include "autoware/behavior_path_static_obstacle_avoidance_module/manager.hpp"

#include "autoware/behavior_path_static_obstacle_avoidance_module/parameter_helper.hpp"
#include "autoware_utils/ros/parameter.hpp"
#include "autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
void StaticObstacleAvoidanceModuleManager::init(rclcpp::Node * node)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  using autoware_utils::get_or_declare_parameter;

  // init manager interface
  initInterface(node, {"left", "right"});

  auto p = getParameter(node);

  parameters_ = std::make_shared<AvoidanceParameters>(p);
}

void StaticObstacleAvoidanceModuleManager::updateModuleParams(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  using autoware_utils::update_param;

  auto p = parameters_;

  const auto update_object_param = [&p, &parameters](
                                     const auto & semantic, const std::string & ns) {
    auto & config = p->object_parameters.at(semantic);
    update_param<double>(parameters, ns + "th_moving_speed", config.moving_speed_threshold);
    update_param<double>(parameters, ns + "th_moving_time", config.moving_time_threshold);
    update_param<double>(parameters, ns + "max_expand_ratio", config.max_expand_ratio);
    update_param<double>(parameters, ns + "envelope_buffer_margin", config.envelope_buffer_margin);
    update_param<double>(parameters, ns + "lateral_margin.soft_margin", config.lateral_soft_margin);
    update_param<double>(parameters, ns + "lateral_margin.hard_margin", config.lateral_hard_margin);
    update_param<double>(
      parameters, ns + "lateral_margin.hard_margin_for_parked_vehicle",
      config.lateral_hard_margin_for_parked_vehicle);
    update_param<double>(parameters, ns + "longitudinal_margin", config.longitudinal_margin);
    update_param<double>(
      parameters, ns + "th_error_eclipse_long_radius", config.th_error_eclipse_long_radius);
  };

  {
    const std::string ns = "avoidance.target_object.";
    update_object_param(ObjectClassification::MOTORCYCLE, ns + "motorcycle.");
    update_object_param(ObjectClassification::CAR, ns + "car.");
    update_object_param(ObjectClassification::TRUCK, ns + "truck.");
    update_object_param(ObjectClassification::TRAILER, ns + "trailer.");
    update_object_param(ObjectClassification::BUS, ns + "bus.");
    update_object_param(ObjectClassification::PEDESTRIAN, ns + "pedestrian.");
    update_object_param(ObjectClassification::BICYCLE, ns + "bicycle.");
    update_object_param(ObjectClassification::UNKNOWN, ns + "unknown.");

    update_param<double>(
      parameters, ns + "lower_distance_for_polygon_expansion",
      p->lower_distance_for_polygon_expansion);
    update_param<double>(
      parameters, ns + "upper_distance_for_polygon_expansion",
      p->upper_distance_for_polygon_expansion);
  }

  {
    const auto set_target_flag = [&](const uint8_t & object_type, const std::string & ns) {
      if (p->object_parameters.count(object_type) == 0) {
        return;
      }
      update_param<bool>(parameters, ns, p->object_parameters.at(object_type).is_avoidance_target);
    };

    const std::string ns = "avoidance.target_filtering.";
    set_target_flag(ObjectClassification::CAR, ns + "target_type.car");
    set_target_flag(ObjectClassification::TRUCK, ns + "target_type.truck");
    set_target_flag(ObjectClassification::TRAILER, ns + "target_type.trailer");
    set_target_flag(ObjectClassification::BUS, ns + "target_type.bus");
    set_target_flag(ObjectClassification::PEDESTRIAN, ns + "target_type.pedestrian");
    set_target_flag(ObjectClassification::BICYCLE, ns + "target_type.bicycle");
    set_target_flag(ObjectClassification::MOTORCYCLE, ns + "target_type.motorcycle");
    set_target_flag(ObjectClassification::UNKNOWN, ns + "target_type.unknown");

    update_param<double>(
      parameters, ns + "object_check_goal_distance", p->object_check_goal_distance);
    update_param<double>(
      parameters, ns + "object_check_return_pose_distance", p->object_check_return_pose_distance);
    update_param<double>(parameters, ns + "max_compensation_time", p->object_last_seen_threshold);
    update_param<double>(
      parameters, ns + "unstable_classification_time", p->unstable_classification_time);
  }

  {
    const std::string ns = "avoidance.target_filtering.parked_vehicle.";
    update_param<double>(
      parameters, ns + "th_offset_from_centerline", p->threshold_distance_object_is_on_center);
    update_param<double>(parameters, ns + "th_shiftable_ratio", p->object_check_shiftable_ratio);
    update_param<double>(
      parameters, ns + "min_road_shoulder_width", p->object_check_min_road_shoulder_width);
  }

  {
    const std::string ns = "avoidance.target_filtering.detection_area.";
    update_param<bool>(parameters, ns + "static", p->use_static_detection_area);
    update_param<double>(
      parameters, ns + "min_forward_distance", p->object_check_min_forward_distance);
    update_param<double>(
      parameters, ns + "max_forward_distance", p->object_check_max_forward_distance);
    update_param<double>(parameters, ns + "backward_distance", p->object_check_backward_distance);
  }

  {
    const std::string ns = "avoidance.target_filtering.merging_vehicle.";
    update_param<double>(parameters, ns + "th_overhang_distance", p->th_overhang_distance);
  }

  {
    const std::string ns = "avoidance.avoidance.lateral.avoidance_for_ambiguous_vehicle.";
    update_param<std::string>(parameters, ns + "policy", p->policy_ambiguous_vehicle);
    update_param<double>(
      parameters, ns + "wait_and_see.th_closest_distance", p->wait_and_see_th_closest_distance);
    update_param<double>(
      parameters, ns + "condition.th_stopped_time", p->time_threshold_for_ambiguous_vehicle);
    update_param<double>(
      parameters, ns + "condition.th_moving_distance", p->distance_threshold_for_ambiguous_vehicle);
    update_param<double>(
      parameters, ns + "ignore_area.traffic_light.front_distance",
      p->object_ignore_section_traffic_light_in_front_distance);
    update_param<double>(
      parameters, ns + "ignore_area.crosswalk.front_distance",
      p->object_ignore_section_crosswalk_in_front_distance);
    update_param<double>(
      parameters, ns + "ignore_area.crosswalk.behind_distance",
      p->object_ignore_section_crosswalk_behind_distance);
  }

  {
    const std::string ns = "avoidance.target_filtering.freespace.";
    update_param<double>(
      parameters, ns + "condition.th_stopped_time", p->freespace_condition_th_stopped_time);
  }

  {
    const std::string ns = "avoidance.target_filtering.intersection.";
    update_param<double>(parameters, ns + "yaw_deviation", p->object_check_yaw_deviation);
  }

  {
    const std::string ns = "avoidance.avoidance.lateral.";
    update_param<double>(parameters, ns + "th_avoid_execution", p->lateral_execution_threshold);
    update_param<double>(
      parameters, ns + "th_small_shift_length", p->lateral_small_shift_threshold);
    update_param<double>(
      parameters, ns + "soft_drivable_bound_margin", p->soft_drivable_bound_margin);
    update_param<double>(
      parameters, ns + "hard_drivable_bound_margin", p->hard_drivable_bound_margin);
  }

  {
    const std::string ns = "avoidance.avoidance.longitudinal.";
    update_param<double>(parameters, ns + "min_prepare_time", p->min_prepare_time);
    update_param<double>(parameters, ns + "max_prepare_time", p->max_prepare_time);
    update_param<double>(parameters, ns + "min_prepare_distance", p->min_prepare_distance);
    update_param<double>(parameters, ns + "min_slow_down_speed", p->min_slow_down_speed);
    update_param<double>(parameters, ns + "buf_slow_down_speed", p->buf_slow_down_speed);
    update_param<bool>(parameters, ns + "consider_front_overhang", p->consider_front_overhang);
    update_param<bool>(parameters, ns + "consider_rear_overhang", p->consider_rear_overhang);
  }

  {
    const std::string ns = "avoidance.cancel.";
    update_param<double>(parameters, ns + "force.duration_time", p->force_deactivate_duration_time);
  }

  {
    const std::string ns = "avoidance.stop.";
    update_param<double>(parameters, ns + "max_distance", p->stop_max_distance);
    update_param<double>(parameters, ns + "stop_buffer", p->stop_buffer);
  }

  {
    const std::string ns = "avoidance.policy.";
    update_param<std::string>(parameters, ns + "make_approval_request", p->policy_approval);
    update_param<std::string>(parameters, ns + "deceleration", p->policy_deceleration);
    update_param<std::string>(parameters, ns + "lateral_margin", p->policy_lateral_margin);
    update_param<bool>(
      parameters, ns + "use_shorten_margin_immediately", p->use_shorten_margin_immediately);

    if (p->policy_approval != "per_shift_line" && p->policy_approval != "per_avoidance_maneuver") {
      RCLCPP_ERROR(
        rclcpp::get_logger(__func__),
        "invalid policy. please select 'per_shift_line' or 'per_avoidance_maneuver'.");
    }

    if (p->policy_deceleration != "best_effort" && p->policy_deceleration != "reliable") {
      RCLCPP_ERROR(
        rclcpp::get_logger(__func__),
        "invalid deceleration policy. Please select 'best_effort' or 'reliable'.");
    }

    if (p->policy_lateral_margin != "best_effort" && p->policy_lateral_margin != "reliable") {
      RCLCPP_ERROR(
        rclcpp::get_logger(__func__),
        "invalid lateral margin policy. Please select 'best_effort' or 'reliable'.");
    }
  }

  {
    const std::string ns = "avoidance.constrains.lateral.";

    std::vector<double> velocity_map;
    update_param<std::vector<double>>(parameters, ns + "velocity", velocity_map);
    std::vector<double> lateral_max_accel_map;
    update_param<std::vector<double>>(parameters, ns + "max_accel_values", lateral_max_accel_map);
    std::vector<double> avoid_lateral_min_jerk_map;
    update_param<std::vector<double>>(
      parameters, ns + "min_jerk_values.avoid", avoid_lateral_min_jerk_map);
    std::vector<double> return_lateral_min_jerk_map;
    update_param<std::vector<double>>(
      parameters, ns + "min_jerk_values.return", return_lateral_min_jerk_map);
    std::vector<double> lateral_max_jerk_map;
    update_param<std::vector<double>>(parameters, ns + "max_jerk_values", lateral_max_jerk_map);

    if (!velocity_map.empty()) {
      p->velocity_map = velocity_map;
    }

    if (!velocity_map.empty() && velocity_map.size() == lateral_max_accel_map.size()) {
      p->lateral_max_accel_map = lateral_max_accel_map;
    }

    if (!velocity_map.empty() && velocity_map.size() == avoid_lateral_min_jerk_map.size()) {
      p->avoid_lateral_min_jerk_map = avoid_lateral_min_jerk_map;
    }

    if (!velocity_map.empty() && velocity_map.size() == return_lateral_min_jerk_map.size()) {
      p->return_lateral_min_jerk_map = return_lateral_min_jerk_map;
    }

    if (!velocity_map.empty() && velocity_map.size() == lateral_max_jerk_map.size()) {
      p->lateral_max_jerk_map = lateral_max_jerk_map;
    }
  }

  {
    const std::string ns = "avoidance.constraints.longitudinal.";

    update_param<double>(parameters, ns + "nominal_deceleration", p->nominal_deceleration);
    update_param<double>(parameters, ns + "nominal_jerk", p->nominal_jerk);
    update_param<double>(parameters, ns + "max_deceleration", p->max_deceleration);
    update_param<double>(parameters, ns + "max_jerk", p->max_jerk);
    update_param<double>(parameters, ns + "max_acceleration", p->max_acceleration);
    update_param<double>(
      parameters, ns + "min_velocity_to_limit_max_acceleration",
      p->min_velocity_to_limit_max_acceleration);
  }

  {
    const std::string ns = "avoidance.shift_line_pipeline.";
    update_param<double>(parameters, ns + "trim.quantize_size", p->quantize_size);
    update_param<double>(parameters, ns + "trim.th_similar_grad_1", p->th_similar_grad_1);
    update_param<double>(parameters, ns + "trim.th_similar_grad_2", p->th_similar_grad_2);
    update_param<double>(parameters, ns + "trim.th_similar_grad_3", p->th_similar_grad_3);
  }

  {
    const std::string ns = "avoidance.debug.";
    update_param<bool>(
      parameters, ns + "enable_other_objects_marker", p->enable_other_objects_marker);
    update_param<bool>(parameters, ns + "enable_other_objects_info", p->enable_other_objects_info);
    update_param<bool>(
      parameters, ns + "enable_detection_area_marker", p->enable_detection_area_marker);
    update_param<bool>(
      parameters, ns + "enable_drivable_bound_marker", p->enable_drivable_bound_marker);
    update_param<bool>(
      parameters, ns + "enable_safety_check_marker", p->enable_safety_check_marker);
    update_param<bool>(parameters, ns + "enable_shift_line_marker", p->enable_shift_line_marker);
    update_param<bool>(parameters, ns + "enable_lane_marker", p->enable_lane_marker);
    update_param<bool>(parameters, ns + "enable_misc_marker", p->enable_misc_marker);
  }

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}
}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::StaticObstacleAvoidanceModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
