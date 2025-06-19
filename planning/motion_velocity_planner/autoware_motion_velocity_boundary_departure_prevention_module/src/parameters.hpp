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

#include "type_alias.hpp"

#include <autoware_utils_math/unit_conversion.hpp>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

namespace autoware::motion_velocity_planner::experimental
{
struct PredictedPathFootprint
{
  double scale{1.0};
  double extra_margin_m{0.0};
  double resample_interval_m{0.3};
};

struct Output
{
  std::unordered_map<std::string, double> processing_time_map;
  AbnormalitiesData abnormalities_data;
  ClosestProjectionsToBound closest_projections_to_bound;
  std::vector<std::tuple<Pose, Pose, double>> slowdown_intervals;

  trajectory::Trajectory<TrajectoryPoint> aw_ref_traj;
  trajectory::Trajectory<TrajectoryPoint> aw_ego_traj;

  DepartureIntervals departure_intervals;
  Side<DeparturePoints> departure_points;
  CriticalDeparturePoints critical_departure_points;
  std::unordered_map<DepartureType, bool> diagnostic_output{
    {DepartureType::NEAR_BOUNDARY, false},
    {DepartureType::APPROACHING_DEPARTURE, false},
    {DepartureType::CRITICAL_DEPARTURE, false}};
};

struct NodeParam
{
  double th_pt_shift_dist_m{1.0};
  double th_pt_shift_angle_rad{autoware_utils_math::deg2rad(2.0)};
  double th_goal_shift_dist_m{1.0};
  BDCParam bdc_param;
  std::unordered_set<DepartureType> slow_down_types;
  std::unordered_map<DepartureType, int8_t> diagnostic_level;

  NodeParam() = default;
  explicit NodeParam(rclcpp::Node & node)
  {
    const std::string module_name{"boundary_departure_prevention."};
    bdc_param.boundary_types_to_detect = get_or_declare_parameter<std::vector<std::string>>(
      node, module_name + "boundary_types_to_detect");
    bdc_param.th_dist_hysteresis_m =
      get_or_declare_parameter<double>(node, module_name + "th_dist_hysteresis_m");
    th_pt_shift_angle_rad =
      get_or_declare_parameter<double>(node, module_name + "th_pt_shift.dist_m");
    th_pt_shift_angle_rad = autoware_utils_math::deg2rad(
      get_or_declare_parameter<double>(node, module_name + "th_pt_shift.angle_deg"));
    th_goal_shift_dist_m =
      get_or_declare_parameter<double>(node, module_name + "th_pt_shift.goal_dist_m");

    bdc_param.th_max_lateral_query_num =
      get_or_declare_parameter<int>(node, module_name + "th_max_lateral_query_num");
    std::invoke([&node, &module_name, this]() {
      const std::string ns_abnormality{module_name + "abnormality."};
      const std::string ns_normal_abnormality{ns_abnormality + "normal."};
      const std::string ns_steering_abnormality{ns_abnormality + "steering."};
      const std::string ns_localization_abnormality{ns_abnormality + "localization."};
      const std::string ns_longitudinal_abnormality{ns_abnormality + "longitudinal."};

      const auto compensate_normal =
        get_or_declare_parameter<bool>(node, ns_normal_abnormality + "enable");
      const auto compensate_steering =
        get_or_declare_parameter<bool>(node, ns_steering_abnormality + "enable");
      const auto compensate_localization =
        get_or_declare_parameter<bool>(node, ns_localization_abnormality + "enable");
      const auto compensate_longitudinal =
        get_or_declare_parameter<bool>(node, ns_longitudinal_abnormality + "enable");

      std::vector<AbnormalityType> abnormality_types_to_compensate;
      AbnormalitiesConfigs configs;
      abnormality_types_to_compensate.reserve(4);
      if (compensate_normal) {
        abnormality_types_to_compensate.emplace_back(AbnormalityType::NORMAL);
        NormalConfig normal_config;
        const std::string footprint_envelop_ns{ns_normal_abnormality + "footprint_envelop."};
        normal_config.footprint_envelop.lat_m =
          get_or_declare_parameter<double>(node, footprint_envelop_ns + "lat_m");
        normal_config.footprint_envelop.lon_m =
          get_or_declare_parameter<double>(node, footprint_envelop_ns + "lon_m");
        configs.insert({AbnormalityType::NORMAL, normal_config});
      }

      if (compensate_steering) {
        abnormality_types_to_compensate.emplace_back(AbnormalityType::STEERING);
        SteeringConfig steering_config;
        steering_config.steering_rate_rps =
          get_or_declare_parameter<double>(node, ns_steering_abnormality + "steering_rate_rps");
        configs.insert({AbnormalityType::STEERING, steering_config});
      }

      if (compensate_localization) {
        abnormality_types_to_compensate.emplace_back(AbnormalityType::LOCALIZATION);
        LocalizationConfig localization_config;
        const std::string footprint_envelop_ns{ns_localization_abnormality + "footprint_envelop."};
        localization_config.footprint_envelop.lat_m =
          get_or_declare_parameter<double>(node, footprint_envelop_ns + "lat_m");
        localization_config.footprint_envelop.lon_m =
          get_or_declare_parameter<double>(node, footprint_envelop_ns + "lon_m");
        configs.insert({AbnormalityType::LOCALIZATION, localization_config});
      }

      if (compensate_longitudinal) {
        abnormality_types_to_compensate.emplace_back(AbnormalityType::LONGITUDINAL);
        LongitudinalConfig longitudinal_config;
        const std::string lon_tracking_ns{ns_longitudinal_abnormality + "lon_tracking."};
        longitudinal_config.lon_tracking.scale =
          get_or_declare_parameter<double>(node, lon_tracking_ns + "scale");
        longitudinal_config.lon_tracking.extra_margin_m =
          get_or_declare_parameter<double>(node, lon_tracking_ns + "extra_margin_m");
        configs.insert({AbnormalityType::LONGITUDINAL, longitudinal_config});
      }

      bdc_param.abnormality_types_to_compensate = abnormality_types_to_compensate;
      bdc_param.abnormality_configs = configs;
    });

    const auto select_diag_lvl = [](const int level) {
      if (DiagStatus::OK) {
        return DiagStatus::OK;
      }

      if (level == 1) {
        return DiagStatus::WARN;
      }

      if (level == 2) {
        return DiagStatus::ERROR;
      }

      return DiagStatus::STALE;
    };

    diagnostic_level = std::invoke([&]() {
      const std::string ns_diag{module_name + "diagnostic."};
      std::unordered_map<DepartureType, int8_t> diag;

      const auto near_boundary_diag_lvl =
        select_diag_lvl(get_or_declare_parameter<int>(node, ns_diag + "near_boundary"));
      const auto approaching_departure_diag_lvl =
        select_diag_lvl(get_or_declare_parameter<int>(node, ns_diag + "approaching_departure"));
      const auto critical_departure_diag_lvl =
        select_diag_lvl(get_or_declare_parameter<int>(node, ns_diag + "critical_departure"));

      diag.insert({DepartureType::NEAR_BOUNDARY, near_boundary_diag_lvl});
      diag.insert({DepartureType::APPROACHING_DEPARTURE, approaching_departure_diag_lvl});
      diag.insert({DepartureType::CRITICAL_DEPARTURE, critical_departure_diag_lvl});
      return diag;
    });

    const std::string ns_slow_down{module_name + "slow_down_behavior."};
    slow_down_types = std::invoke([&]() {
      std::unordered_set<DepartureType> departure_types;
      const std::string ns_dpt_type = {ns_slow_down + "enable."};
      const auto enable_slow_down_near_bound =
        get_or_declare_parameter<bool>(node, ns_dpt_type + "slow_down_near_boundary");
      const auto enable_critical_dpt =
        get_or_declare_parameter<bool>(node, ns_dpt_type + "stop_before_departure");
      const auto enable_approaching_dpt =
        get_or_declare_parameter<bool>(node, ns_dpt_type + "slow_down_before_departure");

      if (enable_slow_down_near_bound) {
        departure_types.insert(DepartureType::NEAR_BOUNDARY);
      }
      if (enable_approaching_dpt) {
        departure_types.insert(DepartureType::APPROACHING_DEPARTURE);
      }

      if (enable_critical_dpt) {
        departure_types.insert(DepartureType::CRITICAL_DEPARTURE);
      }

      return departure_types;
    });

    const std::string ns_th_trigger{ns_slow_down + "th_trigger."};

    bdc_param.th_trigger.th_vel_mps = std::invoke([&]() {
      const std::string ns_vel{ns_th_trigger + "th_vel_kmph."};
      TriggerThreshold::MinMax th_vel_mps;
      th_vel_mps.min =
        autoware_utils_math::kmph2mps(get_or_declare_parameter<double>(node, ns_vel + "min"));
      th_vel_mps.max =
        autoware_utils_math::kmph2mps(get_or_declare_parameter<double>(node, ns_vel + "max"));
      return th_vel_mps;
    });

    bdc_param.th_trigger.th_acc_mps2 = std::invoke([&]() {
      const std::string ns_acc{ns_th_trigger + "th_acc_mps2."};
      TriggerThreshold::MinMax th_acc_mps2;
      th_acc_mps2.min = get_or_declare_parameter<double>(node, ns_acc + "min");
      th_acc_mps2.max = get_or_declare_parameter<double>(node, ns_acc + "max");
      return th_acc_mps2;
    });

    bdc_param.th_trigger.th_jerk_mps3 = std::invoke([&]() {
      const std::string ns_jerk{ns_th_trigger + "th_jerk_mps3."};
      TriggerThreshold::MinMax th_jerk_mps3;
      th_jerk_mps3.min = get_or_declare_parameter<double>(node, ns_jerk + "min");
      th_jerk_mps3.max = get_or_declare_parameter<double>(node, ns_jerk + "max");
      return th_jerk_mps3;
    });

    bdc_param.th_trigger.brake_delay_s =
      get_or_declare_parameter<double>(node, ns_th_trigger + "brake_delay_s");

    bdc_param.th_trigger.dist_error_m =
      get_or_declare_parameter<double>(node, ns_th_trigger + "dist_error_m");

    bdc_param.th_trigger.th_dist_to_boundary_m = std::invoke([&]() {
      const std::string ns_dist_to_bound{ns_th_trigger + "th_dist_to_boundary_m."};
      Side<TriggerThreshold::MinMax> th_dist_to_boundary_m;
      const std::string ns_dist_to_bound_left{ns_dist_to_bound + "left."};
      const std::string ns_dist_to_bound_right{ns_dist_to_bound + "right."};

      th_dist_to_boundary_m.left.min =
        get_or_declare_parameter<double>(node, ns_dist_to_bound_left + "min");
      th_dist_to_boundary_m.left.max =
        get_or_declare_parameter<double>(node, ns_dist_to_bound_left + "max");

      th_dist_to_boundary_m.right.min =
        get_or_declare_parameter<double>(node, ns_dist_to_bound_right + "min");
      th_dist_to_boundary_m.right.max =
        get_or_declare_parameter<double>(node, ns_dist_to_bound_right + "max");
      return th_dist_to_boundary_m;
    });
  }
};
}  // namespace autoware::motion_velocity_planner::experimental

#endif  // PARAMETERS_HPP_
