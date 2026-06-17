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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/traffic_light_stop.hpp"

#include "autoware/traffic_light_compliance_checker/traffic_light_compliance_checker.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp"

#include <algorithm>
#include <memory>
#include <string>

namespace
{
autoware::traffic_light_compliance_checker::Parameters to_checker_params(
  const autoware::trajectory_modifier::plugin::TrajectoryModifierParams & params)
{
  const auto tl_stop_p = params.traffic_light_stop;
  const auto stopping_params = params.stopping_constraints;
  autoware::traffic_light_compliance_checker::Parameters p;
  p.deceleration_limit = stopping_params.maximum_deceleration;
  p.jerk_limit = stopping_params.jerk_limit;
  p.crossing_time_limit = tl_stop_p.crossing_time_limit;
  p.treat_amber_light_as_red_light = tl_stop_p.treat_amber_light_as_red;
  p.stop_overshoot_margin = tl_stop_p.overshoot_tolerance;
  p.stable_duration_threshold_red = tl_stop_p.th_stable_duration_red;
  p.stable_duration_threshold_amber = tl_stop_p.th_stable_duration_amber;
  p.amber_rejection_hysteresis_duration = tl_stop_p.th_amber_rejection_hysteresis;
  p.checked_trajectory_length.deceleration_limit = stopping_params.nominal_deceleration;
  p.checked_trajectory_length.jerk_limit = stopping_params.jerk_limit;
  return p;
}
}  // namespace

namespace autoware::trajectory_modifier::plugin
{

void TrafficLightStop::on_initialize([[maybe_unused]] const TrajectoryModifierParams & params)
{
  const auto node_ptr = get_node_ptr();
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      node_ptr, "modifier_traffic_light_stop");

  pub_debug_text_ = node_ptr->create_publisher<StringStamped>("~/traffic_light_stop/debug/text", 1);

  enabled_ = params.use_traffic_light_stop;
  params_ = params.traffic_light_stop;
  stopping_params_ = params.stopping_constraints;

  checker_ =
    std::make_unique<autoware::traffic_light_compliance_checker::TrafficLightComplianceChecker>(
      to_checker_params(params), context_->vehicle_info);
}

void TrafficLightStop::update_params([[maybe_unused]] const TrajectoryModifierParams & params)
{
  enabled_ = params.use_traffic_light_stop;
  params_ = params.traffic_light_stop;
  stopping_params_ = params.stopping_constraints;
  checker_->update_parameters(to_checker_params(params));
}

bool TrafficLightStop::is_trajectory_modification_required(
  [[maybe_unused]] const TrajectoryPoints & traj_points, [[maybe_unused]] const InputData & input)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "TrafficLightStop::is_trajectory_modification_required", *get_time_keeper());
  if (!enabled_ || !check_inputs(input)) return false;

  if (!checker_) {
    RCLCPP_ERROR(get_node_ptr()->get_logger(), "Compliance checker is not initialized.");
    return false;
  }

  return check_traffic_lights(traj_points, input);
}

bool TrafficLightStop::check_inputs(const InputData & input)
{
  return input.current_odometry && input.current_acceleration && input.route &&
         input.traffic_light_signals && input.lanelet_map;
}

bool TrafficLightStop::check_traffic_lights(
  const TrajectoryPoints & traj_points, const InputData & input)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "TrafficLightStop::check_traffic_lights", *get_time_keeper());

  const traffic_light_compliance_checker::Inputs inputs{
    traj_points,
    input.lanelet_map,
    *input.route,
    *input.traffic_light_signals,
    get_clock()->now(),
    input.current_odometry->twist.twist.linear.x,
    input.current_acceleration->accel.accel.linear.x};

  const auto result =
    checker_->check(inputs, params_.stop_for_red_light, params_.stop_for_amber_light);
  if (!result) return false;

  if (result->violations.empty()) return false;

  const auto nearest_it = std::min_element(result->violations.begin(), result->violations.end());
  nearest_violation_ = *nearest_it;

  debug_data_.violations_count = result->violations.size();
  debug_data_.nearest_violation_arc_length = nearest_it->arc_length_to_cross_point;

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 1000,
    "[TM TrafficLightStop] Detected traffic light violation at arc length %f m",
    nearest_it->arc_length_to_cross_point);
  return true;
}

bool TrafficLightStop::modify_trajectory(
  [[maybe_unused]] TrajectoryPoints & traj_points, [[maybe_unused]] const InputData & input)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "TrafficLightStop::modify_trajectory", *get_time_keeper());
  debug_data_ = DebugData{};

  if (is_trajectory_modification_required(traj_points, input) && nearest_violation_) {
    debug_data_.active = set_stop_point(traj_points, input);
  }

  publish_debug_string();
  return debug_data_.active;
}

bool TrafficLightStop::set_stop_point(TrajectoryPoints & traj_points, const InputData & input)
{
  autoware_utils_debug::ScopedTimeTrack st("TrafficLightStop::set_stop_point", *get_time_keeper());

  const auto trajectory_length =
    motion_utils::calcSignedArcLength(traj_points, 0, traj_points.size() - 1);

  const auto stop_margin = params_.stop_margin + context_->vehicle_info.max_longitudinal_offset_m;
  const auto target_stop_point_arc_length = utils::clamp_stop_point_arc_length(
    nearest_violation_->arc_length_to_cross_point - stop_margin, trajectory_length,
    input.current_odometry->twist.twist.linear.x, input.current_acceleration->accel.accel.linear.x,
    stopping_params_.maximum_deceleration, stopping_params_.jerk_limit);

  if (utils::stop_point_exists(traj_points, target_stop_point_arc_length)) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM TrafficLightStop] Preceding (or duplicate) stop point exists, skip inserting stop "
      "point");
    return false;
  }

  if (
    target_stop_point_arc_length < stopping_params_.arrived_distance_threshold ||
    !utils::insert_stop_point(traj_points, target_stop_point_arc_length, trajectory_length)) {
    traj_points = std::invoke([&]() {
      TrajectoryPoints stop_points;
      auto p = traj_points.front();
      p.longitudinal_velocity_mps = 0.0;
      p.acceleration_mps2 = 0.0;
      p.time_from_start = rclcpp::Duration::from_seconds(0.0);
      stop_points.push_back(p);
      p.time_from_start = rclcpp::Duration::from_seconds(trajectory_time_step_);
      stop_points.push_back(p);
      return stop_points;
    });
  }

  const auto & stop_pose = traj_points.back().pose;
  const auto & ego_pose = input.current_odometry->pose.pose;
  auto distance =
    motion_utils::calcSignedArcLength(traj_points, ego_pose.position, stop_pose.position);
  if (std::isnan(distance)) distance = 0.0;
  planning_factor_interface_->add(distance, stop_pose, PlanningFactor::STOP, SafetyFactorArray{});

  debug_data_.stop_point_arc_length = target_stop_point_arc_length;

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 1000,
    "[TM TrafficLightStop] Inserted stop point at arc length %f m", target_stop_point_arc_length);
  return true;
}

void TrafficLightStop::publish_debug_string() const
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2) << std::boolalpha;
  ss << "TRAFFIC LIGHT STOP MODIFIER: " << "\n";
  ss << "\t\t" << "ACTIVE: " << debug_data_.active << "\n";
  if (debug_data_.active) {
    ss << "\t\t" << "VIOLATIONS: " << debug_data_.violations_count << "\n";
    ss << "\t\t" << "NEAREST VIOLATION: " << debug_data_.nearest_violation_arc_length << " m"
       << "\n";
    ss << "\t\t" << "STOP POINT: " << debug_data_.stop_point_arc_length << " m" << "\n";
  }
  StringStamped string_stamp;
  string_stamp.stamp = get_clock()->now();
  string_stamp.data = ss.str();
  pub_debug_text_->publish(string_stamp);
}

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::TrafficLightStop,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
