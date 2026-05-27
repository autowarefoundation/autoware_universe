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

#include "autoware/trajectory_validator/filters/traffic_rule/traffic_light_filter.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
std::optional<std::string> is_invalid_input(
  const autoware::trajectory_validator::FilterContext & context,
  const std::shared_ptr<autoware::vehicle_info_utils::VehicleInfo> & vehicle_info)
{
  if (!context.lanelet_map) {
    return "Lanelet map is not available in the context.";
  }

  if (!context.route) {
    return "Route is not available in the context.";
  }

  if (!vehicle_info) {
    return "Vehicle info is not set.";
  }

  if (!context.traffic_light_signals) {
    return "Traffic light signals are not available in the context.";
  }

  if (!context.odometry || !context.acceleration) {
    return "Odometry or acceleration is not available in the context.";
  }

  return std::nullopt;
}

autoware::trajectory_validator::traffic_light_filter::Parameters to_checker_params(
  const validator::Params::TrafficLight & params)
{
  autoware::trajectory_validator::traffic_light_filter::Parameters p;
  p.deceleration_limit = params.deceleration_limit;
  p.jerk_limit = params.jerk_limit;
  p.delay_response_time = params.delay_response_time;
  p.crossing_time_limit = params.crossing_time_limit;
  p.treat_amber_light_as_red_light = params.treat_amber_light_as_red_light;
  p.stop_overshoot_margin = params.stop_overshoot_margin;
  p.stable_duration_threshold_red = params.stable_duration_threshold_red;
  p.stable_duration_threshold_amber = params.stable_duration_threshold_amber;
  p.amber_rejection_hysteresis_duration = params.amber_rejection_hysteresis_duration;
  p.ego_stopped_velocity_threshold = params.ego_stopped_velocity_threshold;
  p.checked_trajectory_length.deceleration_limit =
    params.checked_trajectory_length.deceleration_limit;
  p.checked_trajectory_length.jerk_limit = params.checked_trajectory_length.jerk_limit;
  return p;
}

bool is_equal(
  const autoware_perception_msgs::msg::TrafficLightElement & a,
  const autoware_perception_msgs::msg::TrafficLightElement & b)
{
  return a.color == b.color && a.shape == b.shape && a.status == b.status;
}

bool is_equal(
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & a,
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & b)
{
  if (a.size() != b.size()) {
    return false;
  }

  for (size_t i = 0; i < a.size(); ++i) {
    if (!is_equal(a[i], b[i])) {
      return false;
    }
  }

  return true;
}
}  // namespace

namespace autoware::trajectory_validator::plugin::traffic_rule
{

TrafficLightFilter::TrafficLightFilter() : ValidatorInterface("traffic_light_filter")
{
}

void TrafficLightFilter::update_parameters(const validator::Params & params)
{
  params_ = params.traffic_light;
  if (checker_) {
    checker_->update_parameters(to_checker_params(params_));
  }
}

void TrafficLightFilter::set_vehicle_info(const VehicleInfo & vehicle_info)
{
  ValidatorInterface::set_vehicle_info(vehicle_info);
  checker_ = std::make_unique<traffic_light_filter::TrafficLightComplianceChecker>(
    to_checker_params(params_), vehicle_info);
}

TrafficLightFilter::result_t TrafficLightFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  if (const auto has_invalid_input = is_invalid_input(context, vehicle_info_ptr_)) {
    return tl::make_unexpected(*has_invalid_input);
  }

  if (!checker_) {
    return tl::make_unexpected("Compliance checker is not initialized.");
  }

  const auto current_time = rclcpp::Time(context.odometry->header.stamp);
  const auto velocity = context.odometry->twist.twist.linear.x;
  const bool is_ego_stopped = std::abs(velocity) < params_.ego_stopped_velocity_threshold;

  // Signal Stability Filter
  const auto filtered_signals =
    filter_signals(*context.traffic_light_signals, current_time, is_ego_stopped);

  // Amber Hysteresis Tracking
  const auto force_reject_amber_ids = get_force_reject_amber_ids(current_time, is_ego_stopped);

  const traffic_light_filter::Inputs inputs{
    traj_points,
    context.lanelet_map,
    *context.route,
    filtered_signals,
    context.odometry->twist.twist.linear.x,
    context.acceleration->accel.accel.linear.x,
    force_reject_amber_ids};

  const auto result = checker_->check(inputs);
  if (!result) {
    return tl::make_unexpected(result.error());
  }

  bool is_crossing_red = false;
  bool is_crossing_amber = false;

  for (const auto & violation : result->violations) {
    if (violation.type == traffic_light_filter::ViolationType::RED_LIGHT) {
      is_crossing_red = true;
    } else if (violation.type == traffic_light_filter::ViolationType::AMBER_LIGHT) {
      is_crossing_amber = true;
      if (
        std::find(
          force_reject_amber_ids.begin(), force_reject_amber_ids.end(),
          violation.traffic_light_id) == force_reject_amber_ids.end()) {
        amber_rejection_history_[violation.traffic_light_id] = current_time;
      }
    }
  }

  // Memory Management
  cleanup_history(current_time);

  std::vector<MetricReport> metrics;
  metrics.push_back(
    autoware_trajectory_validator::build<MetricReport>()
      .validator_name(get_name())
      .validator_category(category())
      .metric_name("check_crossing_red_light")
      .metric_value(0.0)
      .level(is_crossing_red ? MetricReport::ERROR : MetricReport::OK));

  metrics.push_back(
    autoware_trajectory_validator::build<MetricReport>()
      .validator_name(get_name())
      .validator_category(category())
      .metric_name("check_crossing_amber_light")
      .metric_value(0.0)
      .level(is_crossing_amber ? MetricReport::ERROR : MetricReport::OK));

  const bool is_feasible = !is_crossing_red && !is_crossing_amber;

  return ValidationResult{is_feasible, std::move(metrics)};
}

autoware_perception_msgs::msg::TrafficLightGroupArray TrafficLightFilter::filter_signals(
  const autoware_perception_msgs::msg::TrafficLightGroupArray & signals,
  const rclcpp::Time & current_time, bool is_ego_stopped)
{
  autoware_perception_msgs::msg::TrafficLightGroupArray filtered_signals;
  filtered_signals.stamp = signals.stamp;

  for (const auto & signal : signals.traffic_light_groups) {
    const auto id = signal.traffic_light_group_id;
    if (signal_history_.find(id) == signal_history_.end()) {
      signal_history_[id] = {signal, current_time, current_time};
    } else {
      if (!is_equal(signal_history_[id].msg.elements, signal.elements)) {
        signal_history_[id].first_seen_time = current_time;
        signal_history_[id].msg = signal;
      }
      signal_history_[id].last_seen_time = current_time;
    }

    auto filtered_signal = signal;
    if (is_ego_stopped) {
      filtered_signals.traffic_light_groups.push_back(filtered_signal);
      continue;
    }
    const auto state_duration = (current_time - signal_history_[id].first_seen_time).seconds();
    const bool is_red = autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
      signal.elements, autoware_perception_msgs::msg::TrafficLightElement::CIRCLE,
      autoware_perception_msgs::msg::TrafficLightElement::RED);
    const bool is_amber = autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
      signal.elements, autoware_perception_msgs::msg::TrafficLightElement::CIRCLE,
      autoware_perception_msgs::msg::TrafficLightElement::AMBER);

    if (is_red && state_duration < params_.stable_duration_threshold_red) {
      filtered_signal.elements.clear();
    } else if (is_amber && state_duration < params_.stable_duration_threshold_amber) {
      filtered_signal.elements.clear();
    }
    filtered_signals.traffic_light_groups.push_back(filtered_signal);
  }

  return filtered_signals;
}

std::vector<int64_t> TrafficLightFilter::get_force_reject_amber_ids(
  const rclcpp::Time & current_time, bool is_ego_stopped) const
{
  std::vector<int64_t> force_reject_amber_ids;
  if (is_ego_stopped) {
    return force_reject_amber_ids;
  }
  for (const auto & [id, rejected_time] : amber_rejection_history_) {
    if ((current_time - rejected_time).seconds() <= params_.amber_rejection_hysteresis_duration) {
      force_reject_amber_ids.push_back(id);
    }
  }
  return force_reject_amber_ids;
}

void TrafficLightFilter::cleanup_history(const rclcpp::Time & current_time)
{
  for (auto it = amber_rejection_history_.begin(); it != amber_rejection_history_.end();) {
    if ((current_time - it->second).seconds() > params_.amber_rejection_hysteresis_duration) {
      it = amber_rejection_history_.erase(it);
    } else {
      ++it;
    }
  }

  for (auto it = signal_history_.begin(); it != signal_history_.end();) {
    const auto stable_duration =
      autoware::traffic_light_utils::hasTrafficLightColor(
        it->second.msg.elements, autoware_perception_msgs::msg::TrafficLightElement::RED)
        ? params_.stable_duration_threshold_red
        : params_.stable_duration_threshold_amber;
    if ((current_time - it->second.last_seen_time).seconds() > stable_duration) {
      it = signal_history_.erase(it);
    } else {
      ++it;
    }
  }
}
}  // namespace autoware::trajectory_validator::plugin::traffic_rule

#include <pluginlib/class_list_macros.hpp>
namespace traffic_rule = autoware::trajectory_validator::plugin::traffic_rule;
PLUGINLIB_EXPORT_CLASS(
  traffic_rule::TrafficLightFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
