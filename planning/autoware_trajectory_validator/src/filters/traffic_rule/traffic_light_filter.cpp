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

autoware::traffic_light_compliance_checker::Parameters to_checker_params(
  const validator::Params::TrafficLight & params)
{
  autoware::traffic_light_compliance_checker::Parameters p;
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
  checker_ = std::make_unique<traffic_light_compliance_checker::TrafficLightComplianceChecker>(
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

  const traffic_light_compliance_checker::Inputs inputs{
    traj_points,
    context.lanelet_map,
    *context.route,
    *context.traffic_light_signals,
    current_time,
    context.odometry->twist.twist.linear.x,
    context.acceleration->accel.accel.linear.x};

  const auto result = checker_->check(inputs);
  if (!result) {
    return tl::make_unexpected(result.error());
  }

  bool is_crossing_red = false;
  bool is_crossing_amber = false;

  for (const auto & violation : result->violations) {
    if (violation.type == traffic_light_compliance_checker::ViolationType::RED_LIGHT) {
      is_crossing_red = true;
    } else if (violation.type == traffic_light_compliance_checker::ViolationType::AMBER_LIGHT) {
      is_crossing_amber = true;
    }
  }

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
}  // namespace autoware::trajectory_validator::plugin::traffic_rule

#include <pluginlib/class_list_macros.hpp>
namespace traffic_rule = autoware::trajectory_validator::plugin::traffic_rule;
PLUGINLIB_EXPORT_CLASS(
  traffic_rule::TrafficLightFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
