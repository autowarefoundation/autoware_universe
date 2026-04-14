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

#include "autoware/trajectory_validator/filters/safety/uncrossable_boundary_departure_filter.hpp"

#include <autoware/deprecated/boundary_departure_checker/debug.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
UncrossableBoundaryDepartureFilter::result_t UncrossableBoundaryDepartureFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  if (const auto has_invalid_input = is_invalid_input(traj_points, context)) {
    return tl::make_unexpected(*has_invalid_input);
  }

  if (!uncrossable_boundary_departure_checker_ptr_) {
    uncrossable_boundary_departure_checker_ptr_ =
      std::make_unique<boundary_departure_checker::UncrossableBoundaryDepartureChecker>(
        clock_, context.lanelet_map, *vehicle_info_ptr_);
  }

  auto departure_data = uncrossable_boundary_departure_checker_ptr_->get_departure_data(
    traj_points, traj_points, context.odometry->pose, context.odometry->twist.twist.linear.x,
    context.acceleration->accel.accel.linear.x);

  if (!departure_data) {
    warn_throttle("%s", departure_data.error().c_str());
    return tl::make_unexpected(departure_data.error());
  }

  bool is_feasible = true;
  const auto found_critical_departure = !departure_data->critical_departure_points.empty();
  if (found_critical_departure) {
    is_feasible = false;

    debug_markers_ = boundary_departure_checker::debug::create_debug_markers(
      *departure_data, context.odometry->header.stamp, context.odometry->pose.pose.position.z,
      params_);
  }

  std::vector<MetricReport> metrics{
    autoware_trajectory_validator::build<MetricReport>()
      .validator_name(get_name())
      .validator_category(category())
      .metric_name("check_critical_departure")
      .metric_value(0.0)
      .level(found_critical_departure ? MetricReport::ERROR : MetricReport::OK)};

  return ValidationResult{is_feasible, std::move(metrics)};
}

void UncrossableBoundaryDepartureFilter::update_parameters(const validator::Params & params)
{
  params_.th_trigger.th_dist_to_boundary_m.left.min =
    params.boundary_departure.lateral_gap_to_boundary_m;
  params_.th_trigger.th_dist_to_boundary_m.right.min =
    params.boundary_departure.lateral_gap_to_boundary_m;
  params_.min_braking_distance = params.boundary_departure.longitudinal_gap_to_boundary_m;
  params_.th_trigger.th_acc_mps2.max = params.boundary_departure.max_deceleration_mps2;
  params_.th_trigger.th_jerk_mps3.max = params.boundary_departure.max_jerk_mps3;
  params_.th_trigger.brake_delay_s = params.boundary_departure.brake_delay_s;
  params_.th_cutoff_time_departure_s = params.boundary_departure.cutoff_time_s;

  if (uncrossable_boundary_departure_checker_ptr_) {
    uncrossable_boundary_departure_checker_ptr_->set_param(params_);
  }
}

std::optional<std::string> UncrossableBoundaryDepartureFilter::is_invalid_input(
  const TrajectoryPoints & traj_points, const FilterContext & context) const
{
  if (traj_points.empty()) {
    return "Trajectory points are empty.";
  }

  if (!context.lanelet_map) {
    return "Lanelet map is not available in the context.";
  }

  if (!vehicle_info_ptr_) {
    return "Vehicle info is not set.";
  }

  return std::nullopt;
}
}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::UncrossableBoundaryDepartureFilter,
  autoware::trajectory_validator::plugin::ValidatorInterface)
