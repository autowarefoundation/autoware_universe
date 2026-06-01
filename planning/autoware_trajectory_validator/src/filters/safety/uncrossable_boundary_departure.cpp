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

#include "autoware/trajectory_validator/filters/safety/uncrossable_boundary_departure.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
UncrossableBoundaryDepartureFilter::result_t UncrossableBoundaryDepartureFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  if (const auto validate_context = validate_filter_context(context); !validate_context) {
    return tl::make_unexpected(validate_context.error());
  }

  if (!checker_) {
    checker_ = std::make_unique<boundary_departure_checker::UncrossableBoundaryChecker>(
      context.lanelet_map, params_, *vehicle_info_ptr_);
  }

  boundary_departure_checker::EgoDynamicState ego_state;
  ego_state.pose_with_cov = context.odometry->pose;
  ego_state.velocity = context.odometry->twist.twist.linear.x;
  ego_state.acceleration = context.acceleration->accel.accel.linear.x;
  ego_state.current_time_s = rclcpp::Time(context.odometry->header.stamp).seconds();

  auto status = checker_->update_departure_status(traj_points, ego_state);

  const bool is_feasible = status.status != boundary_departure_checker::DepartureType::CRITICAL;

  if (!is_feasible) {
    std::move(
      status.debug_markers.markers.begin(), status.debug_markers.markers.end(),
      std::back_inserter(debug_markers_.markers));
  }

  std::vector<MetricReport> metrics{
    autoware_trajectory_validator::build<MetricReport>()
      .validator_name(get_name())
      .validator_category(category())
      .metric_name("check_critical_departure")
      .metric_value(is_feasible ? 1.0 : 0.0)
      .level(!is_feasible ? MetricReport::ERROR : MetricReport::OK)};

  return ValidationResult{is_feasible, std::move(metrics)};
}

void UncrossableBoundaryDepartureFilter::update_parameters(const validator::Params & params)
{
  params_.lateral_margin_m = params.boundary_departure.lateral_margin_m;
  params_.longitudinal_margin_m = params.boundary_departure.longitudinal_margin_m;
  params_.max_deceleration_mps2 = params.boundary_departure.max_deceleration_mps2;
  params_.max_jerk_mps3 = params.boundary_departure.max_jerk_mps3;
  params_.brake_delay_s = params.boundary_departure.brake_delay_s;
  params_.time_to_departure_cutoff_s = params.boundary_departure.time_to_departure_cutoff_s;
  params_.on_time_buffer_s = params.boundary_departure.on_time_buffer_s;
  params_.off_time_buffer_s = params.boundary_departure.off_time_buffer_s;
  params_.enable_developer_marker = params.boundary_departure.enable_developer_marker;
  params_.boundary_types_to_detect = params.boundary_departure.boundary_types;

  if (checker_) {
    checker_->update_parameters(params_);
  }
}

tl::expected<void, std::string> UncrossableBoundaryDepartureFilter::validate_filter_context(
  const FilterContext & context) const
{
  if (!context.lanelet_map || context.lanelet_map->lineStringLayer.empty()) {
    return tl::make_unexpected("Lanelet map is not available in the context.");
  }

  if (!vehicle_info_ptr_) {
    return tl::make_unexpected("Vehicle info is not set.");
  }

  return {};
}
}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::UncrossableBoundaryDepartureFilter,
  autoware::trajectory_validator::plugin::ValidatorInterface)
