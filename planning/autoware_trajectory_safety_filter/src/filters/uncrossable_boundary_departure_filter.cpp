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

#include "autoware/trajectory_safety_filter/filters/uncrossable_boundary_departure_filter.hpp"

#include <memory>
#include <string>

namespace autoware::trajectory_safety_filter::plugin
{
bool UncrossableBoundaryDepartureFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  if (const auto has_invalid_input = is_invalid_input(traj_points, context)) {
    warn_throttle("%s", has_invalid_input->c_str());
    return false;
  }

  if (!uncrossable_boundary_departure_checker_ptr_) {
    uncrossable_boundary_departure_checker_ptr_ =
      std::make_unique<boundary_departure_checker::UncrossableBoundaryDepartureChecker>(
        clock_, context.lanelet_map, *vehicle_info_ptr_);
  }

  const auto departure_data = uncrossable_boundary_departure_checker_ptr_->get_departure_data(
    traj_points, traj_points, context.odometry->pose, context.odometry->twist.twist.linear.x,
    context.acceleration->accel.accel.linear.x);

  if (!departure_data) {
    warn_throttle("%s", departure_data.error().c_str());
    return false;
  }

  return departure_data->critical_departure_points.empty();
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
}  // namespace autoware::trajectory_safety_filter::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_safety_filter::plugin::UncrossableBoundaryDepartureFilter,
  autoware::trajectory_safety_filter::plugin::SafetyFilterInterface)
