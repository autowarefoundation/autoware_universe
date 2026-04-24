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

#include "autoware/boundary_departure_checker/detail/severity_evaluator.hpp"

#include <autoware/motion_utils/distance/distance.hpp>

#include <algorithm>
#include <limits>

namespace autoware::boundary_departure_checker::severity_evaluator
{
ProjectionsToBound filter_and_assign_departure_types(
  const ProjectionsToBound & side_value, const UncrossableBoundaryDepartureParam & param,
  const double min_braking_dist)
{
  ProjectionsToBound out;
  out.reserve(side_value.size());

  DepartureCheckThresholds thresholds;
  thresholds.min_braking_distance = min_braking_dist;
  thresholds.cutoff_time = param.time_to_departure_cutoff_s;
  thresholds.th_lat_critical = param.lateral_margin_m;

  for (size_t idx = 0; idx < side_value.size(); ++idx) {
    const auto & original_candidate = side_value[idx];
    if (original_candidate.pose_index != idx) continue;

    ProjectionEvaluationMetrics metrics;
    metrics.lon_dist_to_departure =
      original_candidate.dist_along_trajectory_m - original_candidate.ego_front_to_proj_offset_m;
    metrics.time_from_start = original_candidate.time_from_start;
    metrics.lat_dist = original_candidate.lat_dist;

    out.push_back(original_candidate);
    out.back().departure_type = assign_departure_type(metrics, thresholds);

    if (out.back().is_critical()) break;
  }

  return out;
}

std::optional<CriticalPointPair> apply_backward_buffer_and_filter(
  const ProjectionsToBound & side_value, const UncrossableBoundaryDepartureParam & param)
{
  if (side_value.empty() || side_value.back().is_none_departure()) return std::nullopt;

  const auto & departure_point = side_value.back();

  CriticalPointPair result;
  result.physical_departure_point = departure_point;
  result.safety_buffer_start = departure_point;

  if (!departure_point.is_critical()) {
    return result;
  }

  for (auto it = std::next(side_value.rbegin()); it != side_value.rend(); ++it) {
    const double dist_between_proj =
      boost::geometry::distance(result.physical_departure_point.pt_on_ego, it->pt_on_ego);

    if (dist_between_proj >= param.longitudinal_margin_m) {
      result.safety_buffer_start = *it;
      result.safety_buffer_start.departure_type = DepartureType::CRITICAL;
      break;
    }
  }

  return result;
}

DepartureType assign_departure_type(
  const ProjectionEvaluationMetrics & metrics, const DepartureCheckThresholds & thresholds)
{
  if (metrics.lat_dist > thresholds.th_lat_critical) {
    return DepartureType::NONE;
  }

  if (
    metrics.lon_dist_to_departure > thresholds.min_braking_distance &&
    metrics.time_from_start > thresholds.cutoff_time) {
    return DepartureType::APPROACHING;
  }

  return DepartureType::CRITICAL;
}

Side<std::optional<CriticalPointPair>> evaluate_projections_severity(
  const Side<ProjectionsToBound> & projections_to_bound,
  const UncrossableBoundaryDepartureParam & param, const EgoDynamicState & ego_state,
  const vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const auto min_braking_dist = calc_minimum_braking_distance(ego_state, param, vehicle_info);

  return projections_to_bound.transform_each_side([&](const auto & side_value) {
    const auto min_to_bounds =
      filter_and_assign_departure_types(side_value, param, min_braking_dist);
    return apply_backward_buffer_and_filter(min_to_bounds, param);
  });
}

bool is_critical(const Side<std::optional<CriticalPointPair>> & evaluated_projections)
{
  return evaluated_projections.any_of_side([](const auto & critical_pair_opt) {
    return critical_pair_opt.has_value() &&
           critical_pair_opt->physical_departure_point.is_critical();
  });
}

double calc_minimum_braking_distance(
  const EgoDynamicState & ego_state, const UncrossableBoundaryDepartureParam & param,
  const vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const auto kinematic_stop_dist = motion_utils::calculate_stop_distance(
    ego_state.velocity, ego_state.acceleration, param.max_deceleration_mps2, param.max_jerk_mps3,
    param.brake_delay_s);

  return vehicle_info.front_overhang_m +
         (kinematic_stop_dist ? std::max(0.0, *kinematic_stop_dist) : 0.0);
}

}  // namespace autoware::boundary_departure_checker::severity_evaluator
