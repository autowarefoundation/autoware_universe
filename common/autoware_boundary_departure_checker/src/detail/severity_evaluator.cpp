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
#include <cmath>
#include <limits>
#include <optional>

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
  thresholds.th_lat_critical = param.critical_departure_lateral_th_m;
  // A near boundary threshold below the critical margin would leave projections inside the critical
  // margin unclassified, so the advisory band always covers at least the critical margin.
  thresholds.th_near_boundary =
    std::max(param.near_boundary_lateral_th_m, param.critical_departure_lateral_th_m);

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

    if (out.back().is_critical()) {
      return out;
    }
  }

  return out;
}

std::optional<DeparturePointPair> apply_backward_buffer_and_filter(
  const ProjectionsToBound & side_value, const UncrossableBoundaryDepartureParam & param)
{
  if (side_value.empty() || !side_value.back().is_critical()) return std::nullopt;

  DeparturePointPair result;
  const auto & departure_point = side_value.back();

  result.physical_departure_point = departure_point;
  result.safety_buffer_start = departure_point;

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
  const auto footprint_overlaps_critical_margin = metrics.lat_dist <= thresholds.th_lat_critical;
  const auto ego_cannot_stop_before_departure =
    metrics.lon_dist_to_departure <= thresholds.min_braking_distance ||
    metrics.time_from_start <= thresholds.cutoff_time;

  if (footprint_overlaps_critical_margin && ego_cannot_stop_before_departure) {
    return DepartureType::CRITICAL;
  }

  if (metrics.lat_dist <= thresholds.th_near_boundary) {
    return DepartureType::NEAR_BOUNDARY;
  }

  return DepartureType::NONE;
}

Side<std::optional<DeparturePointPair>> evaluate_projections_severity(
  const Side<ProjectionsToBound> & projections_to_bound,
  const UncrossableBoundaryDepartureParam & param, const EgoDynamicState & ego_state,
  const vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const auto min_braking_dist = calc_minimum_braking_distance(ego_state, param, vehicle_info);

  return projections_to_bound.transform_each_side(
    [&](const auto & side_value) -> std::optional<DeparturePointPair> {
      const auto min_to_bounds =
        filter_and_assign_departure_types(side_value, param, min_braking_dist);

      if (min_to_bounds.empty()) return std::nullopt;

      if (min_to_bounds.back().is_critical()) {
        return apply_backward_buffer_and_filter(min_to_bounds, param);
      }

      const auto closest_to_bound = std::min_element(
        min_to_bounds.begin(), min_to_bounds.end(),
        [](const ProjectionToBound & lhs, const ProjectionToBound & rhs) {
          return lhs.lat_dist < rhs.lat_dist;
        });

      return DeparturePointPair{*closest_to_bound, *closest_to_bound};
    });
}

bool is_critical(const Side<std::optional<DeparturePointPair>> & evaluated_projections)
{
  return evaluated_projections.any_of_side([](const auto & critical_pair_opt) {
    return critical_pair_opt.has_value() &&
           critical_pair_opt->physical_departure_point.is_critical();
  });
}

bool is_near_boundary(const Side<std::optional<DeparturePointPair>> & evaluated_projections)
{
  return evaluated_projections.any_of_side([](const auto & departure_pair_opt) {
    return departure_pair_opt.has_value() &&
           departure_pair_opt->physical_departure_point.is_near_boundary();
  });
}

double get_min_lateral_distance_to_bound(
  const Side<std::optional<DeparturePointPair>> & evaluated_projections)
{
  auto min_lat_dist = std::numeric_limits<double>::infinity();
  evaluated_projections.for_each_side([&min_lat_dist](const auto & departure_pair_opt) {
    if (!departure_pair_opt.has_value()) return;
    // A projection that failed to find a segment keeps its sentinel default. Treat it as "no
    // measurement" instead of letting the sentinel be published as a real distance.
    const auto lat_dist = departure_pair_opt->physical_departure_point.lat_dist;
    if (!std::isfinite(lat_dist) || lat_dist >= std::numeric_limits<double>::max()) return;
    min_lat_dist = std::min(min_lat_dist, lat_dist);
  });
  return min_lat_dist;
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
