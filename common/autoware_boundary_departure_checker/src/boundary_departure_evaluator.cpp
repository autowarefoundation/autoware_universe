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

#include "autoware/boundary_departure_checker/boundary_departure_evaluator.hpp"

#include "autoware/boundary_departure_checker/boundary_segment_finder.hpp"
#include "autoware/boundary_departure_checker/severity_evaluator.hpp"

#include <stdexcept>

namespace autoware::boundary_departure_checker
{
BoundaryDepartureEvaluator::BoundaryDepartureEvaluator(
  const lanelet::LaneletMapPtr & map, const UncrossableBoundaryDepartureParam & param,
  const vehicle_info_utils::VehicleInfo & vehicle_info)
: map_(map),
  param_(param),
  vehicle_info_(vehicle_info),
  rtree_(*map, param.boundary_types_to_detect)
{
  if (!map) {
    throw std::runtime_error("Map is NULL");
  }

  if (map->lineStringLayer.empty()) {
    throw std::runtime_error("Map without any linestrings.");
  }
}

std::optional<Side<std::optional<CriticalPointPair>>> BoundaryDepartureEvaluator::evaluate(
  const TrajectoryPoints & predicted_traj, const FootprintSideSegmentsArray & footprints_sides,
  const EgoDynamicState & ego_state) const
{
  if (predicted_traj.empty() || rtree_.empty()) {
    return std::nullopt;
  }

  const auto boundary_segments = boundary_segment_finder::get_boundary_segments(
    rtree_, map_, param_, footprints_sides, predicted_traj, vehicle_info_.vehicle_height_m);

  if (boundary_segments.all_empty()) {
    return std::nullopt;
  }

  const auto projections_to_bound =
    boundary_segment_finder::get_closest_boundary_segments_from_side(
      predicted_traj, boundary_segments, footprints_sides);

  return severity_evaluator::evaluate_projections_severity(
    projections_to_bound, param_, ego_state, vehicle_info_);
}

}  // namespace autoware::boundary_departure_checker
