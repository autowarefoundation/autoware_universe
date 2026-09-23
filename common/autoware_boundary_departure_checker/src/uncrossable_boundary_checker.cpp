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

#include "autoware/boundary_departure_checker/uncrossable_boundary_checker.hpp"

#include "autoware/boundary_departure_checker/detail/debug.hpp"
#include "autoware/boundary_departure_checker/detail/footprints_generator.hpp"
#include "autoware/boundary_departure_checker/detail/severity_evaluator.hpp"

#include <autoware_utils_system/stop_watch.hpp>

#include <memory>

namespace autoware::boundary_departure_checker
{
UncrossableBoundaryChecker::UncrossableBoundaryChecker(
  const lanelet::LaneletMapPtr & map, const UncrossableBoundaryDepartureParam & param,
  const VehicleInfo & vehicle_info)
: param_(param), vehicle_info_(vehicle_info)
{
  evaluator_ptr_ = std::make_unique<BoundaryDepartureEvaluator>(map, param, vehicle_info);
}

void UncrossableBoundaryChecker::update_parameters(const UncrossableBoundaryDepartureParam & param)
{
  param_ = param;
  evaluator_ptr_->update_parameters(param);
}

DepartureResult UncrossableBoundaryChecker::update_departure_status(
  const TrajectoryPoints & predicted_traj, const EgoDynamicState & ego_state,
  HysteresisState & state)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  DepartureResult result;
  if (predicted_traj.empty()) {
    return result;
  }

  const auto footprints =
    footprints::generate(predicted_traj, vehicle_info_, ego_state.pose_with_cov);
  const auto footprints_sides = footprints::get_sides_from_footprints(footprints);

  const auto evaluation_result =
    evaluator_ptr_->evaluate(predicted_traj, footprints_sides, ego_state);

  const auto hysteresis_result =
    update_and_judge(state, evaluation_result, param_, ego_state.current_time_s);

  state = hysteresis_result.updated_state;

  // CRITICAL is hysteresis filtered, whereas NEAR_BOUNDARY is a non-latching advisory taken
  // directly from the current evaluation. A CRITICAL projection that the hysteresis buffer still
  // suppresses is reported as NEAR_BOUNDARY, so the reported severity never dips back to NONE while
  // ego closes on the boundary.
  const auto footprint_is_close_to_bound =
    evaluation_result && (severity_evaluator::is_near_boundary(*evaluation_result) ||
                          severity_evaluator::is_critical(*evaluation_result));
  result.status = hysteresis_result.status;
  if (result.status == DepartureType::NONE && footprint_is_close_to_bound) {
    result.status = DepartureType::NEAR_BOUNDARY;
  }

  if (evaluation_result) {
    result.lat_dist_to_uncrossable_bound =
      severity_evaluator::get_min_lateral_distance_to_bound(*evaluation_result);
  }

  result.debug_markers =
    debug::create_debug_markers(state, footprints, ego_state, param_.enable_developer_marker);
  return result;
}

}  // namespace autoware::boundary_departure_checker
