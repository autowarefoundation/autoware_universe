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

#include "autoware/boundary_departure_checker/debug.hpp"
#include "autoware/boundary_departure_checker/footprints_generator.hpp"

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
  const TrajectoryPoints & predicted_traj, const EgoDynamicState & ego_state)
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
    update_and_judge(hysteresis_state_, evaluation_result, param_, ego_state.current_time_s);

  hysteresis_state_ = hysteresis_result.updated_state;

  result.status = hysteresis_result.status;
  result.debug_markers = debug::create_debug_markers(
    hysteresis_state_, footprints, ego_state, param_.enable_developer_marker);
  return result;
}

}  // namespace autoware::boundary_departure_checker
