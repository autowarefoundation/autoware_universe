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

#include "in_lane_mrm_trajectory_validator.hpp"

#include <cmath>

namespace autoware::in_lane_mrm_planner
{

InLaneMrmTrajectoryValidator::InLaneMrmTrajectoryValidator(const Params & params)
: params_(params.trajectory_validator)
{
}

void InLaneMrmTrajectoryValidator::update_params(const Params & params)
{
  params_ = params.trajectory_validator;
}

bool InLaneMrmTrajectoryValidator::has_finite_values(const TrajectoryPoints & points) const
{
  for (const auto & point : points) {
    if (
      !std::isfinite(point.pose.position.x) || !std::isfinite(point.pose.position.y) ||
      !std::isfinite(point.pose.position.z) || !std::isfinite(point.pose.orientation.w) ||
      !std::isfinite(point.pose.orientation.x) || !std::isfinite(point.pose.orientation.y) ||
      !std::isfinite(point.pose.orientation.z) || !std::isfinite(point.longitudinal_velocity_mps) ||
      !std::isfinite(point.lateral_velocity_mps) || !std::isfinite(point.acceleration_mps2)) {
      return false;
    }
  }
  return true;
}

InLaneMrmTrajectoryValidator::ValidationResult InLaneMrmTrajectoryValidator::validate(
  const TrajectoryPoints & points) const
{
  ValidationResult result;
  result.ok = true;

  if (!params_.enable) {
    return result;
  }

  // min_point_count is validated (gt<>: [1]) to be >= 2, so the size_t cast cannot wrap.
  if (points.size() < static_cast<size_t>(params_.min_point_count)) {
    result.ok = false;
    result.code = FailureCode::INSUFFICIENT_POINT_COUNT;
    result.reason = "trajectory has too few points";
    return result;
  }

  if (!has_finite_values(points)) {
    result.ok = false;
    result.code = FailureCode::NON_FINITE_VALUES;
    result.reason = "trajectory contains non-finite values";
    return result;
  }

  return result;
}

}  // namespace autoware::in_lane_mrm_planner
