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

#ifndef IN_LANE_MRM_TRAJECTORY_VALIDATOR_HPP_
#define IN_LANE_MRM_TRAJECTORY_VALIDATOR_HPP_

#include "type_alias.hpp"

#include <cstdint>
#include <string>

namespace autoware::in_lane_mrm_planner
{

class InLaneMrmTrajectoryValidator
{
public:
  enum class FailureCode : int32_t {
    NONE = 0,
    INSUFFICIENT_POINT_COUNT = 1,
    NON_FINITE_VALUES = 2,
  };

  struct ValidationResult
  {
    bool ok{false};
    FailureCode code{FailureCode::NONE};
    std::string reason;
  };

  explicit InLaneMrmTrajectoryValidator(const Params & params);

  void update_params(const Params & params);

  ValidationResult validate(const TrajectoryPoints & points) const;

private:
  using TrajectoryValidatorParams = Params::TrajectoryValidator;

  bool has_finite_values(const TrajectoryPoints & points) const;

  TrajectoryValidatorParams params_;
};

}  // namespace autoware::in_lane_mrm_planner

#endif  // IN_LANE_MRM_TRAJECTORY_VALIDATOR_HPP_
