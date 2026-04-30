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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__TRAJECTORY_PIPELINE_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__TRAJECTORY_PIPELINE_HPP_

#include "autoware/trajectory_validator/evaluation_context.hpp"
#include "autoware/trajectory_validator/pipeline_report.hpp"
#include "autoware/trajectory_validator/validator_interface.hpp"

#include <memory>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator
{

class TrajectoryPipeline
{
public:
  explicit TrajectoryPipeline(std::vector<std::shared_ptr<plugin::ValidatorInterface>> validators)
  : validators_(std::move(validators))
  {
  }

  [[nodiscard]] PipelineReport process(
    const autoware_internal_planning_msgs::msg::CandidateTrajectories & input_trajectories,
    const EvaluationContext & context) const;

private:
  std::vector<std::shared_ptr<plugin::ValidatorInterface>> validators_;
};

}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__TRAJECTORY_PIPELINE_HPP_
