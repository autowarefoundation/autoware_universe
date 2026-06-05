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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__TRAJECTORY_VALIDATOR_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__TRAJECTORY_VALIDATOR_HPP_

#include "autoware/trajectory_validator/detail/trajectory_validator_report.hpp"
#include "autoware/trajectory_validator/detail/validator_context.hpp"
#include "autoware/trajectory_validator/validator_interface.hpp"
#include "autoware_trajectory_validator/autoware_trajectory_validator_param.hpp"

#include <memory>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator
{
using autoware::vehicle_info_utils::VehicleInfo;

/**
 * @brief Runs a set of validator plugins against each candidate trajectory.
 */
class TrajectoryValidator
{
public:
  /**
   * @brief Constructs the validator with the given plugin set.
   * @param plugins Validator plugins to run against each trajectory.
   */
  explicit TrajectoryValidator(std::vector<std::shared_ptr<plugin::ValidatorInterface>> plugins)
  : plugins_(std::move(plugins))
  {
  }

  /**
   * @brief Forwards updated parameters to all plugins.
   * @param params Latest parameter values.
   */
  void update_parameters(const validator::Params & params) const
  {
    for (const auto & plugin : plugins_) {
      plugin->update_parameters(params);
    }
  }

  /**
   * @brief Evaluates all plugins against every trajectory and returns a validation report.
   * @param input_trajectories Candidate trajectories to validate.
   * @param context Current world state snapshot.
   */
  [[nodiscard]] TrajectoryValidatorReport process(
    const autoware_internal_planning_msgs::msg::CandidateTrajectories & input_trajectories,
    const ValidatorContext & context) const;

private:
  std::vector<std::shared_ptr<plugin::ValidatorInterface>> plugins_;
};

}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__TRAJECTORY_VALIDATOR_HPP_
