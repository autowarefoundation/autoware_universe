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

// NOTE: This file implements an ad-hoc pseudo emergency stop fallback used for evaluation only.
// It is intended to be removed once the proper MRM is implemented.

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__PSEUDO_EMERGENCY_STOP_HANDLER_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__PSEUDO_EMERGENCY_STOP_HANDLER_HPP_

#include "autoware/trajectory_validator/detail/trajectory_validator_report.hpp"
#include "autoware/trajectory_validator/detail/validator_context.hpp"

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_trajectory_validator/autoware_trajectory_validator_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::trajectory_validator
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::CandidateTrajectory;

class PseudoEmergencyStopHandler
{
public:
  explicit PseudoEmergencyStopHandler(rclcpp::Node & node);

  void handle(
    const CandidateTrajectories & input_trajectories, CandidateTrajectories & filtered_trajectories,
    const std::vector<EvaluationTable> & evaluation_tables, const FilterContext & context,
    const validator::Params & params);

private:
  bool is_pseudo_emergency_stop_triggered(
    const std::vector<EvaluationTable> & evaluation_tables) const;
  void update_pseudo_emergency_stop_state(
    bool triggered, double ego_velocity_mps, const validator::Params & params);
  void cache_fallback_trajectory(
    const CandidateTrajectories & input_trajectories,
    const std::vector<EvaluationTable> & evaluation_tables);
  void apply_pseudo_emergency_stop_fallback(
    CandidateTrajectories & filtered_trajectories, const FilterContext & context,
    const validator::Params & params);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  bool pseudo_emergency_stop_active_{false};
  std::optional<CandidateTrajectory> cached_fallback_trajectory_{std::nullopt};
  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;
};

}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__PSEUDO_EMERGENCY_STOP_HANDLER_HPP_
