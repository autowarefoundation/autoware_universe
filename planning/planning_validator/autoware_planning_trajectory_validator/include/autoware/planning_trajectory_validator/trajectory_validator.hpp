// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__PLANNING_TRAJECTORY_VALIDATOR__TRAJECTORY_VALIDATOR_HPP_
#define AUTOWARE__PLANNING_TRAJECTORY_VALIDATOR__TRAJECTORY_VALIDATOR_HPP_

#include "autoware/planning_trajectory_validator/parameters.hpp"

#include <autoware/planning_validator/plugin_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::planning_validator
{

class TrajectoryValidator : public PluginInterface
{
public:
  void init(rclcpp::Node & node, const std::string & name) override;
  void validate(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status, bool & is_critical) override;
  std::string get_module_name() const override { return module_name_; };

private:
  void setupParameters(rclcpp::Node & node);

  bool checkValidFiniteValue(const std::shared_ptr<const PlanningValidatorData> & data);
  bool checkValidSize(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool checkValidInterval(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool checkValidRelativeAngle(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool checkValidCurvature(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool checkValidLateralAcceleration(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool checkValidLateralJerk(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool checkValidMaxLongitudinalAcceleration(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool checkValidMinLongitudinalAcceleration(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool checkValidSteering(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool checkValidSteeringRate(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool checkValidVelocityDeviation(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool checkValidDistanceDeviation(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool checkValidLongitudinalDistanceDeviation(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool checkValidForwardTrajectoryLength(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool checkValidYawDeviation(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool checkTrajectoryShift(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);

  bool is_critical_error_ = false;

  std::string module_name_;
  rclcpp::Clock::SharedPtr clock_{};

  TrajectoryValidatorParams params_;
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_TRAJECTORY_VALIDATOR__TRAJECTORY_VALIDATOR_HPP_
