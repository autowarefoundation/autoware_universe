// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__PLANNING_VALIDATOR_TRAJECTORY_CHECKER__TRAJECTORY_CHECKER_HPP_
#define AUTOWARE__PLANNING_VALIDATOR_TRAJECTORY_CHECKER__TRAJECTORY_CHECKER_HPP_

#include "autoware/planning_validator_trajectory_checker/parameters.hpp"

#include <autoware/planning_validator/plugin_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace autoware::planning_validator
{

class TrajectoryChecker : public PluginInterface
{
public:
  void init(
    rclcpp::Node & node, const std::string & name,
    const std::shared_ptr<PlanningValidatorContext> & context) override;
  void validate(bool & is_critical) override;
  void setup_diag() override;
  std::string get_module_name() const override { return module_name_; };

  bool check_valid_finite_value(const std::shared_ptr<const PlanningValidatorData> & data);
  bool check_valid_size(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool check_valid_interval(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool check_valid_relative_angle(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool check_valid_curvature(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool check_valid_lateral_acceleration(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool check_valid_lateral_jerk(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool check_valid_max_longitudinal_acceleration(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool check_valid_min_longitudinal_acceleration(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool check_valid_steering(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status, const double vehicle_wheel_base_m);
  bool check_valid_steering_rate(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status, const double vehicle_wheel_base_m);
  bool check_valid_velocity_deviation(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool check_valid_distance_deviation(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool check_valid_longitudinal_distance_deviation(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool check_valid_forward_trajectory_length(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool check_valid_yaw_deviation(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);
  bool check_trajectory_shift(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status);

private:
  void setup_parameters(rclcpp::Node & node);

  bool is_critical_error_ = false;

  TrajectoryCheckerParams params_;
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR_TRAJECTORY_CHECKER__TRAJECTORY_CHECKER_HPP_
