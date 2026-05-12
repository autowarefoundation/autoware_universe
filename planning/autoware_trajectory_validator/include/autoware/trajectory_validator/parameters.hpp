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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__PARAMETERS_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__PARAMETERS_HPP_

#include <autoware_trajectory_concatenator/autoware_trajectory_concatenator_param.hpp>
#include <autoware_trajectory_validator/autoware_trajectory_validator_param.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include <string>

namespace autoware::trajectory_validator
{
struct TrajectoryValidatorParam
{
  explicit TrajectoryValidatorParam(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface)
  : validator_params_listener(node_parameters_interface),
    concatenator_params_listener(node_parameters_interface)
  {
    validator_params = validator_params_listener.get_params();
    concatenator_params = concatenator_params_listener.get_params();
  }

  validator::ParamListener validator_params_listener;
  validator::Params validator_params;
  concatenator::ParamListener concatenator_params_listener;
  concatenator::Params concatenator_params;

  std::optional<std::string> update_parameters()
  {
    std::string status;
    if (validator_params_listener.is_old(validator_params)) {
      validator_params = validator_params_listener.get_params();
      status += "Validator parameters updated. ";
    }

    if (concatenator_params_listener.is_old(concatenator_params)) {
      concatenator_params = concatenator_params_listener.get_params();
      status += "Concatenator parameters updated. ";
    }

    if (status.empty()) {
      return std::nullopt;
    }

    return status;
  }
};
}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__PARAMETERS_HPP_
