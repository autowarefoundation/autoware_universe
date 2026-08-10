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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_PARAMETERS_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_PARAMETERS_HPP_

#include <autoware_trajectory_processor/trajectory_modifier_param.hpp>
#include <autoware_trajectory_processor/trajectory_optimizer_param.hpp>

#include <utility>

namespace autoware::trajectory_processor
{

/// @brief Compatibility parameter superset for modifier and optimizer plugins.
///
/// Keeping the generated structures intact preserves existing ROS names, validation, and defaults
/// during the plugin API migration.
struct TrajectoryProcessorParams : public trajectory_modifier_params::Params,
                                   public trajectory_optimizer_node_params::Params
{
  /// @brief Construct both parameter groups with their generated defaults.
  TrajectoryProcessorParams() = default;

  /// @brief Construct the superset from modifier parameters.
  explicit TrajectoryProcessorParams(trajectory_modifier_params::Params params)
  : trajectory_modifier_params::Params{std::move(params)}
  {
  }

  /// @brief Construct the superset from optimizer parameters.
  explicit TrajectoryProcessorParams(trajectory_optimizer_node_params::Params params)
  : trajectory_optimizer_node_params::Params{std::move(params)}
  {
  }

  /// @brief Return the modifier portion of the common parameter snapshot.
  [[nodiscard]] trajectory_modifier_params::Params & modifier_params() { return *this; }
  /// @brief Return the modifier portion of the common parameter snapshot.
  [[nodiscard]] const trajectory_modifier_params::Params & modifier_params() const { return *this; }
  /// @brief Return the optimizer portion of the common parameter snapshot.
  [[nodiscard]] trajectory_optimizer_node_params::Params & optimizer_params() { return *this; }
  /// @brief Return the optimizer portion of the common parameter snapshot.
  [[nodiscard]] const trajectory_optimizer_node_params::Params & optimizer_params() const
  {
    return *this;
  }
};

}  // namespace autoware::trajectory_processor

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_PARAMETERS_HPP_
