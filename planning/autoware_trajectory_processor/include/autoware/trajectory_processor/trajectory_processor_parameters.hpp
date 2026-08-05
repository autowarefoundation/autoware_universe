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

// Compatibility-first parameter superset. Keeping the generated parameter structures intact
// preserves their existing ROS names, validation, and defaults during the plugin API migration.
struct TrajectoryProcessorParams
{
  TrajectoryProcessorParams() = default;

  explicit TrajectoryProcessorParams(trajectory_modifier_params::Params params)
  : modifier{std::move(params)}
  {
  }

  explicit TrajectoryProcessorParams(trajectory_optimizer_node_params::Params params)
  : optimizer{std::move(params)}
  {
  }

  trajectory_modifier_params::Params modifier;
  trajectory_optimizer_node_params::Params optimizer;
};

}  // namespace autoware::trajectory_processor

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_PARAMETERS_HPP_
