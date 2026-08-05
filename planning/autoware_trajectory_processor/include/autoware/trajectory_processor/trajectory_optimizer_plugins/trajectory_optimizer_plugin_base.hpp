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

// NOLINTNEXTLINE
#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_OPTIMIZER_PLUGIN_BASE_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_OPTIMIZER_PLUGIN_BASE_HPP_
#include "autoware/trajectory_processor/trajectory_processor_plugin_base.hpp"

namespace autoware::trajectory_optimizer::plugin
{
using TrajectoryOptimizerPluginBase =
  autoware::trajectory_processor::plugin::TrajectoryProcessorPluginBase;
using ProcessingResult = autoware::trajectory_processor::plugin::ProcessingResult;
using TrajectoryPoints = autoware::trajectory_processor::plugin::TrajectoryPoints;
using TrajectoryProcessorData = autoware::trajectory_processor::TrajectoryProcessorData;
using TrajectoryProcessorParams = autoware::trajectory_processor::TrajectoryProcessorParams;
}  // namespace autoware::trajectory_optimizer::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_OPTIMIZER_PLUGIN_BASE_HPP_
