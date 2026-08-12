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

#ifndef PLANNING__AUTOWARE_TRAJECTORY_PROCESSOR__TESTS__TRAJECTORY_PROCESSOR_TEST_UTILS_HPP_
#define PLANNING__AUTOWARE_TRAJECTORY_PROCESSOR__TESTS__TRAJECTORY_PROCESSOR_TEST_UTILS_HPP_

#include "autoware/trajectory_processor/trajectory_processor_plugin_base.hpp"

namespace autoware::trajectory_processor::test
{

/// @brief Process a trajectory and report whether the plugin modified it.
template <class Plugin>
bool process_plugin(
  Plugin & processor_plugin, plugin::TrajectoryPoints & trajectory, TrajectoryProcessorData data)
{
  return processor_plugin.process(trajectory, data) == plugin::ProcessingResult::Modified;
}

}  // namespace autoware::trajectory_processor::test

#endif  // PLANNING__AUTOWARE_TRAJECTORY_PROCESSOR__TESTS__TRAJECTORY_PROCESSOR_TEST_UTILS_HPP_
