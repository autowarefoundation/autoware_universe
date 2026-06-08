// Copyright 2026 The Autoware Contributors
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

#ifndef AUTOWARE_DRIVING_MODE_MANAGER__CONFIG_HPP_
#define AUTOWARE_DRIVING_MODE_MANAGER__CONFIG_HPP_

#include "types.hpp"

#include <optional>

namespace autoware::driving_mode_manager
{

struct DrivingModeConfigInterface
{
  struct Gates
  {
    std::optional<TrajectorySource> trajectory;
    std::optional<CommandSource> command;
  };

  virtual ~DrivingModeConfigInterface() = default;
  virtual void define_autoware_mode(const AutowareMode & mode, const OperationMode & opmode) = 0;
  virtual void define_autoware_mode(const AutowareMode & mode, const MrmBehavior & behavior) = 0;
  virtual void define_trajectory_source(const TrajectorySource & source) = 0;
  virtual void define_command_source(const CommandSource & source) = 0;
  virtual void bind_gates(const AutowareMode & mode, const Gates & gates) = 0;
};

}  // namespace autoware::driving_mode_manager

#endif  // AUTOWARE_DRIVING_MODE_MANAGER__CONFIG_HPP_
