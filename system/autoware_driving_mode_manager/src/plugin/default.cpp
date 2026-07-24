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

#include "default.hpp"

#include <string>
#include <vector>

namespace autoware::driving_mode_manager
{

constexpr auto StopMode = AutowareMode{1001};
constexpr auto AutonomousMode = AutowareMode{1002};
constexpr auto LocalMode = AutowareMode{1003};
constexpr auto RemoteMode = AutowareMode{1004};
constexpr auto EmergencyStop = AutowareMode{2001};
constexpr auto ComfortableStop = AutowareMode{2002};
constexpr auto MainTrajectory = TrajectorySource{101};
constexpr auto StopCommand = CommandSource{11};
constexpr auto MainCommand = CommandSource{12};
constexpr auto LocalCommand = CommandSource{13};
constexpr auto RemoteCommand = CommandSource{14};
constexpr auto EmergencyStopCommand = CommandSource{21};

AutowareMode DefaultPlugin::decide(const RequestModes & modes, const AutowareModeSet & available)
{
  std::vector<AutowareMode> candidates;

  // Select the highest priority mode based on the MRM strategy.
  switch (modes.mrm_strategy) {
    case MrmStrategy::kNone:
      // If there is no MRM request, set the operation mode to the highest priority.
      candidates.push_back(modes.operation_mode);
      break;
    case MrmStrategy::kBehavior:
      // If specific MRM behavior is requested, set it as the highest priority.
      candidates.push_back(modes.mrm_behavior);
      break;
    case MrmStrategy::kDelegate:
      // If the selection of MRM behavior is delegated, follow the default MRM priority.
      break;
    case MrmStrategy::kUnknown:
      // If the MRM strategy is unknown, follow the default MRM priority.
      break;
  }

  // Set the MRM behaviors in order of priority.
  candidates.push_back(ComfortableStop);
  candidates.push_back(EmergencyStop);

  // Return the first available mode from the candidates list.
  for (const auto & mode : candidates) {
    if (available.count(mode)) {
      return mode;
    }
  }

  // Return EmergencyStop as a fallback if no available mode is found,
  return EmergencyStop;
};

void DefaultPlugin::setup(DrivingModeConfigInterface & config) const
{
  config.define_autoware_mode(StopMode, OperationMode::kStop);
  config.define_autoware_mode(AutonomousMode, OperationMode::kAutonomous);
  config.define_autoware_mode(LocalMode, OperationMode::kLocal);
  config.define_autoware_mode(RemoteMode, OperationMode::kRemote);

  config.define_autoware_mode(EmergencyStop, MrmBehavior{2});
  config.define_autoware_mode(ComfortableStop, MrmBehavior{3});

  config.define_trajectory_source(MainTrajectory);

  config.define_command_source(StopCommand);
  config.define_command_source(MainCommand);
  config.define_command_source(LocalCommand);
  config.define_command_source(RemoteCommand);
  config.define_command_source(EmergencyStopCommand);

  config.set_ignore_flags(StopMode, {false, true, true, false});
  config.set_ignore_flags(AutonomousMode, {false, true, false, false});
  config.set_ignore_flags(LocalMode, {false, true, true, false});
  config.set_ignore_flags(RemoteMode, {false, true, true, false});
  config.set_ignore_flags(EmergencyStop, {false, false, true, false});
  config.set_ignore_flags(ComfortableStop, {false, false, true, false});

  config.bind_name(AutonomousMode, "autonomous");
  config.bind_name(EmergencyStop, "emergency_stop");
  config.bind_name(ComfortableStop, "comfortable_stop");

  config.bind_gates(StopMode, {MainTrajectory, StopCommand});
  config.bind_gates(AutonomousMode, {MainTrajectory, MainCommand});
  config.bind_gates(LocalMode, {std::nullopt, LocalCommand});
  config.bind_gates(RemoteMode, {std::nullopt, RemoteCommand});
  config.bind_gates(EmergencyStop, {std::nullopt, EmergencyStopCommand});
  config.bind_gates(ComfortableStop, {MainTrajectory, MainCommand});
}

}  // namespace autoware::driving_mode_manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::driving_mode_manager::DefaultPlugin, autoware::driving_mode_manager::Plugin)
