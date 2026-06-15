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

// DEBUG
#include <rclcpp/logging.hpp>

namespace autoware::driving_mode_manager
{

const auto logger = rclcpp::get_logger("DefaultPlugin");
constexpr auto StopMode = AutowareMode{1001};
constexpr auto AutonomousMode = AutowareMode{1002};
constexpr auto LocalMode = AutowareMode{1003};
constexpr auto RemoteMode = AutowareMode{1004};
constexpr auto EmergencyStop = AutowareMode{2001};
constexpr auto ComfortableStop = AutowareMode{2002};
constexpr auto UnknownMode = AutowareMode{0};
constexpr auto MainTrajectory = TrajectorySource{100};
constexpr auto StopCommand = CommandSource{11};
constexpr auto MainCommand = CommandSource{12};
constexpr auto LocalCommand = CommandSource{13};
constexpr auto RemoteCommand = CommandSource{14};
constexpr auto EmergencyStopCommand = CommandSource{21};

void print_modes(const std::string & title, const std::vector<AutowareMode> & modes)
{
  std::string text;
  for (const auto & mode : modes) {
    text = text + " " + std::to_string(mode.id);
  }
  RCLCPP_INFO_STREAM(logger, title << ":" << text);
}

AutowareMode DefaultPlugin::decide(const RequestModes & modes, const AutowareModeSet & availables)
{
  std::vector<AutowareMode> candidates;

  if (modes.mrm_strategy == MrmStrategy::kNone) {
    candidates.push_back(modes.operation_mode);
  }

  candidates.push_back(ComfortableStop);
  candidates.push_back(EmergencyStop);
  // print_modes("Candidates", candidates);

  std::vector<AutowareMode> result;
  for (const auto & mode : candidates) {
    if (availables.count(mode)) result.push_back(mode);
  }

  // print_modes("Availables", result);
  return result.empty() ? EmergencyStop : result.front();
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

  config.bind_name(EmergencyStop, "emergency_stop");
  config.bind_name(ComfortableStop, "comfortable_stop");

  config.bind_gates(StopMode, {std::nullopt, StopCommand});
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
