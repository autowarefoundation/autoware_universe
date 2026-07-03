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

#ifndef UTIL__MOCK_HPP_
#define UTIL__MOCK_HPP_

#include "type/interface.hpp"

#include <string>

using namespace autoware::driving_mode_manager;  // NOLINT(build/namespaces)

class MockInterface : public Interface
{
public:
  void init(MainLogic * logic) override;

  bool get_enable_debug_topics() const override;

  rclcpp::Time now() const override;
  void change_trajectory_source(const TrajectorySource & source) override;
  void change_command_source(const CommandSource & source) override;
  void change_command_filter(const CommandFilter & filter) override;
  void change_platform_mode(const PlatformMode & mode) override;
  void publish_operation_mode(const OperationModeState & state) override;
  void publish_mrm_state(const MrmState & state) override;
  void publish_driving_mode_request(const ModeRequest & request) override;
  void publish_driving_mode_sync(const AutowareModeSet & modes) override;
  void publish_driving_mode_info(const ModeInfo & info) override;
  void publish_debug_flags(const DebugFlags & flags) override;
  void publish_debug_request(const RequestModes & request) override;

  void log_info(const std::string & message) override;
  void log_warn(const std::string & message) override;
  void log_error(const std::string & message) override;
  void log_debug(const std::string & message) override;

private:
  MainLogic * logic_ = nullptr;

public:
  void update();

  TrajectorySource trajectory_source = TrajectorySource{0};
  CommandSource command_source = CommandSource{0};
  CommandFilter command_filter = CommandFilter{false};
  PlatformMode platform_mode = PlatformMode::kUnknown;
  std::optional<OperationModeState> operation_mode_state;

  std::optional<TrajectorySource> response_trajectory_source;
  std::optional<CommandSource> response_command_source;
  std::optional<CommandFilter> response_command_filter;
  std::optional<PlatformMode> response_platform_mode;
};

#endif  // UTIL__MOCK_HPP_
