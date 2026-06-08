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

#ifndef CORE__INIT_HPP_
#define CORE__INIT_HPP_

#include "core/config.hpp"
#include "core/status.hpp"
#include "type/data.hpp"
#include "type/interface.hpp"

#include <autoware_driving_mode_manager/plugin.hpp>

#include <memory>
#include <optional>
#include <unordered_map>

namespace autoware::driving_mode_manager
{

class ManagerInit : public MainLogic
{
public:
  ManagerInit(std::unique_ptr<Interface> && interface, std::shared_ptr<Plugin> plugin);
  bool is_ready() const;
  GateStatusItem gates() const;

  void update() override;
  void on_trajectory_source(const TrajectorySource & source) override;
  void on_command_source(const CommandSource & source) override;
  void on_command_filter(const CommandFilter & filter) override;
  void on_vehicle_control_mode(const PlatformMode & mode) override;
  void on_available_flag(const AutowareMode & mode, bool flag) override;
  void on_stable_flag(const AutowareMode & mode, bool flag) override;
  void on_continuable_flag(const AutowareMode & mode, bool flag) override;
  void on_mrm_state(const AutowareMode & mode, const MrmState::State & state) override;
  ServiceResponse change_operation_mode(const OperationMode & operation_mode) override;
  ServiceResponse change_autoware_control(const AutowareControl & autoware_control) override;
  ServiceResponse change_mrm_request(const MrmRequest & request) override;

  std::unique_ptr<Interface> interface_;
  std::shared_ptr<Plugin> plugin_;
  std::unique_ptr<DrivingModeConfig> config_;
  std::unique_ptr<DrivingModeStatus> status_;
  std::unordered_map<AutowareMode, MrmState::State> mrm_states_;
  std::optional<PlatformMode> platform_mode;

private:
  void publish_debug_status() const;
  std::optional<TrajectorySource> trajectory_source;
  std::optional<CommandSource> command_source;
  std::optional<CommandFilter> command_filter;
};

}  // namespace autoware::driving_mode_manager

#endif  // CORE__INIT_HPP_
