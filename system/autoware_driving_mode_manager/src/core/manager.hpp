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

#ifndef CORE__MANAGER_HPP_
#define CORE__MANAGER_HPP_

#include "core/config.hpp"
#include "core/init.hpp"
#include "core/status.hpp"
#include "core/task.hpp"
#include "type/interface.hpp"

#include <autoware_driving_mode_manager/plugin.hpp>

#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace autoware::driving_mode_manager
{

class Manager : public MainLogic
{
public:
  explicit Manager(ManagerInit & init);
  bool is_ready() const;

  void update() override;
  void on_trajectory_source(const TrajectorySource & source) override;
  void on_command_source(const CommandSource & source) override;
  void on_command_filter(const CommandFilter & filter) override;
  void on_vehicle_control_mode(const PlatformMode & mode) override;
  void on_available_flag(const AutowareMode & mode, bool flag) override;
  void on_stable_flag(const AutowareMode & mode, bool flag) override;
  void on_continuable_flag(const AutowareMode & mode, bool flag) override;
  void on_mrm_state(const AutowareMode & mode, const MrmState::State & state) override;
  ServiceResponse change_mrm_request(const MrmRequest & request) override;
  ServiceResponse change_operation_mode(const OperationMode & operation_mode) override;
  ServiceResponse change_autoware_control(const AutowareControl & autoware_control) override;

private:
  void execute_tasks();
  void publish_operation_mode() const;
  void publish_mrm_state() const;
  void publish_driving_mode_request() const;
  void publish_debug() const;
  void change_autoware_mode(const AutowareMode & mode);

  std::unique_ptr<Interface> interface_;
  std::shared_ptr<Plugin> plugin_;

  std::unique_ptr<DrivingModeConfig> config_;
  std::unique_ptr<DrivingModeStatus> status_;
  std::unordered_map<AutowareMode, MrmState::State> mrm_states_;
  std::unordered_set<AutowareMode> temporary_unavailable_modes_;

  RequestModes request_;
  GateStatus gates_;

  TaskPhase phase_ = TaskPhase::kCompleted;
  std::queue<std::unique_ptr<Task>> tasks_;
};

}  // namespace autoware::driving_mode_manager

#endif  // CORE__MANAGER_HPP_
