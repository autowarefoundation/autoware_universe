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

#include "manager.hpp"

#include "values.hpp"

#include <rclcpp/logging.hpp>

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace autoware::driving_mode_manager
{

const auto logger = rclcpp::get_logger("Manager");
constexpr AutowareMode unknown_mode = AutowareMode{0};

template <typename ModeIterable>
void print_modes(const std::string & title, const ModeIterable & modes)
{
  std::string text;
  for (const auto & mode : modes) {
    text = text + " " + std::to_string(mode.id);
  }
  RCLCPP_INFO_STREAM(logger, title << ":" << text);
}

Manager::Manager(ManagerInit & init)
{
  interface_ = std::move(init.interface_);
  interface_->init(this);

  plugin_ = std::move(init.plugin_);
  config_ = std::move(init.config_);
  status_ = std::move(init.status_);
  mrm_states_ = std::move(init.mrm_states_);

  gates_.status = init.gates();
  gates_.expect = init.gates();

  request_.operation_mode = config_->to_autoware_mode(OperationMode::kStop);
  request_.platform_mode = init.platform_mode.value();
  request_.mrm_strategy = MrmStrategy::kNone;
  request_.mrm_behavior = unknown_mode;
  request_.autoware_mode = unknown_mode;
}

bool Manager::is_ready() const
{
  if (!status_->is_ready()) return false;
  return true;
}

void Manager::update()
{
  // Detect status timeout.
  status_->update(interface_->now(), 1.0);

  // List available modes.
  AutowareModeSet availables;
  for (const auto & mode : config_->autoware_modes()) {
    if (temporary_unavailable_modes_.count(mode) == 0) {
      if (mode.id != request_.autoware_mode.id) {
        if (status_->is_available(mode)) availables.insert(mode);
      } else {
        if (status_->is_continuable(mode)) availables.insert(mode);
      }
    }
  }

  // TODO(isamu-takagi): Check frequently mode change.
  change_autoware_mode(plugin_->decide(request_, availables));
  execute_tasks();
  publish_operation_mode();
  publish_mrm_state();
  publish_driving_mode_request();
  publish_debug_status();
}

void Manager::execute_tasks()
{
  while (!tasks_.empty()) {
    const auto result = tasks_.front()->execute(*interface_, gates_);
    switch (result) {
      case TaskResult::kFinished:
        RCLCPP_INFO_STREAM(logger, tasks_.front()->describe() << ": finished");
        tasks_.pop();
        break;
      case TaskResult::kRunning:
        RCLCPP_INFO_STREAM(logger, tasks_.front()->describe() << ": running");
        return;
      case TaskResult::kTimeout:
        RCLCPP_WARN_STREAM(logger, tasks_.front()->describe() << ": timeout");
        tasks_ = std::queue<std::unique_ptr<Task>>();
        phase_ = TaskPhase::kAborted;
        temporary_unavailable_modes_.insert(request_.autoware_mode);
        return;
      default:
        throw std::logic_error("invalid task result");
    }
  }

  const auto create_complete_tasks = [](const AutowareMode & mode) {
    std::queue<std::unique_ptr<Task>> tasks;
    tasks.push(std::make_unique<WaitModeStableTask>(mode));
    tasks.push(std::make_unique<TransitionFilterTask>(CommandFilter{false}));
    return tasks;
  };

  const auto create_override_tasks = []() {
    std::queue<std::unique_ptr<Task>> tasks;
    tasks.push(std::make_unique<TransitionFilterTask>(CommandFilter{false}));
    return tasks;
  };

  switch (phase_) {
    case TaskPhase::kAutowareMode:
    case TaskPhase::kPlatformMode:
      phase_ = TaskPhase::kWaitStable;
      tasks_ = create_complete_tasks(request_.autoware_mode);
      break;
    case TaskPhase::kOverridden:
      phase_ = TaskPhase::kWaitStable;
      tasks_ = create_override_tasks();
      break;
    case TaskPhase::kWaitStable:
      phase_ = TaskPhase::kCompleted;
      break;
    case TaskPhase::kAborted:
    case TaskPhase::kCompleted:
      break;
    default:
      throw std::logic_error("invalid task phase");
  }
}

void Manager::publish_operation_mode() const
{
  const auto is_available = [this](const OperationMode & mode) {
    return status_->is_available(config_->to_autoware_mode(mode));
  };

  OperationModeState state;
  state.mode = config_->to_operation_mode(request_.operation_mode);
  state.is_autoware_control_enabled = (request_.platform_mode != PlatformMode::kManual);
  state.is_in_transition = !tasks_.empty();
  state.is_stop_mode_available = is_available(OperationMode::kStop);
  state.is_autonomous_mode_available = is_available(OperationMode::kAutonomous);
  state.is_local_mode_available = is_available(OperationMode::kLocal);
  state.is_remote_mode_available = is_available(OperationMode::kRemote);
  interface_->publish_operation_mode(state);
}

void Manager::publish_mrm_state() const
{
  const auto behavior = config_->to_mrm_behavior(request_.autoware_mode);
  if (behavior) {
    const auto iter = mrm_states_.find(request_.autoware_mode);
    const auto state = iter != mrm_states_.end() ? iter->second : MrmState::State::kUnknown;
    interface_->publish_mrm_state(MrmState{state, behavior.value()});
  } else {
    interface_->publish_mrm_state(MrmState{MrmState::State::kNormal, MrmState::NoneBehavior});
  }
}

void Manager::publish_driving_mode_request() const
{
  interface_->publish_driving_mode_request(request_.autoware_mode);
}

void Manager::publish_debug_status() const
{
  DebugStatus debug;
  for (const auto & mode : config_->autoware_modes()) {
    debug.availables[mode] = status_->is_available(mode);
    debug.stables[mode] = status_->is_stable(mode);
    debug.continuables[mode] = status_->is_continuable(mode);
  }
  interface_->publish_debug_status(debug);
  interface_->publish_debug_status(request_);
}

void Manager::on_trajectory_source(const TrajectorySource & source)
{
  if (gates_.expect.trajectory_source != source) {
    RCLCPP_WARN_STREAM(logger, "trajectory source override: " << source.id);
  }
  gates_.status.trajectory_source = source;
  gates_.expect.trajectory_source = source;
  execute_tasks();
}

void Manager::on_command_source(const CommandSource & source)
{
  if (gates_.expect.command_source != source) {
    RCLCPP_WARN_STREAM(logger, "command source override: " << source.id);
  }
  gates_.status.command_source = source;
  gates_.expect.command_source = source;
  execute_tasks();
}

void Manager::on_command_filter(const CommandFilter & filter)
{
  if (gates_.expect.command_filter != filter) {
    RCLCPP_WARN_STREAM(logger, "command filter override: " << filter.flag);
  }
  gates_.status.command_filter = filter;
  gates_.expect.command_filter = filter;
  execute_tasks();
}

void Manager::on_vehicle_control_mode(const PlatformMode & mode)
{
  if (gates_.expect.platform_mode != mode) {
    RCLCPP_WARN_STREAM(logger, "platform mode override: " << to_string(mode));
    request_.platform_mode = mode;
    if (phase_ == TaskPhase::kPlatformMode) {
      tasks_ = std::queue<std::unique_ptr<Task>>();
      phase_ = TaskPhase::kOverridden;
    }
  }
  gates_.status.platform_mode = mode;
  gates_.expect.platform_mode = mode;
  execute_tasks();
}

void Manager::on_available_flag(const AutowareMode & mode, bool flag)
{
  if (const auto & data = status_->data(mode)) {
    data->available.update(interface_->now(), flag);
  }
}

void Manager::on_stable_flag(const AutowareMode & mode, bool flag)
{
  if (const auto & data = status_->data(mode)) {
    data->stable.update(interface_->now(), flag);
  }
}

void Manager::on_continuable_flag(const AutowareMode & mode, bool flag)
{
  if (const auto & data = status_->data(mode)) {
    data->continuable.update(interface_->now(), flag);
  }
}

void Manager::on_mrm_state(const AutowareMode & mode, const MrmState::State & state)
{
  mrm_states_[mode] = state;
}

ServiceResponse Manager::change_mrm_request(const MrmRequest & request)
{
  if (request.strategy == MrmStrategy::kNone || request.strategy == MrmStrategy::kDelegate) {
    request_.mrm_strategy = request.strategy;
    request_.mrm_behavior = unknown_mode;
    return ServiceResponse{true, ""};
  }
  if (request.strategy == MrmStrategy::kBehavior) {
    const auto behavior = config_->to_autoware_mode(request.behavior);
    if (!behavior) {
      return ServiceResponse{false, "unknown behavior"};
    }
    request_.mrm_strategy = request.strategy;
    request_.mrm_behavior = behavior.value();
    return ServiceResponse{true, ""};
  }
  return ServiceResponse{false, "unknown strategy"};
}

ServiceResponse Manager::change_operation_mode(const OperationMode & operation_mode)
{
  const auto mode = config_->to_autoware_mode(operation_mode);

  if (!status_->is_available(mode)) {
    return ServiceResponse{false, "operation mode is not available"};
  }

  request_.operation_mode = mode;
  return ServiceResponse{true, ""};
}

ServiceResponse Manager::change_autoware_control(const AutowareControl & autoware_control)
{
  const auto platform_mode = to_platform_mode(autoware_control);

  // If disable, request the manual mode immediately.
  if (platform_mode == PlatformMode::kManual) {
    RCLCPP_INFO_STREAM(logger, "accept autoware control disable");
    request_.platform_mode = platform_mode;
    interface_->change_platform_mode(platform_mode);
    return ServiceResponse{true, ""};
  }

  // The check target is the normal behavior, operation mode. MRM is not included.
  const auto mode = request_.operation_mode;

  if (!status_->is_available(mode)) {
    return ServiceResponse{false, "operation mode is not available"};
  }

  std::queue<std::unique_ptr<Task>> tasks;
  tasks.push(std::make_unique<TransitionFilterTask>(CommandFilter{true}));
  tasks.push(std::make_unique<PlatformModeTask>(PlatformMode::kAutoware));
  tasks_.swap(tasks);
  phase_ = TaskPhase::kPlatformMode;

  request_.platform_mode = platform_mode;
  return ServiceResponse{true, ""};
}

void Manager::change_autoware_mode(const AutowareMode & mode)
{
  AutowareMode & prev = request_.autoware_mode;
  if (prev.id == mode.id) {
    return;
  }
  if (!config_->exists(mode)) {
    RCLCPP_ERROR_STREAM(logger, "decision logic returns unknown mode: " << mode.id);
    return;
  }

  RCLCPP_INFO_STREAM(logger, "Change Autoware Mode: " << prev.id << " -> " << mode.id);
  prev = mode;

  const auto gates = config_->gates(mode);
  std::queue<std::unique_ptr<Task>> tasks;
  tasks.push(std::make_unique<TransitionFilterTask>(CommandFilter{true}));
  tasks.push(std::make_unique<WaitModeReadyTask>(mode));
  if (gates.trajectory) tasks.push(std::make_unique<TrajectorySourceTask>(*gates.trajectory));
  if (gates.command) tasks.push(std::make_unique<CommandSourceTask>(*gates.command));
  tasks_.swap(tasks);
  phase_ = TaskPhase::kAutowareMode;
}

}  // namespace autoware::driving_mode_manager
