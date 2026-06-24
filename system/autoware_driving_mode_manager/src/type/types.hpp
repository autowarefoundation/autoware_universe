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

#ifndef TYPE__TYPES_HPP_
#define TYPE__TYPES_HPP_

#include <autoware_driving_mode_manager/types.hpp>

#include <string>
#include <unordered_map>

namespace autoware::driving_mode_manager
{

struct ServiceResponse
{
  bool success;
  std::string message;
};

struct OperationModeState
{
  OperationMode mode;
  bool is_autoware_control_enabled;
  bool is_in_transition;
  bool is_stop_mode_available;
  bool is_autonomous_mode_available;
  bool is_local_mode_available;
  bool is_remote_mode_available;
};

struct MrmState
{
  enum class State {
    kUnknown,
    kNormal,
    kOperating,
    kSucceeded,
    kFailed,
  };
  static constexpr MrmBehavior NoneBehavior{1};
  State state;
  MrmBehavior behavior;
};

struct MrmRequest
{
  MrmStrategy strategy;
  MrmBehavior behavior;
};

struct ModeRequest
{
  AutowareMode mode;
  uint16_t priority;
};

struct ModeInfo
{
  std::unordered_map<AutowareMode, std::string> names;
};

struct GateStatusItem
{
  TrajectorySource trajectory_source;
  CommandSource command_source;
  CommandFilter command_filter;
  PlatformMode platform_mode;
};

struct GateStatus
{
  GateStatusItem status;
  GateStatusItem expect;
};

struct DebugStatus
{
  struct Flag
  {
    bool available;
    bool active;
    bool stable;
    bool continuable;
  };
  std::unordered_map<AutowareMode, Flag> flags;
};

}  // namespace autoware::driving_mode_manager

#endif  // TYPE__TYPES_HPP_
