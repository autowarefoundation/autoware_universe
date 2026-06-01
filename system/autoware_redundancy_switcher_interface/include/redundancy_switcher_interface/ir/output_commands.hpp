//  Copyright 2025 The Autoware Contributors
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
#ifndef REDUNDANCY_SWITCHER_INTERFACE__IR__OUTPUT_COMMANDS_HPP_
#define REDUNDANCY_SWITCHER_INTERFACE__IR__OUTPUT_COMMANDS_HPP_

#include "redundancy_switcher_interface/ir/domain_types.hpp"

#include <string>
#include <variant>
#include <vector>

namespace autoware::redundancy_switcher
{

struct ResetCommand {};
struct SelfInterruptionCommand {};

enum class LogLevel { Debug, Info, Warn, Error, Fatal };

struct LogCommand
{
  LogLevel level{LogLevel::Info};
  std::string message;
};

// DiagAdapter reads the current state via gateway->snapshot(); snapshot field is for compatibility.
struct UpdateStatusDiagCommand  { DomainSnapshot snapshot; };
struct UpdateActiveControlUnitCommand { ActiveControlUnit value; };

// SubSystemAdapter maps reason to ResponseStatus:
//   accepted=true                     → SUCCESS
//   accepted=false, Ignored           → IGNORED   (precondition not met: vehicle moving, etc.)
//   accepted=false, NotNecessary      → SUCCESS    (already stable: reset not needed)
//   accepted=false, Error             → ERROR      (fault or abnormal state)
enum class ResetRejectedReason { Ignored, NotNecessary, Error };

struct ResetResultCommand
{
  bool accepted;
  ResetRejectedReason reason{ResetRejectedReason::Error};  // Valid only when accepted=false.
  std::string message;
};

struct UpdateAutowareReadyCommand               { AutowareReady value; };
struct UpdateAnotherEcuAvailabilityTimeoutCommand { bool timed_out; };

using OutputCommand = std::variant<
  LogCommand, ResetCommand, SelfInterruptionCommand, UpdateStatusDiagCommand,
  UpdateActiveControlUnitCommand, UpdateAutowareReadyCommand, ResetResultCommand,
  UpdateAnotherEcuAvailabilityTimeoutCommand>;

}  // namespace autoware::redundancy_switcher
#endif  // REDUNDANCY_SWITCHER_INTERFACE__IR__OUTPUT_COMMANDS_HPP_
