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
#ifndef DIAG_LOGIC_HPP_
#define DIAG_LOGIC_HPP_

#include <redundancy_switcher_interface/ir/domain_types.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace autoware::redundancy_switcher
{

// Maps directly to diagnostic_msgs::msg::DiagnosticStatus values.
enum class DiagLevel : uint8_t { Ok = 0, Warn = 1, Error = 2 };

struct SwitcherLevelResult
{
  DiagLevel level;
  std::string message;
  // Updated transitional start timestamp (ms). nullopt means "cleared".
  std::optional<double> transitional_start_ms;
};

// Pure function: no ROS, no side effects, fully testable.
//
// Determines the diagnostic level and message for the switcher_signals field.
// Manages the transitional-state timeout by returning the updated
// transitional_start_ms that the caller must persist across calls.
//
// @param switcher               Current switcher state from DomainSnapshot.
// @param now_ms                 Current time in milliseconds (monotonic).
// @param transitional_start_ms  Persistent start time of the current transitional period.
//                               Pass nullopt on first call or after a non-transitional state.
// @param timeout_milli          Threshold (ms) after which transitional → ERROR.
SwitcherLevelResult compute_switcher_level(
  const std::optional<Annotated<SwitcherSignals>> & switcher, double now_ms,
  std::optional<double> transitional_start_ms, double timeout_milli);

}  // namespace autoware::redundancy_switcher
#endif  // DIAG_LOGIC_HPP_
