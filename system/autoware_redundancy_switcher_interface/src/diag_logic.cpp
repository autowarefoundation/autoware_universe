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
#include "diag_logic.hpp"

namespace autoware::redundancy_switcher
{

SwitcherLevelResult compute_switcher_level(
  const std::optional<Annotated<SwitcherSignals>> & switcher,
  double now_ms,
  std::optional<double> transitional_start_ms,
  double timeout_milli)
{
  if (!switcher.has_value()) {
    return {DiagLevel::Warn, "Startup not yet complete: awaiting switcher data (WARN)", std::nullopt};
  }

  const auto & sw = *switcher;

  if (sw.value.is_faulted) {
    return {DiagLevel::Error, "Switcher fault: " + sw.annotation + " (ERROR)", std::nullopt};
  }
  if (sw.value.is_self_interrupted) {
    return {DiagLevel::Warn, "Self-interruption occurred: " + sw.annotation + " (WARN)", std::nullopt};
  }
  if (sw.value.is_stable) {
    return {DiagLevel::Ok, "Switcher stable: " + sw.annotation + " (OK)", std::nullopt};
  }

  // Transitional state (startup or state change in progress)
  const double start_ms = transitional_start_ms.value_or(now_ms);
  const double elapsed_ms = now_ms - start_ms;

  if (elapsed_ms > timeout_milli) {
    return {
      DiagLevel::Error,
      "Switcher transitional state too long (" +
        std::to_string(static_cast<int>(elapsed_ms)) + "ms): " + sw.annotation + " (ERROR)",
      start_ms};
  }

  return {
    DiagLevel::Warn,
    "Switcher in transitional state: " + sw.annotation + " (WARN)",
    start_ms};
}

}  // namespace autoware::redundancy_switcher
