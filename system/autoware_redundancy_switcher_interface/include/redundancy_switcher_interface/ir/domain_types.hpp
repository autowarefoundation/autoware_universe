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
#ifndef REDUNDANCY_SWITCHER_INTERFACE__IR__DOMAIN_TYPES_HPP_
#define REDUNDANCY_SWITCHER_INTERFACE__IR__DOMAIN_TYPES_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::redundancy_switcher
{

struct SwitcherSignals {
  bool is_stable;           // Switcher is ready; self-interruption is possible.
  bool is_self_interrupted; // Self-interruption acknowledged; only reset is accepted.
  bool is_faulted;          // Unrecoverable fault reported; all operations rejected.
  // all false: transitional state (startup or state change in progress)
};

enum class AutowareReady { False, True };
enum class VelocityStatus { Stopped, Moving };
enum class ControlMode { Manual, Auto };

struct ActiveControlUnit {
  std::vector<uint8_t> unit_ids;
};

// Attaches a human-readable annotation string to an enum value for diagnostics and logging.
// The enum value's meaning is unchanged; annotation is metadata only.
template <typename E>
struct Annotated
{
  E value;
  std::string annotation;

  Annotated(E v, std::string r = {}) : value(v), annotation(std::move(r)) {}  // NOLINT
};

using RequestId = uint64_t;
using TimerId = std::string;

// Read-only view of Processor's current state.
// nullopt for any field means the data has never been received (startup not yet complete).
struct DomainSnapshot
{
  std::optional<Annotated<AutowareReady>> autoware_ready;
  std::optional<Annotated<VelocityStatus>> velocity_status;
  std::optional<Annotated<ControlMode>> control_mode;
  std::optional<Annotated<SwitcherSignals>> switcher;
  // nullopt = not yet evaluated (immediately after startup)
  std::optional<bool> another_ecu_availability_timeout;
};

}  // namespace autoware::redundancy_switcher
#endif  // REDUNDANCY_SWITCHER_INTERFACE__IR__DOMAIN_TYPES_HPP_
