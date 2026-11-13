// Copyright 2026 TIER IV, Inc.
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

#ifndef STOP_PROFILE_HPP_
#define STOP_PROFILE_HPP_

#include <tier4_system_msgs/msg/in_lane_stop_trigger.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace autoware::in_lane_mrm_planner
{

// Deceleration profile requested by the in-lane stop operator. The constraint values of each
// profile are owned by the planner (`mrm_velocity.profiles.<name>`).
enum class StopProfile : uint8_t {
  MODERATE = 0,
  EMERGENCY = 1,
};

inline constexpr std::array kAllStopProfiles{StopProfile::MODERATE, StopProfile::EMERGENCY};

inline constexpr size_t kNumStopProfiles = kAllStopProfiles.size();

// Profile whose candidate is published while no trigger is active (hot standby output).
inline constexpr StopProfile kStandbyStopProfile = StopProfile::MODERATE;

// Profile used when a trigger requests an unknown profile value (e.g. PROFILE_UNKNOWN).
inline constexpr StopProfile kFallbackStopProfile = StopProfile::MODERATE;

inline constexpr size_t to_index(const StopProfile profile)
{
  return static_cast<size_t>(profile);
}

inline const char * to_string(const StopProfile profile)
{
  switch (profile) {
    case StopProfile::MODERATE:
      return "moderate";
    case StopProfile::EMERGENCY:
      return "emergency";
    default:
      return "unknown";
  }
}

// Converts `tier4_system_msgs/msg/InLaneStopTrigger.profile`; returns nullopt for unknown values.
inline std::optional<StopProfile> from_trigger_profile(const uint8_t profile)
{
  using tier4_system_msgs::msg::InLaneStopTrigger;
  switch (profile) {
    case InLaneStopTrigger::PROFILE_MODERATE:
      return StopProfile::MODERATE;
    case InLaneStopTrigger::PROFILE_EMERGENCY:
      return StopProfile::EMERGENCY;
    default:
      return std::nullopt;
  }
}

// Converts back to the `InLaneStopTrigger.profile` constant (used for debug output).
inline uint8_t to_trigger_profile(const StopProfile profile)
{
  using tier4_system_msgs::msg::InLaneStopTrigger;
  switch (profile) {
    case StopProfile::MODERATE:
      return InLaneStopTrigger::PROFILE_MODERATE;
    case StopProfile::EMERGENCY:
      return InLaneStopTrigger::PROFILE_EMERGENCY;
    default:
      return InLaneStopTrigger::PROFILE_UNKNOWN;
  }
}

enum class LatchAction {
  KEEP,      // nothing to do
  LATCH,     // trigger active and nothing latched yet: latch the requested profile
  RE_LATCH,  // trigger active but a different profile is latched: re-plan and latch the new one
  UNLATCH,   // trigger released while latched
};

inline LatchAction decide_latch_action(
  const bool trigger_active, const StopProfile requested_profile,
  const std::optional<StopProfile> & latched_profile)
{
  if (!trigger_active) {
    return latched_profile ? LatchAction::UNLATCH : LatchAction::KEEP;
  }
  if (!latched_profile) {
    return LatchAction::LATCH;
  }
  return *latched_profile == requested_profile ? LatchAction::KEEP : LatchAction::RE_LATCH;
}

}  // namespace autoware::in_lane_mrm_planner

#endif  // STOP_PROFILE_HPP_
