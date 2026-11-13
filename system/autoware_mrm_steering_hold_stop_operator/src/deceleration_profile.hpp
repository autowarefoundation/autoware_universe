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

#ifndef DECELERATION_PROFILE_HPP_
#define DECELERATION_PROFILE_HPP_

#include <tier4_system_msgs/msg/in_lane_stop_trigger.hpp>

namespace autoware::mrm_steering_hold_stop_operator
{

using ProfileType = tier4_system_msgs::msg::InLaneStopTrigger::_profile_type;

struct DecelerationTargets
{
  double target_acceleration{0.0};  // [m/s^2] (negative)
  double target_jerk{0.0};          // [m/s^3] (negative)
};

/// Deceleration constraints per InLaneStopTrigger::PROFILE_*.
struct DecelerationProfiles
{
  DecelerationTargets moderate;
  DecelerationTargets emergency;
};

struct ResolvedProfile
{
  DecelerationTargets targets;
  // true when the requested profile is not known and moderate is used instead.
  bool is_fallback{false};
};

/// Select the deceleration targets for a profile.
/// An unknown profile falls back to moderate: not stopping is more dangerous
/// than stopping with the moderate constraints.
ResolvedProfile resolve_profile(const DecelerationProfiles & profiles, ProfileType profile);

}  // namespace autoware::mrm_steering_hold_stop_operator

#endif  // DECELERATION_PROFILE_HPP_
