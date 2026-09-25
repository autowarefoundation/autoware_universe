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

#include "deceleration_profile.hpp"

#include <gtest/gtest.h>

namespace autoware::mrm_steering_hold_stop_operator
{

using tier4_system_msgs::msg::InLaneStopTrigger;

namespace
{
DecelerationProfiles makeProfiles()
{
  DecelerationProfiles profiles;
  profiles.moderate = DecelerationTargets{-3.0, -5.0};
  profiles.emergency = DecelerationTargets{-6.0, -20.0};
  return profiles;
}
}  // namespace

TEST(DecelerationProfile, ProfileSelectsParameterSet)
{
  const auto profiles = makeProfiles();

  const auto moderate = resolve_profile(profiles, InLaneStopTrigger::PROFILE_MODERATE);
  EXPECT_DOUBLE_EQ(moderate.targets.target_acceleration, -3.0);
  EXPECT_DOUBLE_EQ(moderate.targets.target_jerk, -5.0);
  EXPECT_FALSE(moderate.is_fallback);

  const auto emergency = resolve_profile(profiles, InLaneStopTrigger::PROFILE_EMERGENCY);
  EXPECT_DOUBLE_EQ(emergency.targets.target_acceleration, -6.0);
  EXPECT_DOUBLE_EQ(emergency.targets.target_jerk, -20.0);
  EXPECT_FALSE(emergency.is_fallback);
}

TEST(DecelerationProfile, UnknownProfileFallsBackToModerate)
{
  const auto profiles = makeProfiles();

  for (const ProfileType profile :
       {ProfileType{InLaneStopTrigger::PROFILE_UNKNOWN}, ProfileType{3}}) {
    const auto resolved = resolve_profile(profiles, profile);
    EXPECT_DOUBLE_EQ(resolved.targets.target_acceleration, -3.0);
    EXPECT_DOUBLE_EQ(resolved.targets.target_jerk, -5.0);
    EXPECT_TRUE(resolved.is_fallback);
  }
}

}  // namespace autoware::mrm_steering_hold_stop_operator
