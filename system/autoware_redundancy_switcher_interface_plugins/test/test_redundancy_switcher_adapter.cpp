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

// Tests for RedundancySwitcherAdapter's pure static helper functions.
// No ROS context required.

#include "redundancy_switcher_adapter.hpp"

#include <gtest/gtest.h>

namespace autoware::redundancy_switcher
{

// ---------------------------------------------------------------------------
// is_transitional_state
// ---------------------------------------------------------------------------

TEST(IsTransitionalState, Initializing_IsTransitional)
{
  EXPECT_TRUE(RedundancySwitcherAdapter::is_transitional_state(0 /* INITIALIZING */));
}

TEST(IsTransitionalState, InElection_IsTransitional)
{
  EXPECT_TRUE(RedundancySwitcherAdapter::is_transitional_state(3 /* IN_ELECTION */));
}

TEST(IsTransitionalState, Electable_IsNotTransitional)
{
  EXPECT_FALSE(RedundancySwitcherAdapter::is_transitional_state(1 /* ELECTABLE */));
}

TEST(IsTransitionalState, WaitForAutoware_IsNotTransitional)
{
  EXPECT_FALSE(RedundancySwitcherAdapter::is_transitional_state(2 /* WAIT_FOR_AUTOWARE */));
}

TEST(IsTransitionalState, ElectionCompleted_IsNotTransitional)
{
  EXPECT_FALSE(RedundancySwitcherAdapter::is_transitional_state(4 /* ELECTION_COMPLETED */));
}

TEST(IsTransitionalState, ElectionUnclosed_IsNotTransitional)
{
  EXPECT_FALSE(RedundancySwitcherAdapter::is_transitional_state(5 /* ELECTION_UNCLOSED */));
}

TEST(IsTransitionalState, PathNotFound_IsNotTransitional)
{
  EXPECT_FALSE(RedundancySwitcherAdapter::is_transitional_state(6 /* PATH_NOT_FOUND */));
}

TEST(IsTransitionalState, SelfInterruption_IsNotTransitional)
{
  EXPECT_FALSE(RedundancySwitcherAdapter::is_transitional_state(7 /* SELF_INTERRUPTION */));
}

// ---------------------------------------------------------------------------
// to_switcher_signals
// ---------------------------------------------------------------------------

TEST(ToSwitcherSignals, Initializing_AllFalse)
{
  const auto s = RedundancySwitcherAdapter::to_switcher_signals(0);
  EXPECT_FALSE(s.is_stable);
  EXPECT_FALSE(s.is_self_interrupted);
  EXPECT_FALSE(s.is_faulted);
}

TEST(ToSwitcherSignals, Electable_IsStable)
{
  const auto s = RedundancySwitcherAdapter::to_switcher_signals(1);
  EXPECT_TRUE(s.is_stable);
  EXPECT_FALSE(s.is_self_interrupted);
  EXPECT_FALSE(s.is_faulted);
}

TEST(ToSwitcherSignals, WaitForAutoware_AllFalse)
{
  const auto s = RedundancySwitcherAdapter::to_switcher_signals(2);
  EXPECT_FALSE(s.is_stable);
  EXPECT_FALSE(s.is_self_interrupted);
  EXPECT_FALSE(s.is_faulted);
}

TEST(ToSwitcherSignals, InElection_AllFalse)
{
  const auto s = RedundancySwitcherAdapter::to_switcher_signals(3);
  EXPECT_FALSE(s.is_stable);
  EXPECT_FALSE(s.is_self_interrupted);
  EXPECT_FALSE(s.is_faulted);
}

TEST(ToSwitcherSignals, ElectionCompleted_IsStable)
{
  const auto s = RedundancySwitcherAdapter::to_switcher_signals(4);
  EXPECT_TRUE(s.is_stable);
  EXPECT_FALSE(s.is_self_interrupted);
  EXPECT_FALSE(s.is_faulted);
}

TEST(ToSwitcherSignals, ElectionUnclosed_IsFaulted)
{
  const auto s = RedundancySwitcherAdapter::to_switcher_signals(5);
  EXPECT_FALSE(s.is_stable);
  EXPECT_FALSE(s.is_self_interrupted);
  EXPECT_TRUE(s.is_faulted);
}

TEST(ToSwitcherSignals, PathNotFound_IsFaulted)
{
  const auto s = RedundancySwitcherAdapter::to_switcher_signals(6);
  EXPECT_FALSE(s.is_stable);
  EXPECT_FALSE(s.is_self_interrupted);
  EXPECT_TRUE(s.is_faulted);
}

TEST(ToSwitcherSignals, SelfInterruption_IsSelfInterrupted)
{
  const auto s = RedundancySwitcherAdapter::to_switcher_signals(7);
  EXPECT_FALSE(s.is_stable);
  EXPECT_TRUE(s.is_self_interrupted);
  EXPECT_FALSE(s.is_faulted);
}

TEST(ToSwitcherSignals, Unknown_AllFalse)
{
  const auto s = RedundancySwitcherAdapter::to_switcher_signals(255);
  EXPECT_FALSE(s.is_stable);
  EXPECT_FALSE(s.is_self_interrupted);
  EXPECT_FALSE(s.is_faulted);
}

// ---------------------------------------------------------------------------
// to_active_control_unit
// ---------------------------------------------------------------------------

TEST(ToActiveControlUnit, ZeroPathInfo_EmptyIds)
{
  const auto acu = RedundancySwitcherAdapter::to_active_control_unit(0);
  EXPECT_TRUE(acu.unit_ids.empty());
}

TEST(ToActiveControlUnit, MainEcuToMainVcu_Ids0And2)
{
  // path_info = 5 = 0b0101 → bit0 (main_ecu=0) + bit2 (main_vcu=2)
  const auto acu = RedundancySwitcherAdapter::to_active_control_unit(5);
  ASSERT_EQ(acu.unit_ids.size(), 2u);
  EXPECT_EQ(acu.unit_ids[0], 0u);
  EXPECT_EQ(acu.unit_ids[1], 2u);
}

TEST(ToActiveControlUnit, SubEcuToMainVcu_Ids1And2)
{
  // path_info = 6 = 0b0110 → bit1 (sub_ecu=1) + bit2 (main_vcu=2)
  const auto acu = RedundancySwitcherAdapter::to_active_control_unit(6);
  ASSERT_EQ(acu.unit_ids.size(), 2u);
  EXPECT_EQ(acu.unit_ids[0], 1u);
  EXPECT_EQ(acu.unit_ids[1], 2u);
}

TEST(ToActiveControlUnit, MainEcuToSubVcu_Ids0And3)
{
  // path_info = 9 = 0b1001 → bit0 (main_ecu=0) + bit3 (sub_vcu=3)
  const auto acu = RedundancySwitcherAdapter::to_active_control_unit(9);
  ASSERT_EQ(acu.unit_ids.size(), 2u);
  EXPECT_EQ(acu.unit_ids[0], 0u);
  EXPECT_EQ(acu.unit_ids[1], 3u);
}

TEST(ToActiveControlUnit, SubEcuToSubVcu_Ids1And3)
{
  // path_info = 10 = 0b1010 → bit1 (sub_ecu=1) + bit3 (sub_vcu=3)
  const auto acu = RedundancySwitcherAdapter::to_active_control_unit(10);
  ASSERT_EQ(acu.unit_ids.size(), 2u);
  EXPECT_EQ(acu.unit_ids[0], 1u);
  EXPECT_EQ(acu.unit_ids[1], 3u);
}

}  // namespace autoware::redundancy_switcher
