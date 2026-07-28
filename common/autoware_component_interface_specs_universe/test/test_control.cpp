// Copyright 2023 The Autoware Contributors
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

#include "autoware/component_interface_specs/control.hpp"
#include "autoware/component_interface_specs_universe/control.hpp"
#include "gtest/gtest.h"

#include <type_traits>

// Proves the universe symbols are re-exports of the canonical core types, not
// redefinitions: consumers see one definition, one version authority.
TEST(control_universe, reexports_core)
{
  namespace core = autoware::component_interface_specs::control;
  namespace universe = autoware::component_interface_specs_universe::control;
  static_assert(std::is_same_v<universe::ControlCommand, core::ControlCommand>);
  static_assert(std::is_same_v<universe::GearCommand, core::GearCommand>);
  SUCCEED();
}

TEST(control, interface)
{
  {
    using autoware::component_interface_specs_universe::control::IsPaused;
    IsPaused is_paused;
    size_t depth = 1;
    EXPECT_EQ(is_paused.depth, depth);
    EXPECT_EQ(is_paused.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(is_paused.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  {
    using autoware::component_interface_specs_universe::control::IsStartRequested;
    IsStartRequested is_start_requested;
    size_t depth = 1;
    EXPECT_EQ(is_start_requested.depth, depth);
    EXPECT_EQ(is_start_requested.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(is_start_requested.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  {
    using autoware::component_interface_specs_universe::control::IsStopped;
    IsStopped is_stopped;
    size_t depth = 1;
    EXPECT_EQ(is_stopped.depth, depth);
    EXPECT_EQ(is_stopped.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(is_stopped.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }
}
