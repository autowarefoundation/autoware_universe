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

#include "autoware/component_interface_specs/sensing.hpp"
#include "autoware/component_interface_specs_universe/sensing.hpp"
#include "gtest/gtest.h"

#include <type_traits>

// Proves the universe symbols are re-exports of the canonical core types, not
// redefinitions: consumers see one definition, one version authority.
TEST(sensing_universe, reexports_core)
{
  namespace core = autoware::component_interface_specs::sensing;
  namespace universe = autoware::component_interface_specs_universe::sensing;
  static_assert(
    std::is_same_v<universe::VehicleVelocityConverterTwist, core::VehicleVelocityConverterTwist>);
  SUCCEED();
}

TEST(sensing, interface)
{
  {
    using autoware::component_interface_specs_universe::sensing::VehicleVelocityConverterTwist;
    VehicleVelocityConverterTwist twist;
    size_t depth = 10;
    EXPECT_EQ(twist.depth, depth);
    EXPECT_EQ(twist.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(twist.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }
}
