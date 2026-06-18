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

#include "autoware/ptv3/experimental/semantic_label.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string_view>

namespace autoware::ptv3::experimental
{

class SemanticLabelTest : public ::testing::Test
{
};

// ============================================================================
// Enum Value Tests
// ============================================================================

TEST_F(SemanticLabelTest, EnumValuesCorrect)
{
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::CAR), 0U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::TRUCK), 1U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::BUS), 2U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::BICYCLE), 3U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::PEDESTRIAN), 4U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::HAZARD), 5U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::GROUND), 6U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::STRUCTURE), 7U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::VEGETATION), 8U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::NOISE), 9U);
}

// ============================================================================
// String Conversion Tests
// ============================================================================

TEST_F(SemanticLabelTest, ToStringConversion)
{
  EXPECT_EQ(to_string(SemanticLabel::CAR), "CAR");
  EXPECT_EQ(to_string(SemanticLabel::TRUCK), "TRUCK");
  EXPECT_EQ(to_string(SemanticLabel::BUS), "BUS");
  EXPECT_EQ(to_string(SemanticLabel::BICYCLE), "BICYCLE");
  EXPECT_EQ(to_string(SemanticLabel::PEDESTRIAN), "PEDESTRIAN");
  EXPECT_EQ(to_string(SemanticLabel::HAZARD), "HAZARD");
  EXPECT_EQ(to_string(SemanticLabel::GROUND), "GROUND");
  EXPECT_EQ(to_string(SemanticLabel::STRUCTURE), "STRUCTURE");
  EXPECT_EQ(to_string(SemanticLabel::VEGETATION), "VEGETATION");
  EXPECT_EQ(to_string(SemanticLabel::NOISE), "NOISE");
}

// ============================================================================
// try_into_object Tests
// ============================================================================

TEST_F(SemanticLabelTest, TryIntoObjectValidObjectLabels)
{
  // Object-compatible labels should return non-nullopt values
  auto car = try_into_object(SemanticLabel::CAR);
  EXPECT_TRUE(car.has_value());
  EXPECT_EQ(car.value(), 1U);  // ObjectClassification::CAR

  auto truck = try_into_object(SemanticLabel::TRUCK);
  EXPECT_TRUE(truck.has_value());
  EXPECT_EQ(truck.value(), 2U);  // ObjectClassification::TRUCK

  auto bus = try_into_object(SemanticLabel::BUS);
  EXPECT_TRUE(bus.has_value());
  EXPECT_EQ(bus.value(), 3U);  // ObjectClassification::BUS

  auto bicycle = try_into_object(SemanticLabel::BICYCLE);
  EXPECT_TRUE(bicycle.has_value());
  EXPECT_EQ(bicycle.value(), 6U);  // ObjectClassification::BICYCLE

  auto pedestrian = try_into_object(SemanticLabel::PEDESTRIAN);
  EXPECT_TRUE(pedestrian.has_value());
  EXPECT_EQ(pedestrian.value(), 7U);  // ObjectClassification::PEDESTRIAN

  auto hazard = try_into_object(SemanticLabel::HAZARD);
  EXPECT_TRUE(hazard.has_value());
  EXPECT_EQ(hazard.value(), 9U);  // ObjectClassification::HAZARD
}

TEST_F(SemanticLabelTest, TryIntoObjectNonObjectLabels)
{
  // Non-object labels should return nullopt
  EXPECT_FALSE(try_into_object(SemanticLabel::GROUND).has_value());
  EXPECT_FALSE(try_into_object(SemanticLabel::STRUCTURE).has_value());
  EXPECT_FALSE(try_into_object(SemanticLabel::VEGETATION).has_value());
  EXPECT_FALSE(try_into_object(SemanticLabel::NOISE).has_value());
}

// ============================================================================
// try_into_semantic Tests
// ============================================================================

TEST_F(SemanticLabelTest, TryIntoSemanticValidLabels)
{
  // Valid ObjectClassification labels should map back to SemanticLabel
  auto car = try_into_semantic(1U);  // ObjectClassification::CAR
  EXPECT_TRUE(car.has_value());
  EXPECT_EQ(car.value(), SemanticLabel::CAR);

  auto truck = try_into_semantic(2U);  // ObjectClassification::TRUCK
  EXPECT_TRUE(truck.has_value());
  EXPECT_EQ(truck.value(), SemanticLabel::TRUCK);

  auto bus = try_into_semantic(3U);  // ObjectClassification::BUS
  EXPECT_TRUE(bus.has_value());
  EXPECT_EQ(bus.value(), SemanticLabel::BUS);

  auto bicycle = try_into_semantic(6U);  // ObjectClassification::BICYCLE
  EXPECT_TRUE(bicycle.has_value());
  EXPECT_EQ(bicycle.value(), SemanticLabel::BICYCLE);

  auto pedestrian = try_into_semantic(7U);  // ObjectClassification::PEDESTRIAN
  EXPECT_TRUE(pedestrian.has_value());
  EXPECT_EQ(pedestrian.value(), SemanticLabel::PEDESTRIAN);

  auto hazard = try_into_semantic(9U);  // ObjectClassification::HAZARD
  EXPECT_TRUE(hazard.has_value());
  EXPECT_EQ(hazard.value(), SemanticLabel::HAZARD);
}

TEST_F(SemanticLabelTest, TryIntoSemanticInvalidLabels)
{
  // Invalid ObjectClassification labels should return nullopt
  EXPECT_FALSE(try_into_semantic(0U).has_value());   // UNKNOWN
  EXPECT_FALSE(try_into_semantic(4U).has_value());   // TRAILER (not in semantic mapping)
  EXPECT_FALSE(try_into_semantic(5U).has_value());   // MOTORCYCLE (not in semantic mapping)
  EXPECT_FALSE(try_into_semantic(8U).has_value());   // ANIMAL (not in semantic mapping)
  EXPECT_FALSE(try_into_semantic(10U).has_value());  // OVER_DRIVABLE (not in semantic mapping)
  EXPECT_FALSE(try_into_semantic(11U).has_value());  // UNDER_DRIVABLE (not in semantic mapping)
  EXPECT_FALSE(try_into_semantic(255U).has_value());
}

// ============================================================================
// is_object_compatible Tests
// ============================================================================

TEST_F(SemanticLabelTest, IsObjectCompatibleObjectLabels)
{
  // Object-compatible labels
  EXPECT_TRUE(is_object_compatible(SemanticLabel::CAR));
  EXPECT_TRUE(is_object_compatible(SemanticLabel::TRUCK));
  EXPECT_TRUE(is_object_compatible(SemanticLabel::BUS));
  EXPECT_TRUE(is_object_compatible(SemanticLabel::BICYCLE));
  EXPECT_TRUE(is_object_compatible(SemanticLabel::PEDESTRIAN));
  EXPECT_TRUE(is_object_compatible(SemanticLabel::HAZARD));
}

TEST_F(SemanticLabelTest, IsObjectCompatibleNonObjectLabels)
{
  // Non-object labels
  EXPECT_FALSE(is_object_compatible(SemanticLabel::GROUND));
  EXPECT_FALSE(is_object_compatible(SemanticLabel::STRUCTURE));
  EXPECT_FALSE(is_object_compatible(SemanticLabel::VEGETATION));
  EXPECT_FALSE(is_object_compatible(SemanticLabel::NOISE));
}

// ============================================================================
// Roundtrip Conversion Tests
// ============================================================================

TEST_F(SemanticLabelTest, RoundtripObjectLabelToSemanticAndBack)
{
  // Test roundtrip for object-compatible labels
  constexpr std::uint8_t object_labels[] = {1U, 2U, 3U, 6U, 7U, 9U};

  for (auto obj_label : object_labels) {
    auto semantic = try_into_semantic(obj_label);
    EXPECT_TRUE(semantic.has_value());

    auto back_to_object = try_into_object(semantic.value());
    EXPECT_TRUE(back_to_object.has_value());
    EXPECT_EQ(back_to_object.value(), obj_label);
  }
}

// ============================================================================
// Constexpr Verification Tests
// ============================================================================

TEST_F(SemanticLabelTest, ConstexprEvaluation)
{
  // Verify that functions can be evaluated at compile time
  constexpr auto str = to_string(SemanticLabel::CAR);
  EXPECT_EQ(str, "CAR");

  constexpr auto obj = try_into_object(SemanticLabel::CAR);
  EXPECT_TRUE(obj.has_value());
  EXPECT_EQ(obj.value(), 1U);

  constexpr auto sem = try_into_semantic(1U);
  EXPECT_TRUE(sem.has_value());
  EXPECT_EQ(sem.value(), SemanticLabel::CAR);

  constexpr auto compat = is_object_compatible(SemanticLabel::CAR);
  EXPECT_TRUE(compat);
}

}  // namespace autoware::ptv3::experimental
