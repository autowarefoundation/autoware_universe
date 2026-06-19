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

#include "autoware/ptv3/experimental/semantic_label_helper.hpp"

#include <gtest/gtest.h>

#include <cstdint>

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
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::MOTORCYCLE), 3U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::BICYCLE), 4U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::PEDESTRIAN), 5U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::ANIMAL), 6U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::HAZARD), 7U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::GROUND), 8U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::STRUCTURE), 9U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::VEGETATION), 10U);
  EXPECT_EQ(static_cast<std::uint8_t>(SemanticLabel::NOISE), 11U);
}

// ============================================================================
// String Conversion Tests
// ============================================================================

TEST_F(SemanticLabelTest, ToStringConversion)
{
  EXPECT_EQ(to_string(SemanticLabel::CAR), "CAR");
  EXPECT_EQ(to_string(SemanticLabel::TRUCK), "TRUCK");
  EXPECT_EQ(to_string(SemanticLabel::BUS), "BUS");
  EXPECT_EQ(to_string(SemanticLabel::MOTORCYCLE), "MOTORCYCLE");
  EXPECT_EQ(to_string(SemanticLabel::BICYCLE), "BICYCLE");
  EXPECT_EQ(to_string(SemanticLabel::PEDESTRIAN), "PEDESTRIAN");
  EXPECT_EQ(to_string(SemanticLabel::ANIMAL), "ANIMAL");
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
  EXPECT_EQ(car.value(), ObjectClassification::CAR);

  auto truck = try_into_object(SemanticLabel::TRUCK);
  EXPECT_TRUE(truck.has_value());
  EXPECT_EQ(truck.value(), ObjectClassification::TRUCK);

  auto bus = try_into_object(SemanticLabel::BUS);
  EXPECT_TRUE(bus.has_value());
  EXPECT_EQ(bus.value(), ObjectClassification::BUS);

  auto motorcycle = try_into_object(SemanticLabel::MOTORCYCLE);
  EXPECT_TRUE(motorcycle.has_value());
  EXPECT_EQ(motorcycle.value(), ObjectClassification::MOTORCYCLE);

  auto bicycle = try_into_object(SemanticLabel::BICYCLE);
  EXPECT_TRUE(bicycle.has_value());
  EXPECT_EQ(bicycle.value(), ObjectClassification::BICYCLE);

  auto pedestrian = try_into_object(SemanticLabel::PEDESTRIAN);
  EXPECT_TRUE(pedestrian.has_value());
  EXPECT_EQ(pedestrian.value(), ObjectClassification::PEDESTRIAN);

  auto animal = try_into_object(SemanticLabel::ANIMAL);
  EXPECT_TRUE(animal.has_value());
  EXPECT_EQ(animal.value(), ObjectClassification::ANIMAL);

  auto hazard = try_into_object(SemanticLabel::HAZARD);
  EXPECT_TRUE(hazard.has_value());
  EXPECT_EQ(hazard.value(), ObjectClassification::HAZARD);
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
  auto car = try_into_semantic(ObjectClassification::CAR);
  EXPECT_TRUE(car.has_value());
  EXPECT_EQ(car.value(), SemanticLabel::CAR);

  auto truck = try_into_semantic(ObjectClassification::TRUCK);
  EXPECT_TRUE(truck.has_value());
  EXPECT_EQ(truck.value(), SemanticLabel::TRUCK);

  auto bus = try_into_semantic(ObjectClassification::BUS);
  EXPECT_TRUE(bus.has_value());
  EXPECT_EQ(bus.value(), SemanticLabel::BUS);

  auto trailer = try_into_semantic(ObjectClassification::TRAILER);
  EXPECT_TRUE(trailer.has_value());
  EXPECT_EQ(trailer.value(), SemanticLabel::TRUCK);

  auto motorcycle = try_into_semantic(ObjectClassification::MOTORCYCLE);
  EXPECT_TRUE(motorcycle.has_value());
  EXPECT_EQ(motorcycle.value(), SemanticLabel::MOTORCYCLE);

  auto bicycle = try_into_semantic(ObjectClassification::BICYCLE);
  EXPECT_TRUE(bicycle.has_value());
  EXPECT_EQ(bicycle.value(), SemanticLabel::BICYCLE);

  auto pedestrian = try_into_semantic(ObjectClassification::PEDESTRIAN);
  EXPECT_TRUE(pedestrian.has_value());
  EXPECT_EQ(pedestrian.value(), SemanticLabel::PEDESTRIAN);

  auto animal = try_into_semantic(ObjectClassification::ANIMAL);
  EXPECT_TRUE(animal.has_value());
  EXPECT_EQ(animal.value(), SemanticLabel::ANIMAL);

  auto hazard = try_into_semantic(ObjectClassification::HAZARD);
  EXPECT_TRUE(hazard.has_value());
  EXPECT_EQ(hazard.value(), SemanticLabel::HAZARD);
}

TEST_F(SemanticLabelTest, TryIntoSemanticInvalidLabels)
{
  // Invalid ObjectClassification labels should return nullopt
  EXPECT_FALSE(try_into_semantic(ObjectClassification::UNKNOWN).has_value());  // UNKNOWN
  EXPECT_FALSE(try_into_semantic(ObjectClassification::OVER_DRIVABLE)
                 .has_value());  // OVER_DRIVABLE (not in semantic mapping)
  EXPECT_FALSE(try_into_semantic(ObjectClassification::UNDER_DRIVABLE)
                 .has_value());  // UNDER_DRIVABLE (not in semantic mapping)
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
  EXPECT_TRUE(is_object_compatible(SemanticLabel::MOTORCYCLE));
  EXPECT_TRUE(is_object_compatible(SemanticLabel::BICYCLE));
  EXPECT_TRUE(is_object_compatible(SemanticLabel::PEDESTRIAN));
  EXPECT_TRUE(is_object_compatible(SemanticLabel::ANIMAL));
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
  // TRAILER is intentionally normalized to TRUCK and does not roundtrip to TRAILER.
  struct RoundtripExpectation
  {
    ObjectLabel input;
    ObjectLabel expected_output;
  };

  constexpr RoundtripExpectation object_labels[] = {
    {ObjectClassification::CAR, ObjectClassification::CAR},
    {ObjectClassification::TRUCK, ObjectClassification::TRUCK},
    {ObjectClassification::BUS, ObjectClassification::BUS},
    {ObjectClassification::TRAILER, ObjectClassification::TRUCK},
    {ObjectClassification::MOTORCYCLE, ObjectClassification::MOTORCYCLE},
    {ObjectClassification::BICYCLE, ObjectClassification::BICYCLE},
    {ObjectClassification::PEDESTRIAN, ObjectClassification::PEDESTRIAN},
    {ObjectClassification::ANIMAL, ObjectClassification::ANIMAL},
    {ObjectClassification::HAZARD, ObjectClassification::HAZARD}};

  for (const auto & expectation : object_labels) {
    auto semantic = try_into_semantic(expectation.input);
    EXPECT_TRUE(semantic.has_value());

    auto back_to_object = try_into_object(semantic.value());
    EXPECT_TRUE(back_to_object.has_value());
    EXPECT_EQ(back_to_object.value(), expectation.expected_output);
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

  auto obj = try_into_object(SemanticLabel::CAR);
  EXPECT_TRUE(obj.has_value());
  EXPECT_EQ(obj.value(), ObjectClassification::CAR);

  auto sem = try_into_semantic(ObjectClassification::CAR);
  EXPECT_TRUE(sem.has_value());
  EXPECT_EQ(sem.value(), SemanticLabel::CAR);

  const auto compat = is_object_compatible(SemanticLabel::CAR);
  EXPECT_TRUE(compat);
}

}  // namespace autoware::ptv3::experimental
