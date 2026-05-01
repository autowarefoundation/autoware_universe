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

#include "../src/signal_validator.hpp"
#include "../src/types.hpp"

#include <gtest/gtest.h>

#include <vector>

using autoware::traffic_light::ConflictStatus;
using autoware::traffic_light::ConflictType;
using autoware::traffic_light::SignalValidator;
using autoware::traffic_light::StateKey;
using tier4_perception_msgs::msg::TrafficLightElement;

class SignalValidatorCheckConflict : public ::testing::Test
{
protected:
  SignalValidator validator;
};

// ----------------------------------------------------------------------------
// test checkMismatchLogic
//  basic rules

// same color and shape: no conflict
TEST_F(SignalValidatorCheckConflict, RedCircle_RedCircle)
{
  StateKey input_a = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};
  StateKey input_b = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// different color and same shape
// -> critical conflict
TEST_F(SignalValidatorCheckConflict, RedCircle_GreenCircle)
{
  StateKey input_a = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};
  StateKey input_b = {{TrafficLightElement::GREEN, TrafficLightElement::CIRCLE}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::CONFLICT;
  StateKey expected_common_state_key = {};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// check swapped input
// different colors and same shape
// -> critical conflict
TEST_F(SignalValidatorCheckConflict, GreenCircle_RedCircle)
{
  StateKey input_a = {{TrafficLightElement::GREEN, TrafficLightElement::CIRCLE}};
  StateKey input_b = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::CONFLICT;
  StateKey expected_common_state_key = {};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// same colors and shapes with circles and arrows
// -> no conflict
TEST_F(SignalValidatorCheckConflict, RedCircleGreenLeftarrow_RedCircleGreenLeftarrow)
{
  StateKey input_a = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW}};
  StateKey input_b = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// same color for circles, and multiple matched arrows
// -> no conflict
TEST_F(SignalValidatorCheckConflict, RedCircleMultipleMatchedArrows_RedCircleMultipleMatchedArrows)
{
  StateKey input_a = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::DOWN_LEFT_ARROW}};
  StateKey input_b = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::DOWN_LEFT_ARROW}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::DOWN_LEFT_ARROW}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// different colors for circles, same color for arrows
// -> partial conflict
TEST_F(SignalValidatorCheckConflict, RedCircleGreenLeftarrow_GreenCircleGreenLeftarrow)
{
  StateKey input_a = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW},
  };
  StateKey input_b = {
    {TrafficLightElement::GREEN, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW},
  };

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::PARTIAL_CONFLICT;
  StateKey expected_common_state_key = {
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// same color for circles, different colors for arrows
// -> partial conflict
TEST_F(SignalValidatorCheckConflict, RedCircleGreenLeftarrow_RedCircleRedLeftarrow)
{
  StateKey input_a = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW},
  };
  StateKey input_b = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::RED, TrafficLightElement::LEFT_ARROW},
  };

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::PARTIAL_CONFLICT;
  StateKey expected_common_state_key = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// same color for circles, different colors for arrows
// -> partial conflict
TEST_F(SignalValidatorCheckConflict, GreenLeftarrowRedCircle_RedCircle)
{
  StateKey input_a = {
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW},
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE}};
  StateKey input_b = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
  };

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::PARTIAL_CONFLICT;
  StateKey expected_common_state_key = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// same color for circles, different colors for arrows (swapped input)
// -> partial conflict
TEST_F(SignalValidatorCheckConflict, RedCircle_GreenLeftarrowRedCircle)
{
  StateKey input_a = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
  };
  StateKey input_b = {
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW},
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::PARTIAL_CONFLICT;
  StateKey expected_common_state_key = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// same color for circles, and multiple matched arrows and single mismatched arrow
// -> partial conflict
TEST_F(SignalValidatorCheckConflict, RedCircleMultipleArrows_RedCircleMultipleArrows_ArrowMismatch)
{
  StateKey input_a = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::DOWN_LEFT_ARROW}};
  StateKey input_b = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::DOWN_ARROW}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::PARTIAL_CONFLICT;
  StateKey expected_common_state_key = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// ------------------------------------------
// test circle with arrow and only arrow
// -> critical conflict
TEST_F(SignalValidatorCheckConflict, RedCircleGreendownarrow_GreenUparrow)
{
  StateKey input_a = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::DOWN_ARROW}};
  StateKey input_b = {{TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::CONFLICT;
  StateKey expected_common_state_key = {};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// ------------------------------------------
// test other shapes

// same signal
// -> no conflict
TEST_F(SignalValidatorCheckConflict, WhiteCross_WhiteCross)
{
  StateKey input_a = {{TrafficLightElement::WHITE, TrafficLightElement::CROSS}};
  StateKey input_b = {{TrafficLightElement::WHITE, TrafficLightElement::CROSS}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {{TrafficLightElement::WHITE, TrafficLightElement::CROSS}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// different signal
// -> critical conflict
TEST_F(SignalValidatorCheckConflict, WhiteCross_WhiteCIRCLE)
{
  StateKey input_a = {{TrafficLightElement::WHITE, TrafficLightElement::CROSS}};
  StateKey input_b = {{TrafficLightElement::WHITE, TrafficLightElement::CIRCLE}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::CONFLICT;
  StateKey expected_common_state_key = {};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// same cross signal and different arrow signal
// -> critical conflict
TEST_F(SignalValidatorCheckConflict, AmberCross_AmberCrossGreenUparrow)
{
  StateKey input_a = {{TrafficLightElement::AMBER, TrafficLightElement::CROSS}};
  StateKey input_b = {
    {TrafficLightElement::AMBER, TrafficLightElement::CROSS},
    {TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::PARTIAL_CONFLICT;
  StateKey expected_common_state_key = {{TrafficLightElement::AMBER, TrafficLightElement::CROSS}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// inputs totally mismatch
TEST_F(SignalValidatorCheckConflict, AllMismatch)
{
  StateKey input_a = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW},
  };
  StateKey input_b = {
    {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW},
  };

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  StateKey expected_common_state_key = {};

  EXPECT_EQ(result.conflict_type, ConflictType::CONFLICT);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// ------------------------------------------
// test signal without circle

// same color and shape for arrows
// -> no conflict
TEST_F(SignalValidatorCheckConflict, GreenUparrow_GreenUparrow)
{
  StateKey input_a = {{TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW}};
  StateKey input_b = {{TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {
    {TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// same colors and shapes for arrows
// -> no conflict
TEST_F(SignalValidatorCheckConflict, GreenUparrowGreenRightarrow_GreenUparrowGreenRightarrow)
{
  StateKey input_a = {
    {TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW}};
  StateKey input_b = {
    {TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {
    {TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// different colors and same shape for arrows
// -> critical conflict
TEST_F(SignalValidatorCheckConflict, RedUparrow_GreenUparrow)
{
  StateKey input_a = {{TrafficLightElement::RED, TrafficLightElement::UP_ARROW}};
  StateKey input_b = {{TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::CONFLICT;
  StateKey expected_common_state_key = {};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// same color and different shapes for arrows
// -> critical conflict
TEST_F(SignalValidatorCheckConflict, RedUparrow_RedDownleftarrow)
{
  StateKey input_a = {{TrafficLightElement::RED, TrafficLightElement::UP_ARROW}};
  StateKey input_b = {{TrafficLightElement::RED, TrafficLightElement::DOWN_LEFT_ARROW}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::CONFLICT;
  StateKey expected_common_state_key = {};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// same color and different shapes for arrows
// -> critical conflict
TEST_F(SignalValidatorCheckConflict, RedUparrowRedDownleftarrow_RedDownleftarrow)
{
  StateKey input_a = {
    {TrafficLightElement::RED, TrafficLightElement::UP_ARROW},
    {TrafficLightElement::RED, TrafficLightElement::DOWN_LEFT_ARROW}};
  StateKey input_b = {{TrafficLightElement::RED, TrafficLightElement::DOWN_LEFT_ARROW}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::PARTIAL_CONFLICT;
  StateKey expected_common_state_key = {
    {TrafficLightElement::RED, TrafficLightElement::DOWN_LEFT_ARROW}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// same color and different shapes for arrows (swapped input)
// -> critical conflict
TEST_F(SignalValidatorCheckConflict, RedDownleftarrow_RedUparrowRedDownleftarrow)
{
  StateKey input_a = {{TrafficLightElement::RED, TrafficLightElement::DOWN_LEFT_ARROW}};
  StateKey input_b = {
    {TrafficLightElement::RED, TrafficLightElement::UP_ARROW},
    {TrafficLightElement::RED, TrafficLightElement::DOWN_LEFT_ARROW}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::PARTIAL_CONFLICT;
  StateKey expected_common_state_key = {
    {TrafficLightElement::RED, TrafficLightElement::DOWN_LEFT_ARROW}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// ------------------------------------------
// unknown detection

// unknown input
// -> no conflict (unknown should be treated as invalid detection)
TEST_F(SignalValidatorCheckConflict, RedCircle_Unknown)
{
  StateKey input_a = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};
  StateKey input_b = {{TrafficLightElement::UNKNOWN, TrafficLightElement::UNKNOWN}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// unknown input
// -> no conflict (unknown should be treated as invalid detection)
TEST_F(SignalValidatorCheckConflict, RedUparrow_Unknown)
{
  StateKey input_a = {{TrafficLightElement::RED, TrafficLightElement::UP_ARROW}};
  StateKey input_b = {{TrafficLightElement::UNKNOWN, TrafficLightElement::UNKNOWN}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {{TrafficLightElement::RED, TrafficLightElement::UP_ARROW}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// both unknown inputs
// -> no conflict (unknown should be treated as invalid detection)
TEST_F(SignalValidatorCheckConflict, Unknown_Unknown)
{
  StateKey input_a = {{TrafficLightElement::UNKNOWN, TrafficLightElement::UNKNOWN}};
  StateKey input_b = {{TrafficLightElement::UNKNOWN, TrafficLightElement::UNKNOWN}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {
    {TrafficLightElement::UNKNOWN, TrafficLightElement::UNKNOWN}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// missing input
// -> no conflict
TEST_F(SignalValidatorCheckConflict, RedCircle_NoDetection)
{
  StateKey input_a = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};
  StateKey input_b = {};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// missing input (swapped input)
// -> no conflict
TEST_F(SignalValidatorCheckConflict, NoDetection_RedCircle)
{
  StateKey input_a = {};
  StateKey input_b = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// missing input with arrow
// -> no conflict
TEST_F(SignalValidatorCheckConflict, RedUparrow_NoDetection)
{
  StateKey input_a = {{TrafficLightElement::RED, TrafficLightElement::UP_ARROW}};
  StateKey input_b = {};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {{TrafficLightElement::RED, TrafficLightElement::UP_ARROW}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// inputs with unknown and missing
// -> no conflict
TEST_F(SignalValidatorCheckConflict, Unknown_NoDetection)
{
  StateKey input_a = {{TrafficLightElement::UNKNOWN, TrafficLightElement::UNKNOWN}};
  StateKey input_b = {};

  // unknown and missing inputs will be treated as same,
  // but the returned state key depend on the input order
  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {
    {TrafficLightElement::UNKNOWN, TrafficLightElement::UNKNOWN}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// inputs with unknown and missing (swapped input)
// -> no conflict
TEST_F(SignalValidatorCheckConflict, NoDetection_Unknown)
{
  StateKey input_a = {};
  StateKey input_b = {{TrafficLightElement::UNKNOWN, TrafficLightElement::UNKNOWN}};

  // unknown and missing inputs will be treated as same,
  // but the returned state key depend on the input order
  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// ------------------------------------------
// test edge cases

// duplicated circle signal input
// -> partial conflict
TEST_F(SignalValidatorCheckConflict, RedCircleRedCircle_RedCircle)
{
  StateKey input_a = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE}};
  StateKey input_b = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::PARTIAL_CONFLICT;
  StateKey expected_common_state_key = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// mutiple same shape signal input
// -> partial conflict
TEST_F(SignalValidatorCheckConflict, RedCircleGreenCircle_RedCircle)
{
  StateKey input_a = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::CIRCLE}};
  StateKey input_b = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::PARTIAL_CONFLICT;
  StateKey expected_common_state_key = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// duplicated arrow signal input
// -> no conflict
TEST_F(
  SignalValidatorCheckConflict,
  RedCircleGreenDownrightarrow_RedCircleGreenDownrightarrowGreenDownrightarrow)
{
  StateKey input_a = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::DOWN_RIGHT_ARROW}};
  StateKey input_b = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::DOWN_RIGHT_ARROW},
    {TrafficLightElement::GREEN, TrafficLightElement::DOWN_RIGHT_ARROW}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::PARTIAL_CONFLICT;
  StateKey expected_common_state_key = {
    {TrafficLightElement::RED, TrafficLightElement::CIRCLE},
    {TrafficLightElement::GREEN, TrafficLightElement::DOWN_RIGHT_ARROW}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// ------------------------------------------
// test some edge cases

// input_b is empty
// empty input will treated as unknown prediction
TEST_F(SignalValidatorCheckConflict, EmptyInputs_1)
{
  StateKey input_a = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};
  StateKey input_b = {};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// input_a is empty
// empty input will treated as unknown prediction
TEST_F(SignalValidatorCheckConflict, EmptyInputs_2)
{
  StateKey input_a = {};
  StateKey input_b = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {{TrafficLightElement::RED, TrafficLightElement::CIRCLE}};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}

// both are empty
TEST_F(SignalValidatorCheckConflict, EmptyInputs_3)
{
  StateKey input_a = {};
  StateKey input_b = {};

  ConflictStatus result = validator.checkConflict(input_a, input_b);

  // output expectations
  ConflictType expected_conflict_type = ConflictType::NO_CONFLICT;
  StateKey expected_common_state_key = {};

  EXPECT_EQ(result.conflict_type, expected_conflict_type);
  EXPECT_EQ(result.common_state_key, expected_common_state_key);
}
