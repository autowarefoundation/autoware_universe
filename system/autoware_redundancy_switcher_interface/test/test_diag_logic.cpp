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

#include "../src/diag_logic.hpp"

#include <gtest/gtest.h>

namespace autoware::redundancy_switcher
{

// Helpers
static std::optional<Annotated<SwitcherSignals>> make_switcher(
  bool stable, bool interrupted, bool faulted, const std::string & annotation = "test")
{
  return Annotated<SwitcherSignals>{{stable, interrupted, faulted}, annotation};
}

static constexpr double kTimeout = 1000.0;  // ms
static constexpr double kNow = 5000.0;      // arbitrary base time (ms)

// ── nullopt ────────────────────────────────────────────────────────────────

TEST(DiagLogicTest, NullSwitcher_IsWarn)
{
  const auto r = compute_switcher_level(std::nullopt, kNow, std::nullopt, kTimeout);
  EXPECT_EQ(r.level, DiagLevel::Warn);
  EXPECT_FALSE(r.transitional_start_ms.has_value());
}

// ── Non-transitional states ────────────────────────────────────────────────

TEST(DiagLogicTest, Stable_IsOk)
{
  const auto r = compute_switcher_level(make_switcher(true, false, false), kNow, std::nullopt, kTimeout);
  EXPECT_EQ(r.level, DiagLevel::Ok);
  EXPECT_FALSE(r.transitional_start_ms.has_value());
}

TEST(DiagLogicTest, SelfInterrupted_IsWarn)
{
  const auto r = compute_switcher_level(make_switcher(false, true, false), kNow, std::nullopt, kTimeout);
  EXPECT_EQ(r.level, DiagLevel::Warn);
  EXPECT_FALSE(r.transitional_start_ms.has_value());
}

TEST(DiagLogicTest, Faulted_IsError)
{
  const auto r = compute_switcher_level(make_switcher(false, false, true), kNow, std::nullopt, kTimeout);
  EXPECT_EQ(r.level, DiagLevel::Error);
  EXPECT_FALSE(r.transitional_start_ms.has_value());
}

// ── Transitional state: timer start ───────────────────────────────────────

TEST(DiagLogicTest, Transitional_FirstCall_StartsTimer)
{
  // No prior start → timestamp should be initialized to now_ms
  const auto r = compute_switcher_level(
    make_switcher(false, false, false), kNow, std::nullopt, kTimeout);
  EXPECT_EQ(r.level, DiagLevel::Warn);
  ASSERT_TRUE(r.transitional_start_ms.has_value());
  EXPECT_DOUBLE_EQ(*r.transitional_start_ms, kNow);
}

TEST(DiagLogicTest, Transitional_SubsequentCall_PreservesStartTime)
{
  // Timer already started at kNow - 100ms; must not be reset
  const double start = kNow - 100.0;
  const auto r = compute_switcher_level(
    make_switcher(false, false, false), kNow, start, kTimeout);
  EXPECT_EQ(r.level, DiagLevel::Warn);
  ASSERT_TRUE(r.transitional_start_ms.has_value());
  EXPECT_DOUBLE_EQ(*r.transitional_start_ms, start);
}

// ── Transitional state: timeout threshold ─────────────────────────────────

TEST(DiagLogicTest, Transitional_ExactlyAtTimeout_IsWarn)
{
  // elapsed == timeout is NOT over the threshold (strict >)
  const double start = kNow - kTimeout;
  const auto r = compute_switcher_level(
    make_switcher(false, false, false), kNow, start, kTimeout);
  EXPECT_EQ(r.level, DiagLevel::Warn);
}

TEST(DiagLogicTest, Transitional_JustOverTimeout_IsError)
{
  const double start = kNow - kTimeout - 1.0;
  const auto r = compute_switcher_level(
    make_switcher(false, false, false), kNow, start, kTimeout);
  EXPECT_EQ(r.level, DiagLevel::Error);
  ASSERT_TRUE(r.transitional_start_ms.has_value());
  EXPECT_DOUBLE_EQ(*r.transitional_start_ms, start);
}

// ── Timer cleared on non-transitional state ────────────────────────────────

TEST(DiagLogicTest, StableAfterTransitional_ClearsTimer)
{
  // Had an active timer; switching to stable must clear it
  const auto r = compute_switcher_level(
    make_switcher(true, false, false), kNow, kNow - 500.0, kTimeout);
  EXPECT_EQ(r.level, DiagLevel::Ok);
  EXPECT_FALSE(r.transitional_start_ms.has_value());
}

TEST(DiagLogicTest, FaultedAfterTransitional_ClearsTimer)
{
  const auto r = compute_switcher_level(
    make_switcher(false, false, true), kNow, kNow - 500.0, kTimeout);
  EXPECT_EQ(r.level, DiagLevel::Error);
  EXPECT_FALSE(r.transitional_start_ms.has_value());
}

// ── Message content ────────────────────────────────────────────────────────

TEST(DiagLogicTest, StableMessage_ContainsAnnotation)
{
  const auto r = compute_switcher_level(
    make_switcher(true, false, false, "my_annotation"), kNow, std::nullopt, kTimeout);
  EXPECT_NE(r.message.find("my_annotation"), std::string::npos);
}

TEST(DiagLogicTest, TimeoutMessage_ContainsElapsedMs)
{
  const double start = kNow - 1500.0;
  const auto r = compute_switcher_level(
    make_switcher(false, false, false), kNow, start, kTimeout);
  EXPECT_EQ(r.level, DiagLevel::Error);
  // message should contain elapsed time (1500ms)
  EXPECT_NE(r.message.find("1500"), std::string::npos);
}

}  // namespace autoware::redundancy_switcher
