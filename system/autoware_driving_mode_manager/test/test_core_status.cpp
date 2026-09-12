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

#include "core/status.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace autoware::driving_mode_manager
{

namespace
{

rclcpp::Time seconds_to_time(double seconds)
{
  const auto nanoseconds = static_cast<int64_t>(seconds * 1e9);
  return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

}  // namespace

TEST(TimeoutStatusTest, status_is_false_until_updated)
{
  // Fresh timeout status has no valid stamp/value yet.
  TimeoutStatus status;

  EXPECT_TRUE(status.timeout());
  EXPECT_FALSE(status.status());

  status.update(seconds_to_time(1.0), true);
  // After an explicit update, timeout clears and the value is reported.
  EXPECT_FALSE(status.timeout());
  EXPECT_TRUE(status.status());
}

TEST(TimeoutStatusTest, status_times_out_after_threshold)
{
  // Once elapsed duration exceeds timeout, the status is invalidated.
  TimeoutStatus status;

  status.update(seconds_to_time(1.0), true);
  status.update(seconds_to_time(2.1), 1.0);

  EXPECT_TRUE(status.timeout());
  EXPECT_FALSE(status.status());
}

TEST(DrivingModeStatusTest, readiness_depends_on_all_flags_for_all_modes)
{
  // Readiness requires every flag (available/active/stable/continuable)
  // to be non-timeout for every configured mode.
  const AutowareMode mode_a{1001};
  const AutowareMode mode_b{1002};
  DrivingModeStatus status({mode_a, mode_b});

  EXPECT_FALSE(status.is_ready());

  // Access per-mode status slots once and update them directly.
  auto * a = status.data(mode_a);
  auto * b = status.data(mode_b);
  ASSERT_NE(a, nullptr);
  ASSERT_NE(b, nullptr);

  // Mark every flag as fresh and true for both modes.
  // This represents a fully healthy state across the whole mode set.
  const auto now = seconds_to_time(0.0);
  a->available.update(now, true);
  a->active.update(now, true);
  a->stable.update(now, true);
  a->continuable.update(now, true);

  b->available.update(now, true);
  b->active.update(now, true);
  b->stable.update(now, true);
  b->continuable.update(now, true);

  // Once all flags are valid, readiness and mode-a flag queries should pass.
  EXPECT_TRUE(status.is_ready());
  EXPECT_TRUE(status.is_available(mode_a));
  EXPECT_TRUE(status.is_active(mode_a));
  EXPECT_TRUE(status.is_stable(mode_a));
  EXPECT_TRUE(status.is_continuable(mode_a));

  // A global timeout pass should drop all stale flags.
  status.update(seconds_to_time(2.0), 1.0);
  EXPECT_FALSE(status.is_ready());
  EXPECT_FALSE(status.is_available(mode_a));
  EXPECT_FALSE(status.is_active(mode_a));
  EXPECT_FALSE(status.is_stable(mode_a));
  EXPECT_FALSE(status.is_continuable(mode_a));
}

TEST(DrivingModeStatusTest, unknown_mode_returns_null_or_false)
{
  // Unknown mode lookups must fail safely.
  const AutowareMode known{1001};
  const AutowareMode unknown{9999};
  DrivingModeStatus status({known});

  EXPECT_EQ(status.data(unknown), nullptr);
  EXPECT_FALSE(status.is_available(unknown));
  EXPECT_FALSE(status.is_active(unknown));
  EXPECT_FALSE(status.is_stable(unknown));
  EXPECT_FALSE(status.is_continuable(unknown));
}

}  // namespace autoware::driving_mode_manager
