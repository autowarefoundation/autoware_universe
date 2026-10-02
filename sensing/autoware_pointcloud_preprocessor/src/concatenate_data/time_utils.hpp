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

#pragma once

#include <builtin_interfaces/msg/time.hpp>

#include <chrono>
#include <cstdint>

/// Package-internal stamp arithmetic for builtin_interfaces::msg::Time, replacing rclcpp::Time.
namespace autoware::pointcloud_preprocessor
{

/// Strict weak ordering for stamps.
struct TimeLess
{
  bool operator()(
    const builtin_interfaces::msg::Time & a, const builtin_interfaces::msg::Time & b) const
  {
    if (a.sec != b.sec) return a.sec < b.sec;
    return a.nanosec < b.nanosec;
  }
};

/// is_earlier(a, b) means "a is before b".
inline constexpr TimeLess is_earlier{};

/// stamp + duration. Floor-normalized, so nanosec stays in [0, 1e9) even for negative results.
inline builtin_interfaces::msg::Time add(
  const builtin_interfaces::msg::Time & stamp, const std::chrono::nanoseconds & duration)
{
  const int64_t total_nanoseconds =
    static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec + duration.count();
  int64_t sec = total_nanoseconds / 1'000'000'000LL;
  int64_t nanosec = total_nanoseconds % 1'000'000'000LL;
  if (nanosec < 0) {
    sec -= 1;
    nanosec += 1'000'000'000LL;
  }
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(sec);
  result.nanosec = static_cast<uint32_t>(nanosec);
  return result;
}

/// a - b in integer nanoseconds (exact, unlike subtracting double-second stamps).
inline std::chrono::nanoseconds subtract(
  const builtin_interfaces::msg::Time & a, const builtin_interfaces::msg::Time & b)
{
  return std::chrono::nanoseconds(
    (static_cast<int64_t>(a.sec) - b.sec) * 1'000'000'000LL +
    (static_cast<int64_t>(a.nanosec) - b.nanosec));
}

/// Stamp in double seconds. Rounds once; double(total nanoseconds) * 1e-9 would round twice.
inline double to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace autoware::pointcloud_preprocessor
