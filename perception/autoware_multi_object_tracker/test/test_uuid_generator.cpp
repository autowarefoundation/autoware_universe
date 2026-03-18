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

#include "autoware/multi_object_tracker/object_model/uuid.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

namespace
{
struct ArrayHash
{
  std::size_t operator()(const std::array<uint8_t, 16> & a) const noexcept
  {
    // FNV-1a 64-bit
    std::size_t hash = 1469598103934665603ULL;
    for (const auto b : a) {
      hash ^= static_cast<std::size_t>(b);
      hash *= 1099511628211ULL;
    }
    return hash;
  }
};
}  // namespace

TEST(UUIDGenerator, GeneratesUniqueValuesSingleThread)
{
  constexpr std::size_t N = 10000;
  std::unordered_set<std::array<uint8_t, 16>, ArrayHash> set;
  set.reserve(N);

  for (std::size_t i = 0; i < N; ++i) {
    const auto uuid = autoware::multi_object_tracker::object_model::generate_uuid();
    set.insert(uuid.uuid);
  }

  EXPECT_EQ(set.size(), N);
}

TEST(UUIDGenerator, GeneratesUniqueValuesMultiThread)
{
  constexpr std::size_t threads = 4;
  constexpr std::size_t per_thread = 5000;

  std::unordered_set<std::array<uint8_t, 16>, ArrayHash> set;
  set.reserve(threads * per_thread);
  std::mutex mutex;

  std::vector<std::thread> workers;
  workers.reserve(threads);

  for (std::size_t t = 0; t < threads; ++t) {
    workers.emplace_back([&]() {
      for (std::size_t i = 0; i < per_thread; ++i) {
        const auto uuid = autoware::multi_object_tracker::object_model::generate_uuid();
        std::lock_guard<std::mutex> lock(mutex);
        set.insert(uuid.uuid);
      }
    });
  }

  for (auto & w : workers) {
    w.join();
  }

  EXPECT_EQ(set.size(), threads * per_thread);
}
