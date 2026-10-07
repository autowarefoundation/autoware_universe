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

// Unit tests of ObjectUUIDGenerator: how a radar's 32-bit object id and the input topic name become
// a 16-byte UUID.

#include "radar_objects_adapter.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

namespace
{
using autoware::radar_objects_adapter::ObjectUUIDGenerator;

constexpr char topic_name[] = "/sensing/radar/front/objects_raw";

// The 8 bytes of the topic name's std::hash, least significant byte first.
std::array<std::uint8_t, 8> topic_hash_bytes(const std::string & topic)
{
  const std::size_t hash = std::hash<std::string>{}(topic);
  std::array<std::uint8_t, 8> bytes{};
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    bytes[i] = static_cast<std::uint8_t>((hash >> (i * 8)) & 0xFF);
  }
  return bytes;
}
}  // namespace

// The object id, least significant byte first, then the 8 bytes of the topic name's hash, least
// significant byte first, then 4 zero bytes.
TEST(ObjectUUIDGenerator, Layout_ObjectIdThenTopicHashThenZeros)
{
  const ObjectUUIDGenerator generate(topic_name);

  const auto uuid = generate(0x8899AABBu);

  const std::array<std::uint8_t, 8> hash = topic_hash_bytes(topic_name);
  const std::array<std::uint8_t, 16> expected = {
    0xBB,    0xAA,    0x99,    0x88,    hash[0], hash[1], hash[2], hash[3],
    hash[4], hash[5], hash[6], hash[7], 0,       0,       0,       0};
  EXPECT_EQ(uuid, expected);
}

// The same object id from the same topic gives the same UUID; a different object id or a
// different topic gives a different one.
TEST(ObjectUUIDGenerator, Identity_SameInputsSameUuidDifferentInputsDifferentUuid)
{
  const ObjectUUIDGenerator generate(topic_name);
  const ObjectUUIDGenerator generate_other("/sensing/radar/rear/objects_raw");

  EXPECT_EQ(generate(42u), generate(42u));
  EXPECT_NE(generate(42u), generate(43u));
  EXPECT_NE(generate(42u), generate_other(42u));
}

// Object id zero keeps the first four bytes zero; the hash bytes are still there.
TEST(ObjectUUIDGenerator, Zero_OnlyTheHashRemains)
{
  const ObjectUUIDGenerator generate(topic_name);

  const auto uuid = generate(0u);

  const std::array<std::uint8_t, 8> hash = topic_hash_bytes(topic_name);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(uuid[i], 0);
  }
  for (std::size_t i = 0; i < hash.size(); ++i) {
    EXPECT_EQ(uuid[4 + i], hash[i]);
  }
  for (std::size_t i = 12; i < 16; ++i) {
    EXPECT_EQ(uuid[i], 0);
  }
}
