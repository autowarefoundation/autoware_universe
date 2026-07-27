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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__UUID_HASH_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__UUID_HASH_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace autoware::trajectory_validator
{
/**
 * @brief Hashes a raw 16-byte UUID (FNV-1a) without allocating an intermediate string.
 *
 * Use as the hash policy for an std::unordered_map keyed on a UUID's raw bytes
 * (e.g. unique_identifier_msgs::msg::UUID::uuid), avoiding a per-lookup hex-string conversion.
 */
struct UuidHash
{
  std::size_t operator()(const std::array<uint8_t, 16> & uuid) const noexcept
  {
    std::size_t hash = 14695981039346656037ULL;  // FNV-1a offset basis
    for (const auto byte : uuid) {
      hash ^= static_cast<std::size_t>(byte);
      hash *= 1099511628211ULL;  // FNV-1a prime
    }
    return hash;
  }
};
}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__UUID_HASH_HPP_
