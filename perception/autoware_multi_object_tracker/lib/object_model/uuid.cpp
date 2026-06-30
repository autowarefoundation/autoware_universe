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

#include <cstring>
#include <random>

namespace autoware::multi_object_tracker::object_model
{

std::once_flag UUIDGenerator::init_flag_;
uint64_t UUIDGenerator::namespace_prefix_{0U};
std::atomic<uint64_t> UUIDGenerator::counter_{1U};

void UUIDGenerator::init()
{
  // Generate a random 128-bit number as the namespace prefix. This ensures that UUIDs generated in
  // different runs of the program are different, even if the counter resets.
  std::random_device rd;
  const uint64_t hi = static_cast<uint64_t>(rd());
  const uint64_t lo = static_cast<uint64_t>(rd());
  namespace_prefix_ = (hi << 32U) ^ lo;
}

unique_identifier_msgs::msg::UUID UUIDGenerator::generate()
{
  std::call_once(init_flag_, &UUIDGenerator::init);

  const uint64_t count = counter_.fetch_add(1U, std::memory_order_relaxed);

  unique_identifier_msgs::msg::UUID uuid;
  // Put the counter first so the UUID string prefix changes per object.
  std::memcpy(uuid.uuid.data(), &count, sizeof(count));
  std::memcpy(uuid.uuid.data() + sizeof(count), &namespace_prefix_, sizeof(namespace_prefix_));
  return uuid;
}

}  // namespace autoware::multi_object_tracker::object_model
