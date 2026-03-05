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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__UUID_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__UUID_HPP_

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <atomic>
#include <cstdint>
#include <mutex>

namespace autoware::multi_object_tracker::object_model
{

class UUIDGenerator
{
public:
  static unique_identifier_msgs::msg::UUID generate();

private:
  static void init();

  static std::once_flag init_flag_;
  static uint64_t namespace_prefix_;
  static std::atomic<uint64_t> counter_;
};

inline unique_identifier_msgs::msg::UUID generate_uuid()
{
  return UUIDGenerator::generate();
}

}  // namespace autoware::multi_object_tracker::object_model

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__UUID_HPP_
