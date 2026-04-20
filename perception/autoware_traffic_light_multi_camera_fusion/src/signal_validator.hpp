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

#ifndef SIGNAL_VALIDATOR_HPP_
#define SIGNAL_VALIDATOR_HPP_

#include "types.hpp"

#include <tier4_perception_msgs/msg/traffic_light.hpp>

#include <unordered_set>

namespace autoware::traffic_light
{
struct SignalHash {
  size_t operator()(const std::pair<uint8_t, uint8_t> & keyval) const noexcept {
      // shift the first 8 bits to the left, and combine it with the second 8 bits
      // example: {0x01, 0x23} -> 0x0123
      return (static_cast<size_t>(keyval.first) << 8) | keyval.second;
  }
};

using SignalLUT = std::unordered_set<std::pair<uint8_t, uint8_t>, SignalHash>;

enum class ConflictType {
  NO_CONFLICT = 0, // when all the signals are the same
  CONFLICT,        // when all the signals are different
  PARTIAL_CONFLICT // whene some of the signals are different 
};

struct ConflictStatus {
  ConflictType conflict_type;
  StateKey common_state_key;
};

class SignalValidator
{
public:
  using TrafficLightElement = tier4_perception_msgs::msg::TrafficLightElement;

  ConflictStatus checkConflict(
    const StateKey & state_a, const StateKey & state_b);

  StateKey mergePartialMatch(
    const StateKey & state_a, const StateKey & state_b);
};

}  // namespace autoware::traffic_light

#endif  // SIGNAL_VALIDATOR_HPP_