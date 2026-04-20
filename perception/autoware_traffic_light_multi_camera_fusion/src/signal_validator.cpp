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

#include "signal_validator.hpp"
#include "types.hpp"

namespace autoware::traffic_light
{
namespace {
  inline StateKey signalLutToStateKey(const SignalLUT & lut){
    StateKey state_key;
    for (const auto & signal : lut) {
      state_key.emplace_back(std::make_pair(signal.first, signal.second));
    }

    return state_key;
  }

  inline SignalLUT createSignalLUT(const StateKey& state_key)
  {
    SignalLUT signal_lut;

    for (const auto& key : state_key) {
      signal_lut.insert(key);
    }

    return signal_lut;
  };

  inline SignalLUT extractCommonSignals(
    const SignalLUT & lut_a,
    const SignalLUT & lut_b)
  {
    SignalLUT common_lut;

    for (const auto & pair_a : lut_a) {
      auto it_b = lut_b.find(pair_a);
      if (it_b != lut_b.end()) {
        common_lut.insert(pair_a);
      }
    }

    return common_lut;
  }
}  // namespace

/**
 * @brief Compare two state keys and detect conflicts.
 * @param state_a First StateKey.
 * @param state_b Second StateKey.
 * @return Conflict status and common signals (StateKey).
 */
ConflictStatus SignalValidator::checkConflict(
  const StateKey & state_a, const StateKey & state_b)
{
  if (state_a == state_b) {
    return ConflictStatus{ConflictType::NO_CONFLICT, state_a};
  }

  SignalLUT lut_a = createSignalLUT(state_a);
  SignalLUT lut_b = createSignalLUT(state_b);

  constexpr std::pair<uint8_t, uint8_t> unknown_pair{
    TrafficLightElement::UNKNOWN, TrafficLightElement::UNKNOWN};

  auto it_unknown_a = lut_a.find(unknown_pair);
  auto it_unknown_b = lut_b.find(unknown_pair);

  // treat empty lookup table as unknown
  bool has_unknown_a = it_unknown_a != lut_a.end() || lut_a.size() == 0;
  bool has_unknown_b = it_unknown_b != lut_b.end() || lut_b.size() == 0;

  if (has_unknown_a && !has_unknown_b) {
    // we assume unknown is invalid detection and
    // only contains single unknown prediction (shape: unknown, color: unknown)
    return ConflictStatus{ConflictType::NO_CONFLICT, state_b};
  } else if (!has_unknown_a && has_unknown_b) {
    return ConflictStatus{ConflictType::NO_CONFLICT, state_a};
  } else if (has_unknown_a && has_unknown_b) {
    // unknown and missing inputs are treated the same
    // however, the returned state key depends on the input order
    return ConflictStatus{ConflictType::NO_CONFLICT, state_a};
  } else {
    const SignalLUT lut_common = extractCommonSignals(lut_a, lut_b);
    const StateKey common_state_key = signalLutToStateKey(lut_common);

    // all matching cases are already handled.
    // only need to check for full or partial conflicts.
    if (lut_common.size() > 0) {
      return ConflictStatus{ConflictType::PARTIAL_CONFLICT, common_state_key};
    } else {
      return ConflictStatus{ConflictType::CONFLICT, common_state_key};
    }
  }
}

}  // namespace autoware::traffic_light
