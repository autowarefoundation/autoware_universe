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

#include "mode_config.hpp"

#include <tier4_system_msgs/msg/in_lane_stop_trigger.hpp>

#include <stdexcept>
#include <string>
#include <unordered_map>

namespace autoware::mrm_in_lane_stop_operator
{

ProfileType ModeConfig::profile_from_name(const std::string & profile_name)
{
  using InLaneStopTrigger = tier4_system_msgs::msg::InLaneStopTrigger;
  static const std::unordered_map<std::string, ProfileType> kProfilesByName = {
    {"moderate", InLaneStopTrigger::PROFILE_MODERATE},
    {"emergency", InLaneStopTrigger::PROFILE_EMERGENCY},
  };
  const auto it = kProfilesByName.find(profile_name);
  if (it == kProfilesByName.end()) {
    throw std::invalid_argument(
      "mrm_in_lane_stop_operator: unknown profile '" + profile_name +
      "' (expected 'moderate' or 'emergency')");
  }
  return it->second;
}

}  // namespace autoware::mrm_in_lane_stop_operator
