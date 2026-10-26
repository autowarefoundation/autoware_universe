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

#ifndef MODE_CONFIG_HPP_
#define MODE_CONFIG_HPP_

#include <tier4_system_msgs/msg/in_lane_stop_trigger.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace autoware::mrm_in_lane_stop_operator
{

/// The wire type of InLaneStopTrigger::profile, taken from the message's own rosidl-generated
/// `_profile_type` alias rather than assumed (e.g. hardcoded as uint8_t) here, so this file never
/// needs updating if the message's underlying representation ever changes.
using ProfileType = tier4_system_msgs::msg::InLaneStopTrigger::_profile_type;

/// Configuration and runtime state for one driving mode managed by MrmInLaneStopOperator.
/// `name`/`profile`/`send_active_flag` come from the node's YAML parameters at startup;
/// `mode_id` is resolved later, once driving_mode_manager reports it by name over
/// DrivingModeInfo (see MrmInLaneStopOperator::on_info()).
struct ModeConfig
{
  std::string name;
  ProfileType profile;
  bool send_active_flag;
  std::optional<uint32_t> mode_id;

  /// Resolves a config-file profile name ("moderate"/"emergency") to the numeric
  /// InLaneStopTrigger::PROFILE_* value.
  /// @throws std::invalid_argument if `profile_name` is not a recognized profile name.
  static ProfileType profile_from_name(const std::string & profile_name);
};

}  // namespace autoware::mrm_in_lane_stop_operator

#endif  // MODE_CONFIG_HPP_
