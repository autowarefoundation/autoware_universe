// Copyright 2025 TIER IV, Inc.
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

#include "autoware/simpl/archetype/agent.hpp"

#include "autoware/simpl/archetype/exception.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::simpl::archetype
{
std::vector<size_t> to_label_ids(const std::vector<std::string> & label_names)
{
  std::vector<size_t> output;
  for (const auto & name : label_names) {
    if (name == "VEHICLE") {
      output.push_back(static_cast<size_t>(AgentLabel::VEHICLE));
    } else if (name == "PEDESTRIAN") {
      output.push_back(static_cast<size_t>(AgentLabel::PEDESTRIAN));
    } else if (name == "MOTORCYCLIST") {
      output.push_back(static_cast<size_t>(AgentLabel::MOTORCYCLIST));
    } else if (name == "CYCLIST") {
      output.push_back(static_cast<size_t>(AgentLabel::CYCLIST));
    } else if (name == "LARGE_VEHICLE") {
      output.push_back(static_cast<size_t>(AgentLabel::LARGE_VEHICLE));
    } else if (name == "UNKNOWN") {
      output.push_back(static_cast<size_t>(AgentLabel::UNKNOWN));
    } else {
      std::ostringstream msg;
      msg << "Unexpected agent label name: " << name;
      throw SimplException(SimplError_t::InvalidValue, msg.str());
    }
  }
  return output;
}

AgentState AgentState::transform(const AgentState & to_state) const
{
  const auto to_cos = std::cos(to_state.yaw);
  const auto to_sin = std::sin(to_state.yaw);

  const auto tx = (x - to_state.x) * to_cos + (y - to_state.y) * to_sin;
  const auto ty = -(x - to_state.x) * to_sin + (y - to_state.y) * to_cos;
  const auto t_yaw = yaw - to_state.yaw;
  const auto t_vx = vx * to_cos + vy * to_sin;
  const auto t_vy = -vx * to_sin + vy * to_cos;

  return {tx, ty, z, t_yaw, t_vx, t_vy, label, is_valid};
}

AgentHistory AgentHistory::transform_to_current() const
{
  const auto & current_state = current();
  AgentHistory output(agent_id, queue_.size());
  for (const auto & state_t : *this) {
    output.update(state_t.transform(current_state));
  }
  return output;
}
}  // namespace autoware::simpl::archetype
