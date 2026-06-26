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

#include "selector.hpp"

#include <memory>
#include <stdexcept>

namespace autoware::trajectory_gate
{

TrajectorySelector::TrajectorySelector()
{
  output_ = nullptr;
  current_source_id_ = invalid_source_id;
}

void TrajectorySelector::add_input(TrajectorySender * input, uint32_t source_id)
{
  if (source_id == invalid_source_id) {
    throw std::runtime_error("trajectory input is invalid source id: " + std::to_string(source_id));
  }

  const auto [iter, success] = inputs_.insert({source_id, input});
  if (!success) {
    throw std::runtime_error("trajectory input already exists: " + std::to_string(source_id));
  }
}

void TrajectorySelector::set_output(TrajectoryReceiver * output)
{
  if (output_) {
    throw std::runtime_error("trajectory output already exists");
  }
  output_ = output;
}

bool TrajectorySelector::select(uint32_t target_id)
{
  // NOTE: Switch to the invalid source if an unknown source is requested.

  if (current_source_id_ != invalid_source_id) {
    inputs_.at(current_source_id_)->set_output(nullptr);
    current_source_id_ = invalid_source_id;
  }

  const auto iter = inputs_.find(target_id);
  if (iter == inputs_.end()) {
    return false;
  }

  iter->second->set_output(output_);
  current_source_id_ = target_id;
  return true;
}

uint32_t TrajectorySelector::source() const
{
  return current_source_id_;
}

}  // namespace autoware::trajectory_gate
