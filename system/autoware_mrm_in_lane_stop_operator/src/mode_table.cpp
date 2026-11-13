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

#include "mode_table.hpp"

#include <string>

namespace autoware::mrm_in_lane_stop_operator
{

void ModeTable::bind_id(const std::string & name, uint32_t id)
{
  for (auto & mode : modes_) {
    if (mode.name == name) {
      mode.mode_id = id;
    }
  }
}

ModeConfig * ModeTable::find_by_id(uint32_t id)
{
  for (auto & mode : modes_) {
    if (mode.mode_id == id) {
      return &mode;
    }
  }
  return nullptr;
}

const ModeConfig * ModeTable::find_by_id(uint32_t id) const
{
  for (const auto & mode : modes_) {
    if (mode.mode_id == id) {
      return &mode;
    }
  }
  return nullptr;
}

}  // namespace autoware::mrm_in_lane_stop_operator
