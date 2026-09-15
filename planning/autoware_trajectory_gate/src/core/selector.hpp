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

#ifndef CORE__SELECTOR_HPP_
#define CORE__SELECTOR_HPP_

#include "interface.hpp"

#include <cstdint>
#include <memory>
#include <unordered_map>

namespace autoware::trajectory_gate
{

class TrajectorySelector
{
public:
  TrajectorySelector();
  void add_input(TrajectorySender * input, uint32_t source_id);
  void set_output(TrajectoryReceiver * output);
  bool select(uint32_t target_id);
  uint32_t source() const;

private:
  void select_invalid_source();
  bool select_source(uint32_t target_id);

  static constexpr uint32_t invalid_source_id = 0;
  uint32_t current_source_id_;
  std::unordered_map<uint32_t, TrajectorySender *> inputs_;
  TrajectoryReceiver * output_;
};

}  // namespace autoware::trajectory_gate

#endif  // CORE__SELECTOR_HPP_
