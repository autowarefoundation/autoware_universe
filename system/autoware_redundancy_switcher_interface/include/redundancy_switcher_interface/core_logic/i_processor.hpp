//  Copyright 2025 The Autoware Contributors
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
#ifndef REDUNDANCY_SWITCHER_INTERFACE__CORE_LOGIC__I_PROCESSOR_HPP_
#define REDUNDANCY_SWITCHER_INTERFACE__CORE_LOGIC__I_PROCESSOR_HPP_

#include "redundancy_switcher_interface/ir/domain_types.hpp"
#include "redundancy_switcher_interface/ir/input_events.hpp"
#include "redundancy_switcher_interface/ir/output_commands.hpp"

#include <vector>

namespace autoware::redundancy_switcher
{

class IProcessor
{
public:
  virtual ~IProcessor() = default;

  virtual std::vector<OutputCommand> handle(const InputEvent & event) = 0;
  virtual DomainSnapshot snapshot() const = 0;
};

}  // namespace autoware::redundancy_switcher
#endif  // REDUNDANCY_SWITCHER_INTERFACE__CORE_LOGIC__I_PROCESSOR_HPP_
