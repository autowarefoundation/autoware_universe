// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__SCENARIO_GATE__SCENARIO_SELECTOR_BASE_HPP_
#define AUTOWARE__SCENARIO_GATE__SCENARIO_SELECTOR_BASE_HPP_

#include <string>

namespace autoware::scenario_selector
{
class ScenarioSelectorBase
{
public:
  virtual ~ScenarioSelectorBase() = default;

  // Implementations (DefaultSelector / ExtraSelector) 必須實作這個
  virtual std::string select() = 0;
};
}  // namespace autoware::scenario

#endif  // AUTOWARE__SCENARIO_GATE__SCENARIO_SELECTOR_BASE_HPP_
