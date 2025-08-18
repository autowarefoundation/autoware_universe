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

#include <rclcpp/rclcpp.hpp>

namespace autoware::scenario_selector
{

class ScenarioSelectorPlugin
{
public:
  virtual ~ScenarioSelectorPlugin() = default;

  virtual void initialize(rclcpp::Node * node) = 0;

  virtual bool ready() const = 0;

  virtual std::string select() = 0;
};

}  // namespace autoware::scenario_selector

#endif  // AUTOWARE__SCENARIO_GATE__SCENARIO_SELECTOR_BASE_HPP_
