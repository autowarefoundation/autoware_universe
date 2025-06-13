//  Copyright 2025 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef SWITCHERS__PULL_OVER_HPP_
#define SWITCHERS__PULL_OVER_HPP_

#include "common/plugin.hpp"

#include <string>

namespace autoware::command_mode_switcher
{

class PullOverSwitcher : public SwitcherPlugin
{
public:
  std::string name() const override { return "pull_over"; }
  std::string source() const override { return "main"; }
};

}  // namespace autoware::command_mode_switcher

#endif  // SWITCHERS__PULL_OVER_HPP_
