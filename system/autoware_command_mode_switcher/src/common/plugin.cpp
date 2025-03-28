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

#include "plugin.hpp"

#include <memory>
#include <string>

namespace autoware::command_mode_switcher
{

void SwitcherPlugin::construct(std::shared_ptr<SwitcherContext> context)
{
  context_ = context;
  status_.mode = name();
  status_.activation = false;
  status_.transition = false;
  status_.mrm = CommandModeStatusItem::NONE;
}

void SwitcherPlugin::request(bool activate)
{
  if (activate) {
    context_->select_source(source());
  }
}

void SwitcherPlugin::on_source_status(const CommandSourceStatus & msg)
{
  status_.activation = msg.source == source();
  status_.transition = msg.transition;
}

}  // namespace autoware::command_mode_switcher
