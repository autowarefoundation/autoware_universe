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

#include "util.hpp"

#include "plugin/default.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <utility>

InitData create_init_logic()
{
  auto plugin = std::make_shared<DefaultPlugin>();
  auto mock = std::make_unique<MockInterface>();
  InitData data;
  data.mock = mock.get();
  data.init = std::make_unique<ManagerInit>(std::move(mock), plugin);
  return data;
}

MainData create_main_logic()
{
  InitData data = create_init_logic();
  init_logic(*data.init);
  return {data.mock, std::make_unique<ManagerMain>(*data.init)};
}

void init_logic(ManagerInit & init)
{
  for (const auto & mode : init.config_->autoware_modes()) {
    init.on_available_flag(mode, true);
    init.on_active_flag(mode, true);
    init.on_stable_flag(mode, true);
    init.on_continuable_flag(mode, true);
  }
  init.on_trajectory_source(TrajectorySource{0});
  init.on_command_source(CommandSource{0});
  init.on_command_filter(CommandFilter{false});
  init.on_vehicle_control_mode(PlatformMode::kAutoware);
  EXPECT_TRUE(init.is_ready());
}
