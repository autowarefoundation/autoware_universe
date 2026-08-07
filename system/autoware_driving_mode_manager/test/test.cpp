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

#include "util/util.hpp"

#include <gtest/gtest.h>

#include <iostream>

constexpr int loop_limit = 10;

TEST(TestSuite, ChangeStopMode)
{
  auto [mock, main] = create_main_logic();
  for (int i = 0; i < loop_limit; ++i) {
    main->update();
    mock->update();
    if (mock->operation_mode_state && !mock->operation_mode_state->is_in_transition) break;
  }
  EXPECT_EQ(mock->command_source.id, 11);
}

TEST(TestSuite, ChangeAutonomousMode)
{
  auto [mock, main] = create_main_logic();
  main->change_operation_mode(OperationMode::kAutonomous);
  for (int i = 0; i < loop_limit; ++i) {
    main->update();
    mock->update();
    if (mock->operation_mode_state && !mock->operation_mode_state->is_in_transition) break;
  }
  EXPECT_EQ(mock->trajectory_source.id, 100);
  EXPECT_EQ(mock->command_source.id, 12);
}
