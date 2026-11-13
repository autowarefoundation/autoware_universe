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

TEST(TestSuite, ChangeStopMode)
{
  auto [mock, main] = create_main_logic();
  wait_transition(mock, main.get());
  EXPECT_EQ(mock->trajectory_source.id, 101);
  EXPECT_EQ(mock->command_source.id, 11);
}

TEST(TestSuite, ChangeAutonomousMode)
{
  auto [mock, main] = create_main_logic();
  wait_transition(mock, main.get());
  main->change_operation_mode(OperationMode::kAutonomous);
  wait_transition(mock, main.get());
  EXPECT_EQ(mock->trajectory_source.id, 101);
  EXPECT_EQ(mock->command_source.id, 12);
}
