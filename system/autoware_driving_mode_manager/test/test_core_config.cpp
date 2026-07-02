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

#include "core/config.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace autoware::driving_mode_manager
{

TEST(DrivingModeConfigTest, define_and_bind_maps_modes_correctly)
{
  // Build a minimal but complete configuration including operation and MRM modes.
  DrivingModeConfig config;
  const AutowareMode stop_mode{1001};
  const AutowareMode mrm_mode{2001};
  const OperationMode stop_operation = OperationMode::kStop;
  const MrmBehavior emergency_behavior{2};
  const TrajectorySource trajectory_source{10};
  const CommandSource command_source{20};

  config.define_trajectory_source(trajectory_source);
  config.define_command_source(command_source);
  config.define_autoware_mode(stop_mode, stop_operation, 7);
  config.define_autoware_mode(mrm_mode, emergency_behavior, 3);
  config.bind_name(stop_mode, "stop");
  config.bind_gates(stop_mode, DrivingModeConfig::Gates{trajectory_source, command_source});
  config.finalize();

  // Finalized mode list should contain every defined autoware mode.
  const auto modes = config.autoware_modes();
  EXPECT_EQ(modes.size(), 2U);
  EXPECT_NE(std::find(modes.begin(), modes.end(), stop_mode), modes.end());
  EXPECT_NE(std::find(modes.begin(), modes.end(), mrm_mode), modes.end());

  // Verify conversion tables and per-mode metadata.
  EXPECT_TRUE(config.exists(stop_mode));
  EXPECT_EQ(config.name(stop_mode), "stop");
  EXPECT_EQ(config.to_autoware_mode(stop_operation), stop_mode);
  EXPECT_EQ(config.to_operation_mode(stop_mode), stop_operation);
  EXPECT_EQ(config.to_autoware_mode(emergency_behavior), mrm_mode);
  EXPECT_EQ(config.to_mrm_behavior(mrm_mode), emergency_behavior);
  EXPECT_EQ(config.priority(stop_mode), 7);
  EXPECT_EQ(config.priority(AutowareMode{9999}), 0);

  // Gate mapping must resolve to the bound trajectory/command sources.
  const auto gates = config.gates(stop_mode);
  ASSERT_TRUE(gates.trajectory.has_value());
  ASSERT_TRUE(gates.command.has_value());
  EXPECT_EQ(gates.trajectory.value(), trajectory_source);
  EXPECT_EQ(gates.command.value(), command_source);
}

TEST(DrivingModeConfigTest, duplicate_mode_definition_throws)
{
  // Defining the same autoware mode twice is invalid.
  DrivingModeConfig config;
  const AutowareMode mode{1001};

  config.define_autoware_mode(mode, OperationMode::kStop, 1);

  EXPECT_THROW(
    config.define_autoware_mode(mode, OperationMode::kAutonomous, 1), std::invalid_argument);
}

TEST(DrivingModeConfigTest, bind_gates_rejects_unknown_sources)
{
  // Gate binding must reject sources that were never registered.
  DrivingModeConfig config;
  const AutowareMode mode{1001};

  config.define_autoware_mode(mode, OperationMode::kStop, 1);

  EXPECT_THROW(
    config.bind_gates(mode, DrivingModeConfig::Gates{TrajectorySource{123}, std::nullopt}),
    std::invalid_argument);
  EXPECT_THROW(
    config.bind_gates(mode, DrivingModeConfig::Gates{std::nullopt, CommandSource{456}}),
    std::invalid_argument);
}

TEST(DrivingModeConfigTest, bind_name_for_unknown_mode_throws)
{
  // Names can only be bound for already-defined autoware modes.
  DrivingModeConfig config;
  EXPECT_THROW(config.bind_name(AutowareMode{404}, "unknown"), std::invalid_argument);
}

}  // namespace autoware::driving_mode_manager
