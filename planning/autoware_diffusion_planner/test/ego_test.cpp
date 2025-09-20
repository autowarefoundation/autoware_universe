// Copyright 2025 TIER IV, Inc.
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

#include "ego_test.hpp"

#include <string>

namespace autoware::diffusion_planner::test
{

TEST_F(EgoTest, ConstructorInitialization)
{
  EgoState ego_state(odometry_msg_, acceleration_msg_, wheel_base_);
  // Ego state uses ego base link as reference frame, so x and y should be 0
  EXPECT_FLOAT_EQ(ego_state.x(), 0.0);
  EXPECT_FLOAT_EQ(ego_state.y(), 0.0);
  EXPECT_FLOAT_EQ(ego_state.vx(), 3.0);
  EXPECT_FLOAT_EQ(ego_state.vy(), 0.0);
  EXPECT_FLOAT_EQ(ego_state.ax(), 0.5);
  EXPECT_FLOAT_EQ(ego_state.ay(), 0.2);
  EXPECT_NEAR(ego_state.yaw_rate(), 0.1, 1e-5);
}

TEST_F(EgoTest, ToString)
{
  EgoState ego_state(odometry_msg_, acceleration_msg_, wheel_base_);
  std::string ego_state_str = ego_state.to_string();
  EXPECT_NE(ego_state_str.find("x: 0"), std::string::npos);
  EXPECT_NE(ego_state_str.find("y: 0"), std::string::npos);
  EXPECT_NE(ego_state_str.find("vx: 3"), std::string::npos);
  EXPECT_NE(ego_state_str.find("yaw_rate: 0.1"), std::string::npos);
}

TEST_F(EgoTest, AsArray)
{
  EgoState ego_state(odometry_msg_, acceleration_msg_, wheel_base_);
  auto data_array = ego_state.as_array();

  EXPECT_EQ(data_array.size(), EGO_STATE_DIM);
  EXPECT_FLOAT_EQ(data_array[0], 0.0);  // x
  EXPECT_FLOAT_EQ(data_array[1], 0.0);  // y
  EXPECT_FLOAT_EQ(data_array[4], 3.0);  // vx
  EXPECT_FLOAT_EQ(data_array[5], 0.0);  // vy
  EXPECT_FLOAT_EQ(data_array[6], 0.5);  // ax
  EXPECT_FLOAT_EQ(data_array[7], 0.2);  // ay
}

TEST_F(EgoTest, SteeringAngleClamp)
{
  // Set high angular velocity to test clamping
  odometry_msg_.twist.twist.angular.z = 5.0;

  EgoState ego_state(odometry_msg_, acceleration_msg_, wheel_base_);

  EXPECT_FLOAT_EQ(ego_state.yaw_rate(), EgoState::MAX_YAW_RATE);
}

TEST_F(EgoTest, YawRateClamp)
{
  // Set high yaw rate to test clamping
  odometry_msg_.twist.twist.angular.z = 2.0;

  EgoState ego_state(odometry_msg_, acceleration_msg_, wheel_base_);

  EXPECT_FLOAT_EQ(ego_state.yaw_rate(), EgoState::MAX_YAW_RATE);
}

}  // namespace autoware::diffusion_planner::test
