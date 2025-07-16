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

#include "agent_test.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>

#include <algorithm>

namespace autoware::diffusion_planner::test
{

TEST_F(AgentTest, AgentStateInitialization)
{
  AgentState agent_state(tracked_object_);

  EXPECT_FLOAT_EQ(agent_state.x(), 1.0);
  EXPECT_FLOAT_EQ(agent_state.y(), 2.0);
  EXPECT_FLOAT_EQ(agent_state.z(), 0.0);
  EXPECT_FLOAT_EQ(agent_state.vx(), 3.0);
  EXPECT_FLOAT_EQ(agent_state.vy(), 4.0);
  EXPECT_FLOAT_EQ(agent_state.length(), 5.0);
  EXPECT_FLOAT_EQ(agent_state.width(), 2.0);
}

TEST_F(AgentTest, AgentStateTransformation)
{
  AgentState agent_state(tracked_object_);

  // Apply a simple transformation
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(0, 3) = 10.0;  // Translate x by 10
  transform(1, 3) = 5.0;   // Translate y by 5

  agent_state.apply_transform(transform);

  EXPECT_FLOAT_EQ(agent_state.x(), 11.0);
  EXPECT_FLOAT_EQ(agent_state.y(), 7.0);
}

TEST_F(AgentTest, AgentHistoryInitialization)
{
  AgentState agent_state(tracked_object_);
  AgentHistory agent_history(agent_state, 0, 100.0, 10);

  EXPECT_EQ(agent_history.length(), 10);
  EXPECT_EQ(agent_history.label_id(), 0);
  EXPECT_EQ(agent_history.object_id(), agent_state.object_id_);
}

TEST_F(AgentTest, AgentHistoryUpdate)
{
  AgentState agent_state(tracked_object_);
  AgentHistory agent_history(agent_state, 0, 100.0, 10);

  // Update with a new state
  tracked_object_.kinematics.pose_with_covariance.pose.position.x = 2.0;
  tracked_object_.kinematics.pose_with_covariance.pose.position.y = 3.0;
  agent_history.update(101.0, tracked_object_);

  EXPECT_FLOAT_EQ(agent_history.get_latest_state().x(), 2.0);
  EXPECT_FLOAT_EQ(agent_history.get_latest_state().y(), 3.0);
}

TEST_F(AgentTest, AgentDataInitialization)
{
  AgentData agent_data(tracked_objects_, 5, 10);

  EXPECT_EQ(agent_data.num_agent(), 1);
  EXPECT_EQ(agent_data.time_length(), 10);
  EXPECT_EQ(agent_data.size(), 10 * AgentState::dim());
}

TEST_F(AgentTest, AgentDataUpdateHistories)
{
  AgentData agent_data(tracked_objects_, 5, 10);

  // Update with a new object
  TrackedObject tracked_object_2;
  tracked_object_2.object_id = autoware_utils::generate_uuid();
  tracked_object_2.kinematics.pose_with_covariance.pose.position.x = 3.0;
  tracked_object_2.kinematics.pose_with_covariance.pose.position.y = 4.0;
  tracked_objects_.objects.push_back(tracked_object_2);
  agent_data.update_histories(tracked_objects_);

  EXPECT_EQ(agent_data.num_agent(), 2);
}

TEST_F(AgentTest, AgentDataTrimToClosestAgents)
{
  AgentData agent_data(tracked_objects_, 5, 10);

  // Add more agents
  for (int i = 0; i < 10; ++i) {
    TrackedObject tracked_object_extra;
    tracked_object_extra.object_id = autoware_utils::generate_uuid();
    tracked_object_extra.kinematics.pose_with_covariance.pose.position.x = i * 10.0;
    tracked_object_extra.kinematics.pose_with_covariance.pose.position.y = i * 10.0;
    tracked_objects_.objects.push_back(tracked_object_extra);
  }
  agent_data.update_histories(tracked_objects_);

  EXPECT_EQ(agent_data.num_agent(), 11);

  // Trim to 5 closest agents
  Point ego_position;
  ego_position.x = 0.0;
  ego_position.y = 0.0;
  agent_data.trim_to_k_closest_agents(ego_position);

  EXPECT_EQ(agent_data.num_agent(), 5);
}

}  // namespace autoware::diffusion_planner::test
