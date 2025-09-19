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

#include "autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"

#include <Eigen/Dense>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <vector>

namespace autoware::diffusion_planner::test
{
using autoware_planning_msgs::msg::Trajectory;

TEST(PostprocessingUtilsTest, CreateTrajectoryAndMultipleTrajectories)
{
  constexpr auto prediction_shape = OUTPUT_SHAPE;
  auto batch_size = prediction_shape[0];
  auto agent_size = prediction_shape[1];
  auto rows = prediction_shape[2];
  auto cols = prediction_shape[3];
  std::vector<float> data(batch_size * agent_size * rows * cols, 0.0f);
  // Fill with some values for checking
  for (size_t i = 0; i < data.size(); ++i) data[i] = static_cast<float>(i);

  std::vector<int64_t> shape{batch_size, agent_size, rows, cols};
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  rclcpp::Time stamp(123, 0);

  auto expected_points = prediction_shape[2];

  auto traj = postprocess::create_trajectory(data, stamp, transform, 0, 0);
  ASSERT_EQ(traj.points.size(), expected_points);
}

TEST(PostprocessingUtilsTest, ToCandidateTrajectoriesMsgPopulatesFields)
{
  using UUID = unique_identifier_msgs::msg::UUID;
  Trajectory traj;
  traj.header.frame_id = "map";
  traj.points.resize(2);
  UUID uuid;
  std::fill(uuid.uuid.begin(), uuid.uuid.end(), 42);

  auto msg = postprocess::to_candidate_trajectories_msg(traj, uuid, "test_generator");
  ASSERT_EQ(msg.candidate_trajectories.size(), 1);
  ASSERT_EQ(msg.generator_info.size(), 1);
  EXPECT_EQ(msg.candidate_trajectories[0].header.frame_id, "map");
  EXPECT_EQ(msg.generator_info[0].generator_name.data, "test_generator");
}

}  // namespace autoware::diffusion_planner::test
