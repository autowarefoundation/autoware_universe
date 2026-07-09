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

#include "autoware/diffusion_planner/conversion/agent.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"

#include <Eigen/Dense>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace autoware::diffusion_planner::test
{

using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;
using unique_identifier_msgs::msg::UUID;

namespace
{
HistoryAlignmentParams make_params()
{
  HistoryAlignmentParams params;
  params.enable = true;
  params.dt_sub_step_max = 0.11;
  params.yaw_rate_threshold = 0.01;
  params.max_extrapolation_time = 0.5;
  params.flip_yaw_threshold = 2.35619449;
  params.prune_grace = 0.5;
  return params;
}

TrackedObject make_object(const UUID & uuid, const double x, const double vx)
{
  TrackedObject object;
  object.object_id = uuid;
  object.kinematics.pose_with_covariance.pose.position.x = x;
  object.kinematics.pose_with_covariance.pose.position.y = 0.0;
  object.kinematics.pose_with_covariance.pose.orientation =
    autoware_utils_geometry::create_quaternion_from_yaw(0.0);
  object.kinematics.twist_with_covariance.twist.linear.x = vx;
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  object.shape.dimensions.z = 1.5;
  ObjectClassification classification;
  classification.label = ObjectClassification::CAR;
  classification.probability = 0.9;
  object.classification.push_back(classification);
  return object;
}

// rclcpp::Time(double) would treat the argument as nanoseconds; build from sec/nanosec instead.
rclcpp::Time to_time(const double stamp_sec)
{
  const auto sec = static_cast<int32_t>(stamp_sec);
  const auto nanosec = static_cast<uint32_t>((stamp_sec - static_cast<double>(sec)) * 1e9 + 0.5);
  return rclcpp::Time(sec, nanosec);
}

TrackedObjects make_msg(const std::vector<TrackedObject> & objects, const double stamp_sec)
{
  TrackedObjects msg;
  msg.header.stamp = to_time(stamp_sec);
  msg.objects = objects;
  return msg;
}

double latest_x(const AgentHistory & history)
{
  return history.get_latest_state().pose(0, 3);
}
}  // namespace

// A steadily moving agent, once re-timed, yields exactly INPUT_T_WITH_CURRENT uniformly spaced
// slots anchored at the frame time.
TEST(AgentLifecycleTest, UniformGridAndCount)
{
  const auto params = make_params();
  const UUID uuid = autoware_utils_uuid::generate_uuid();
  AgentData agent_data;

  // 5 observations at 0.1s spacing moving +10 m/s (x = 0,1,2,3,4).
  for (int i = 0; i < 5; ++i) {
    const double stamp = 100.0 + 0.1 * i;
    agent_data.update_histories(
      make_msg({make_object(uuid, static_cast<double>(i), 10.0)}, stamp), params);
  }

  const rclcpp::Time frame_time = to_time(100.4);
  const auto histories = agent_data.resampled_transformed_and_trimmed_histories(
    frame_time, Eigen::Matrix4d::Identity(), NEIGHBOR_SHAPE[1], params);

  ASSERT_EQ(histories.size(), 1u);
  // Exactly 31 slots x 11 features.
  EXPECT_EQ(
    histories[0].as_array().size(), static_cast<size_t>(INPUT_T_WITH_CURRENT) * AGENT_STATE_DIM);
  EXPECT_EQ(histories[0].size(), static_cast<size_t>(INPUT_T_WITH_CURRENT));

  // Frame time coincides with the newest observation, so the latest slot sits at x = 4.
  EXPECT_NEAR(latest_x(histories[0]), 4.0, 1e-6);

  // The two most recent slots are one grid step (0.1s * 10 m/s = 1.0 m) apart.
  const auto & states = histories[0].states();
  const double x_last = states.back().pose(0, 3);
  const double x_prev = states[states.size() - 2].pose(0, 3);
  EXPECT_NEAR(x_last - x_prev, 1.0, 1e-6);
}

// A message whose stamp does not advance (stale re-serve) is ignored, even if its contents differ.
TEST(AgentLifecycleTest, DedupOnNonAdvancingStamp)
{
  const auto params = make_params();
  const UUID uuid = autoware_utils_uuid::generate_uuid();
  AgentData agent_data;

  agent_data.update_histories(make_msg({make_object(uuid, 0.0, 0.0)}, 100.0), params);
  // Same stamp, wildly different position -> must be dropped.
  agent_data.update_histories(make_msg({make_object(uuid, 99.0, 0.0)}, 100.0), params);

  const rclcpp::Time frame_time = to_time(100.0);
  const auto histories = agent_data.resampled_transformed_and_trimmed_histories(
    frame_time, Eigen::Matrix4d::Identity(), NEIGHBOR_SHAPE[1], params);

  ASSERT_EQ(histories.size(), 1u);
  EXPECT_NEAR(latest_x(histories[0]), 0.0, 1e-6);  // stale x = 99 was ignored
}

// A disappeared agent is retained and frozen at its last observation (no forward extrapolation).
TEST(AgentLifecycleTest, DisappearedAgentIsFrozenNotExtrapolated)
{
  const auto params = make_params();
  const UUID uuid = autoware_utils_uuid::generate_uuid();
  AgentData agent_data;

  agent_data.update_histories(make_msg({make_object(uuid, 0.0, 10.0)}, 100.0), params);
  // Next tick: agent absent.
  agent_data.update_histories(make_msg({}, 100.1), params);

  const rclcpp::Time frame_time = to_time(100.1);
  const auto histories = agent_data.resampled_transformed_and_trimmed_histories(
    frame_time, Eigen::Matrix4d::Identity(), NEIGHBOR_SHAPE[1], params);

  ASSERT_EQ(histories.size(), 1u);
  // Frozen at x = 0, NOT extrapolated to ~1.0 that a live 10 m/s agent would reach in 0.1s.
  EXPECT_NEAR(latest_x(histories[0]), 0.0, 1e-6);
}

// A disappeared agent is pruned once it has aged fully out of the history window plus grace.
TEST(AgentLifecycleTest, PruneAfterAging)
{
  const auto params = make_params();
  const UUID uuid = autoware_utils_uuid::generate_uuid();
  AgentData agent_data;

  agent_data.update_histories(make_msg({make_object(uuid, 0.0, 0.0)}, 100.0), params);
  // Advance well beyond window (3.0s) + grace (0.5s).
  agent_data.update_histories(make_msg({}, 104.0), params);

  const rclcpp::Time frame_time = to_time(104.0);
  const auto histories = agent_data.resampled_transformed_and_trimmed_histories(
    frame_time, Eigen::Matrix4d::Identity(), NEIGHBOR_SHAPE[1], params);

  EXPECT_EQ(histories.size(), 0u);
}

// A re-identification (agent A vanishes, agent B appears elsewhere) does not teleport A onto B.
TEST(AgentLifecycleTest, UuidTransitionNoTeleport)
{
  const auto params = make_params();
  const UUID uuid_a = autoware_utils_uuid::generate_uuid();
  const UUID uuid_b = autoware_utils_uuid::generate_uuid();
  AgentData agent_data;

  agent_data.update_histories(make_msg({make_object(uuid_a, 0.0, 0.0)}, 100.0), params);
  // A disappears, B appears far away.
  agent_data.update_histories(make_msg({make_object(uuid_b, 50.0, 0.0)}, 100.1), params);

  const rclcpp::Time frame_time = to_time(100.1);
  const auto histories = agent_data.resampled_transformed_and_trimmed_histories(
    frame_time, Eigen::Matrix4d::Identity(), NEIGHBOR_SHAPE[1], params);

  ASSERT_EQ(histories.size(), 2u);
  // Sorted by distance to origin: A (frozen near 0) first, B (near 50) second. Neither teleports.
  EXPECT_NEAR(latest_x(histories[0]), 0.0, 1e-6);
  EXPECT_NEAR(latest_x(histories[1]), 50.0, 1e-6);
}

}  // namespace autoware::diffusion_planner::test
