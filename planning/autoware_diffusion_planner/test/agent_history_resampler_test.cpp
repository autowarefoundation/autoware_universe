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

#include "autoware/diffusion_planner/conversion/agent_history_resampler.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace autoware::diffusion_planner::test
{

namespace
{
HistoryResamplingParams make_params()
{
  HistoryResamplingParams params;
  params.max_extrapolation_time = 0.5;
  return params;
}

// Single-observation history of a car at (x, 0) heading +x with body-frame longitudinal speed vx.
AgentHistory make_single_obs_history(const double x, const double vx, const rclcpp::Time & stamp)
{
  autoware_perception_msgs::msg::TrackedObject object;
  object.object_id = autoware_utils_uuid::generate_uuid();
  object.kinematics.pose_with_covariance.pose.position.x = x;
  object.kinematics.pose_with_covariance.pose.orientation =
    autoware_utils_geometry::create_quaternion_from_yaw(0.0);
  object.kinematics.twist_with_covariance.twist.linear.x = vx;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 0.9;
  object.classification.push_back(classification);

  AgentHistory history(NEIGHBOR_HISTORY_BUFFER_SIZE);
  history.update(object, stamp);
  return history;
}
}  // namespace

// Constant-velocity: a straight-moving agent advances speed*dt along its heading, yaw unchanged.
TEST(AgentHistoryResamplerTest, PropagateConstantVelocityStraight)
{
  const auto params = make_params();
  const MotionState start{1.0, 2.0, 0.0};
  const auto out = propagate_motion(start, 3.0, 0.0, 0.1, params);

  EXPECT_NEAR(out.x, 1.0 + 3.0 * 0.1, 1e-9);
  EXPECT_NEAR(out.y, 2.0, 1e-9);
  EXPECT_NEAR(out.yaw, 0.0, 1e-9);
}

// A small yaw rate is applied as-is (no dead-band): yaw advances by yaw_rate*dt.
TEST(AgentHistoryResamplerTest, PropagateSmallYawRateIsApplied)
{
  const auto params = make_params();
  const MotionState start{0.0, 0.0, 0.0};
  const auto out = propagate_motion(start, 2.0, 0.005, 0.1, params);

  EXPECT_NEAR(out.yaw, 0.005 * 0.1, 1e-9);
}

// Constant-turn-rate: heading advances by yaw_rate*dt over a single sub-step.
TEST(AgentHistoryResamplerTest, PropagateConstantTurnRateSingleStep)
{
  const auto params = make_params();
  const MotionState start{0.0, 0.0, 0.0};
  const double w = 0.5;
  const double dt = 0.1;  // one fixed sub-step (0.1 s)
  const auto out = propagate_motion(start, 1.0, w, dt, params);

  EXPECT_NEAR(out.yaw, w * dt, 1e-9);
  EXPECT_NEAR(out.x, 1.0 * std::cos(0.0) * dt, 1e-9);
  EXPECT_NEAR(out.y, 1.0 * std::sin(0.0) * dt, 1e-9);
}

// Sub-stepping curves the path: a multi-step turn ends with the expected total heading change and
// a laterally displaced position (y > 0 for a positive turn rate).
TEST(AgentHistoryResamplerTest, PropagateConstantTurnRateSubStepped)
{
  const auto params = make_params();
  const MotionState start{0.0, 0.0, 0.0};
  const double w = 1.0;
  const double dt = 0.5;  // ceil(0.5 / 0.1) = 5 sub-steps
  const auto out = propagate_motion(start, 2.0, w, dt, params);

  EXPECT_NEAR(out.yaw, w * dt, 1e-9);
  EXPECT_GT(out.x, 0.0);
  EXPECT_GT(out.y, 0.0);  // curved left
}

// A zero dt is a no-op.
TEST(AgentHistoryResamplerTest, PropagateZeroDtIsNoOp)
{
  const auto params = make_params();
  const MotionState start{5.0, -3.0, 1.2};
  const auto out = propagate_motion(start, 4.0, 0.3, 0.0, params);
  EXPECT_EQ(out.x, start.x);
  EXPECT_EQ(out.y, start.y);
  EXPECT_EQ(out.yaw, start.yaw);
}

// A negative dt integrates the motion backward: a straight agent heading +x sits behind its start.
TEST(AgentHistoryResamplerTest, PropagateNegativeDtGoesBackward)
{
  const auto params = make_params();
  const MotionState start{1.0, 2.0, 0.0};
  const auto out = propagate_motion(start, 3.0, 0.0, -0.2, params);

  EXPECT_NEAR(out.x, 1.0 - 3.0 * 0.2, 1e-9);
  EXPECT_NEAR(out.y, 2.0, 1e-9);
  EXPECT_NEAR(out.yaw, 0.0, 1e-9);
}

// dt is clamped to max_extrapolation_time.
TEST(AgentHistoryResamplerTest, PropagateClampsDt)
{
  const auto params = make_params();  // max_extrapolation_time = 0.5
  const MotionState start{0.0, 0.0, 0.0};
  const auto out = propagate_motion(start, 1.0, 0.0, 2.0, params);

  EXPECT_NEAR(out.x, 0.5, 1e-9);  // 1.0 m/s * 0.5 s clamp
}

// Yaw interpolation takes the shortest arc across the +/-pi discontinuity.
TEST(AgentHistoryResamplerTest, InterpolateYawShortestArcAcrossPi)
{
  const double y = interpolate_yaw(3.0, -3.0, 0.5);
  // 3.0 and -3.0 are 0.283 rad apart the short way; the midpoint sits near +/-pi, not 0.
  EXPECT_NEAR(std::abs(y), M_PI, 1e-6);
}

TEST(AgentHistoryResamplerTest, InterpolateYawMidpoint)
{
  EXPECT_NEAR(interpolate_yaw(0.0, M_PI_2, 0.5), M_PI_4, 1e-9);
  EXPECT_NEAR(interpolate_yaw(0.0, M_PI_2, 0.0), 0.0, 1e-9);
  EXPECT_NEAR(interpolate_yaw(0.0, M_PI_2, 1.0), M_PI_2, 1e-9);
}

// A tracker heading flip between two observations snaps the interpolated slot to the nearer
// observation's yaw and speed; no perpendicular intermediate heading is emitted.
TEST(AgentHistoryResamplerTest, FlipBetweenObservationsSnapsToNearerYaw)
{
  const auto params = make_params();

  autoware_perception_msgs::msg::TrackedObject object;
  object.object_id = autoware_utils_uuid::generate_uuid();
  object.kinematics.pose_with_covariance.pose.orientation =
    autoware_utils_geometry::create_quaternion_from_yaw(0.0);
  object.kinematics.twist_with_covariance.twist.linear.x = 5.0;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 0.9;
  object.classification.push_back(classification);

  AgentHistory history(NEIGHBOR_HISTORY_BUFFER_SIZE);
  history.update(object, rclcpp::Time(100, 0));
  // Flipped re-expression of the same motion: heading rotated by pi, longitudinal speed negated.
  object.kinematics.pose_with_covariance.pose.position.x = 0.5;
  object.kinematics.pose_with_covariance.pose.orientation =
    autoware_utils_geometry::create_quaternion_from_yaw(M_PI);
  object.kinematics.twist_with_covariance.twist.linear.x = -5.0;
  history.update(object, rclcpp::Time(100, 100000000));

  // Frame time 100.14 s: the second-newest slot (100.04 s) interpolates across the flip.
  const rclcpp::Time frame_time(100, 140000000);
  const auto resampled = resample_history(history, frame_time, params);

  ASSERT_TRUE(resampled.has_value());
  const auto & states = resampled->states();
  const auto & snapped = states[states.size() - 2];
  EXPECT_NEAR(snapped.pose(0, 3), 0.2, 1e-6);  // position interpolates (ratio 0.4)
  EXPECT_NEAR(snapped.pose(0, 0), 1.0, 1e-6);  // yaw snaps to the nearer (older) observation
  EXPECT_NEAR(snapped.original_info.kinematics.twist_with_covariance.twist.linear.x, 5.0, 1e-6);
  // The newest slot extrapolates the flipped state; both re-expressions advance +x.
  EXPECT_NEAR(states.back().pose(0, 3), 0.7, 1e-6);
}

// A negative body-frame linear.x is a reversing vehicle: forward-extrapolated slots land behind
// the newest observation along its heading.
TEST(AgentHistoryResamplerTest, ReverseMotionExtrapolatesBackwardAlongHeading)
{
  const auto params = make_params();
  const rclcpp::Time obs_time(100, 0);
  const auto history = make_single_obs_history(0.0, -5.0, obs_time);

  // Frame time 0.2 s past the observation: the two newest slots extrapolate forward in time.
  const rclcpp::Time frame_time(100, 200000000);
  const auto resampled = resample_history(history, frame_time, params);

  ASSERT_TRUE(resampled.has_value());
  const auto & states = resampled->states();
  ASSERT_EQ(states.size(), static_cast<size_t>(INPUT_T_WITH_CURRENT));
  EXPECT_NEAR(states.back().pose(0, 3), -1.0, 1e-6);              // 0.2 s at -5 m/s
  EXPECT_NEAR(states[states.size() - 2].pose(0, 3), -0.5, 1e-6);  // 0.1 s at -5 m/s
  EXPECT_NEAR(states[states.size() - 3].pose(0, 3), 0.0, 1e-6);   // the observation itself
  // Backward extrapolation runs the reverse motion the other way: older slots sit ahead (+x).
  EXPECT_GT(states[states.size() - 4].pose(0, 3), 0.0);
}

}  // namespace autoware::diffusion_planner::test
