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
#include "autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp"

#include <Eigen/Dense>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <gtest/gtest.h>
#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::test
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_planning_msgs::msg::Trajectory;

class PostprocessingUtilsEdgeCaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a simple tracked object for testing
    tracked_object_.object_id = autoware_utils_uuid::generate_uuid();
    tracked_object_.kinematics.pose_with_covariance.pose.position.x = 10.0;
    tracked_object_.kinematics.pose_with_covariance.pose.position.y = 5.0;
    tracked_object_.kinematics.pose_with_covariance.pose.position.z = 0.0;
    tracked_object_.kinematics.pose_with_covariance.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(0.0);
    tracked_object_.existence_probability = 0.9;
  }

  TrackedObject tracked_object_;
};

// Test edge case: Empty agent data (no neighbors)
TEST_F(PostprocessingUtilsEdgeCaseTest, CreatePredictedObjects_EmptyAgentData)
{
  std::vector<float> prediction(
    OUTPUT_SHAPE[0] * OUTPUT_SHAPE[1] * OUTPUT_SHAPE[2] * OUTPUT_SHAPE[3], 0.0f);

  TrackedObjects empty_objects;
  empty_objects.header.stamp = rclcpp::Time(0);

  AgentData agent_data(empty_objects, NEIGHBOR_SHAPE[1], NEIGHBOR_SHAPE[2], false);
  rclcpp::Time stamp(123, 0);
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  auto result = postprocess::create_predicted_objects(prediction, agent_data, stamp, transform);

  EXPECT_EQ(result.objects.size(), 0);
  EXPECT_EQ(result.header.frame_id, "map");
  EXPECT_EQ(result.header.stamp, stamp);
}

// Test edge case: More agents in prediction than in history
TEST_F(PostprocessingUtilsEdgeCaseTest, CreatePredictedObjects_MorePredictionsThanHistory)
{
  // Create prediction data for maximum agents
  std::vector<float> prediction(
    OUTPUT_SHAPE[0] * OUTPUT_SHAPE[1] * OUTPUT_SHAPE[2] * OUTPUT_SHAPE[3], 1.0f);

  // Create only 2 tracked objects
  TrackedObjects objects;
  objects.header.stamp = rclcpp::Time(0);
  objects.objects.push_back(tracked_object_);
  objects.objects.push_back(tracked_object_);

  AgentData agent_data(objects, NEIGHBOR_SHAPE[1], NEIGHBOR_SHAPE[2], false);
  rclcpp::Time stamp(123, 0);
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  auto result = postprocess::create_predicted_objects(prediction, agent_data, stamp, transform);

  // Should only create predictions for available objects (2)
  EXPECT_EQ(result.objects.size(), 2);
}

// Test edge case: NaN and Inf values in prediction
TEST_F(PostprocessingUtilsEdgeCaseTest, GetPredictionMatrix_NaNInfValues)
{
  std::vector<float> prediction(
    OUTPUT_SHAPE[0] * OUTPUT_SHAPE[1] * OUTPUT_SHAPE[2] * OUTPUT_SHAPE[3], 0.0f);

  // Insert NaN and Inf values at various positions
  prediction[0] = std::numeric_limits<float>::quiet_NaN();
  prediction[1] = std::numeric_limits<float>::infinity();
  prediction[2] = -std::numeric_limits<float>::infinity();

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  auto matrix = postprocess::get_prediction_matrix(prediction, transform, 0, 0);

  // Check that the matrix contains at least some problematic values
  bool has_nan = false;
  bool has_inf = false;

  for (int i = 0; i < matrix.rows(); ++i) {
    for (int j = 0; j < matrix.cols(); ++j) {
      if (std::isnan(matrix(i, j))) has_nan = true;
      if (std::isinf(matrix(i, j))) has_inf = true;
    }
  }

  // At least one of the special values should be present
  EXPECT_TRUE(has_nan || has_inf);
}

// Test edge case: Very large transformation values
TEST_F(PostprocessingUtilsEdgeCaseTest, TransformOutputMatrix_LargeTransform)
{
  Eigen::MatrixXf output_matrix = Eigen::MatrixXf::Ones(4, OUTPUT_T);
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  // Set very large translation values
  transform(0, 3) = 1e6f;
  transform(1, 3) = -1e6f;

  postprocess::transform_output_matrix(transform, output_matrix, 0, 0, true);

  // Check for overflow/precision issues
  EXPECT_FLOAT_EQ(output_matrix(0, 0), 1e6f + 1.0f);
  EXPECT_FLOAT_EQ(output_matrix(1, 0), -1e6f + 1.0f);
}

// Test edge case: Zero standard deviation in trajectory
TEST_F(PostprocessingUtilsEdgeCaseTest, GetTrajectoryFromPredictionMatrix_ZeroMovement)
{
  // Create a prediction matrix where the agent doesn't move
  Eigen::MatrixXf prediction_matrix(5, 4);
  prediction_matrix.setZero();
  prediction_matrix.col(2) = Eigen::VectorXf::Ones(5);  // cos(yaw) = 1

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  rclcpp::Time stamp(123, 0);

  auto trajectory =
    postprocess::get_trajectory_from_prediction_matrix(prediction_matrix, transform, stamp);

  ASSERT_EQ(trajectory.points.size(), 5);

  // All velocities should be zero (no movement)
  for (const auto & point : trajectory.points) {
    EXPECT_FLOAT_EQ(point.longitudinal_velocity_mps, 0.0f);
    EXPECT_FLOAT_EQ(point.pose.position.x, 0.0f);
    EXPECT_FLOAT_EQ(point.pose.position.y, 0.0f);
  }
}

// Test edge case: Extreme yaw values
TEST_F(PostprocessingUtilsEdgeCaseTest, GetTrajectoryFromPredictionMatrix_ExtremeYaw)
{
  Eigen::MatrixXf prediction_matrix(3, 4);
  prediction_matrix << 0, 0, 1, 0,  // yaw = 0
    1, 1, 0, 1,                     // yaw = π/2
    2, 2, -1, 0;                    // yaw = π

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  rclcpp::Time stamp(123, 0);

  auto trajectory =
    postprocess::get_trajectory_from_prediction_matrix(prediction_matrix, transform, stamp);

  ASSERT_EQ(trajectory.points.size(), 3);

  // Check yaw normalization
  auto yaw0 = tf2::getYaw(trajectory.points[0].pose.orientation);
  auto yaw1 = tf2::getYaw(trajectory.points[1].pose.orientation);
  auto yaw2 = tf2::getYaw(trajectory.points[2].pose.orientation);

  EXPECT_NEAR(yaw0, 0.0, 1e-5);
  EXPECT_NEAR(yaw1, M_PI_2, 1e-5);
  EXPECT_NEAR(std::abs(yaw2), M_PI, 1e-5);
}

// Test edge case: Boundary indices
TEST_F(PostprocessingUtilsEdgeCaseTest, CreateMultipleTrajectories_BoundaryIndices)
{
  constexpr auto batch_size = OUTPUT_SHAPE[0];
  constexpr auto agent_size = OUTPUT_SHAPE[1];

  std::vector<float> prediction(
    OUTPUT_SHAPE[0] * OUTPUT_SHAPE[1] * OUTPUT_SHAPE[2] * OUTPUT_SHAPE[3], 1.0f);
  rclcpp::Time stamp(123, 0);
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  // Test with max valid indices
  auto trajectories = postprocess::create_multiple_trajectories(
    prediction, stamp, transform, batch_size - 1, agent_size - 1);

  EXPECT_EQ(trajectories.size(), 1);  // Only one trajectory (last batch, last agent)

  // Test with out-of-bounds indices (should return empty)
  auto empty_trajectories =
    postprocess::create_multiple_trajectories(prediction, stamp, transform, batch_size, 0);

  EXPECT_EQ(empty_trajectories.size(), 0);
}

// Test edge case: Time precision
TEST_F(PostprocessingUtilsEdgeCaseTest, GetTrajectoryFromPredictionMatrix_TimePrecision)
{
  Eigen::MatrixXf prediction_matrix(100, 4);  // 100 time steps
  prediction_matrix.setZero();

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  rclcpp::Time stamp(123, 456789012);  // Include nanoseconds

  auto trajectory =
    postprocess::get_trajectory_from_prediction_matrix(prediction_matrix, transform, stamp);

  ASSERT_EQ(trajectory.points.size(), 100);

  // Check time precision for accumulated time
  const auto & last_point = trajectory.points.back();
  EXPECT_EQ(last_point.time_from_start.sec, 9);  // 99 * 0.1 = 9.9 seconds
  // Due to floating point precision, the nanoseconds might not be exactly 900000000
  EXPECT_NEAR(
    last_point.time_from_start.nanosec, 900000000, 1000);  // Allow 1 microsecond tolerance
}

// Test edge case: Negative prediction values
TEST_F(PostprocessingUtilsEdgeCaseTest, GetTensorData_NegativeValues)
{
  std::vector<float> data(OUTPUT_SHAPE[0] * OUTPUT_SHAPE[1] * OUTPUT_SHAPE[2] * OUTPUT_SHAPE[3]);

  // Fill with negative values
  for (size_t i = 0; i < data.size(); ++i) {
    data[i] = -static_cast<float>(i);
  }

  auto tensor_data = postprocess::get_tensor_data(data);

  // Verify negative values are preserved (skip first element which is 0)
  EXPECT_FLOAT_EQ(tensor_data(0, 0), 0.0f);  // First element is -0
  EXPECT_LT(tensor_data(0, 1), 0.0f);        // Second element should be negative
}

// Test edge case: UUID edge cases
TEST_F(PostprocessingUtilsEdgeCaseTest, ToCandidateTrajectoriesMsg_UUIDEdgeCases)
{
  using UUID = unique_identifier_msgs::msg::UUID;
  Trajectory traj;
  traj.header.frame_id = "map";

  // Test with all zeros UUID
  UUID zero_uuid;
  std::fill(zero_uuid.uuid.begin(), zero_uuid.uuid.end(), 0);

  auto msg1 = postprocess::to_candidate_trajectories_msg(traj, zero_uuid, "");
  EXPECT_EQ(msg1.generator_info[0].generator_name.data, "");

  // Test with all max values UUID
  UUID max_uuid;
  std::fill(max_uuid.uuid.begin(), max_uuid.uuid.end(), 255);

  // Test with very long generator name
  std::string long_name(1000, 'a');
  auto msg2 = postprocess::to_candidate_trajectories_msg(traj, max_uuid, long_name);
  EXPECT_EQ(msg2.generator_info[0].generator_name.data, long_name);
}

}  // namespace autoware::diffusion_planner::test
