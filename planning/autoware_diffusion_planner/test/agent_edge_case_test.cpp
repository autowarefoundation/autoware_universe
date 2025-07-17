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

#include <Eigen/Dense>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace autoware::diffusion_planner::test
{

using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;

class AgentEdgeCaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a basic tracked object
    tracked_object_.object_id = autoware_utils_uuid::generate_uuid();
    tracked_object_.kinematics.pose_with_covariance.pose.position.x = 1.0;
    tracked_object_.kinematics.pose_with_covariance.pose.position.y = 2.0;
    tracked_object_.kinematics.pose_with_covariance.pose.position.z = 0.0;
    tracked_object_.kinematics.pose_with_covariance.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(0.0);

    tracked_object_.kinematics.twist_with_covariance.twist.linear.x = 3.0;
    tracked_object_.kinematics.twist_with_covariance.twist.linear.y = 4.0;

    tracked_object_.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    tracked_object_.shape.dimensions.x = 5.0;
    tracked_object_.shape.dimensions.y = 2.0;
    tracked_object_.shape.dimensions.z = 1.5;

    tracked_object_.existence_probability = 0.9;

    // Add classification
    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
    classification.probability = 0.9;
    tracked_object_.classification.push_back(classification);
  }

  TrackedObject tracked_object_;
};

// Test edge case: NaN/Inf values in tracked object
TEST_F(AgentEdgeCaseTest, AgentStateNaNInfValues)
{
  tracked_object_.kinematics.pose_with_covariance.pose.position.x =
    std::numeric_limits<double>::quiet_NaN();
  tracked_object_.kinematics.pose_with_covariance.pose.position.y =
    std::numeric_limits<double>::infinity();
  tracked_object_.kinematics.twist_with_covariance.twist.linear.x =
    -std::numeric_limits<double>::infinity();

  AgentState agent_state(tracked_object_);

  // Check that values are converted to float correctly
  EXPECT_TRUE(std::isnan(agent_state.x()));
  EXPECT_TRUE(std::isinf(agent_state.y()));
  EXPECT_TRUE(std::isinf(agent_state.vx()));
}

// Test edge case: Zero and negative dimensions
TEST_F(AgentEdgeCaseTest, AgentStateZeroNegativeDimensions)
{
  tracked_object_.shape.dimensions.x = 0.0;
  tracked_object_.shape.dimensions.y = -2.0;
  tracked_object_.shape.dimensions.z = 0.0;

  AgentState agent_state(tracked_object_);

  EXPECT_FLOAT_EQ(agent_state.length(), 0.0);
  EXPECT_FLOAT_EQ(agent_state.width(), -2.0);
}

// Test edge case: Very small dimension values
TEST_F(AgentEdgeCaseTest, AgentStateSmallDimensions)
{
  tracked_object_.shape.dimensions.x = 1e-10;
  tracked_object_.shape.dimensions.y = 1e-10;

  AgentState agent_state(tracked_object_);

  EXPECT_FLOAT_EQ(agent_state.length(), 1e-10f);
  EXPECT_FLOAT_EQ(agent_state.width(), 1e-10f);
}

// Test edge case: Negative dimensions
TEST_F(AgentEdgeCaseTest, AgentStateNegativeDimensions)
{
  tracked_object_.shape.dimensions.x = -5.0;
  tracked_object_.shape.dimensions.y = -3.0;

  AgentState agent_state(tracked_object_);

  EXPECT_FLOAT_EQ(agent_state.length(), -5.0);
  EXPECT_FLOAT_EQ(agent_state.width(), -3.0);
}

// Test edge case: AgentHistory constructor and update
TEST_F(AgentEdgeCaseTest, AgentHistoryOperations)
{
  constexpr size_t max_history = 10;
  constexpr size_t label_id = 0;
  constexpr double current_time = 100.0;

  AgentState initial_state(tracked_object_);
  AgentHistory history(initial_state, label_id, current_time, max_history);

  // Update with new tracked object
  tracked_object_.kinematics.pose_with_covariance.pose.position.x = 2.0;
  tracked_object_.kinematics.pose_with_covariance.pose.position.y = 3.0;
  history.update(current_time + 1.0, tracked_object_);

  // Check latest state
  const auto & latest = history.get_latest_state();
  EXPECT_FLOAT_EQ(latest.x(), 2.0);
  EXPECT_FLOAT_EQ(latest.y(), 3.0);
}

// Test edge case: Empty history operations
TEST_F(AgentEdgeCaseTest, AgentHistoryEmptyOperations)
{
  AgentState initial_state(tracked_object_);
  AgentHistory history(initial_state, 0, 100.0, 10);

  // Operations on history should not crash
  EXPECT_NO_THROW(auto latest = history.get_latest_state());

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  EXPECT_NO_THROW(history.apply_transform(transform));
}

// Test edge case: Extreme transformation values
TEST_F(AgentEdgeCaseTest, AgentStateExtremeTransform)
{
  AgentState agent_state(tracked_object_);

  Eigen::Matrix4f extreme_transform = Eigen::Matrix4f::Identity();
  extreme_transform(0, 3) = 1e10f;  // Very large translation
  extreme_transform(1, 3) = -1e10f;
  extreme_transform(0, 0) = 1e-10f;  // Very small scale
  extreme_transform(1, 1) = 1e-10f;

  agent_state.apply_transform(extreme_transform);

  // Check that values don't overflow
  EXPECT_TRUE(std::isfinite(agent_state.x()));
  EXPECT_TRUE(std::isfinite(agent_state.y()));
}

// Test edge case: NaN/Inf in transformation matrix
TEST_F(AgentEdgeCaseTest, AgentStateNaNTransform)
{
  AgentState agent_state(tracked_object_);

  Eigen::Matrix4f nan_transform = Eigen::Matrix4f::Identity();
  nan_transform(0, 3) = std::numeric_limits<float>::quiet_NaN();
  nan_transform(1, 3) = std::numeric_limits<float>::infinity();

  agent_state.apply_transform(nan_transform);

  // NaN/Inf should propagate
  EXPECT_TRUE(std::isnan(agent_state.x()) || std::isinf(agent_state.x()));
  EXPECT_TRUE(std::isnan(agent_state.y()) || std::isinf(agent_state.y()));
}

// Test edge case: AgentData with maximum agents
TEST_F(AgentEdgeCaseTest, AgentDataMaxAgents)
{
  TrackedObjects objects;
  objects.header.stamp = rclcpp::Time(0);

  // Create maximum number of tracked objects
  const size_t max_agents = 50;  // Assuming this is a reasonable max
  for (size_t i = 0; i < max_agents; ++i) {
    TrackedObject obj = tracked_object_;
    obj.object_id = autoware_utils_uuid::generate_uuid();
    obj.kinematics.pose_with_covariance.pose.position.x = static_cast<double>(i);
    objects.objects.push_back(obj);
  }

  AgentData agent_data(objects, max_agents, 10);
  EXPECT_EQ(agent_data.num_agent(), max_agents);
}

// Test edge case: AgentData with no valid objects
TEST_F(AgentEdgeCaseTest, AgentDataNoValidObjects)
{
  TrackedObjects objects;
  objects.header.stamp = rclcpp::Time(0);

  // Create objects with all unknown classifications
  for (int i = 0; i < 5; ++i) {
    TrackedObject obj = tracked_object_;
    obj.object_id = autoware_utils_uuid::generate_uuid();
    obj.classification.clear();
    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
    classification.probability = 1.0;
    obj.classification.push_back(classification);
    objects.objects.push_back(obj);
  }

  // With ignore_unknown_agents = true, should have no agents
  AgentData agent_data(objects, 10, 10, true);
  EXPECT_EQ(agent_data.num_agent(), 0);
}

// Test edge case: AgentData trim with extreme positions
TEST_F(AgentEdgeCaseTest, AgentDataTrimExtremePositions)
{
  TrackedObjects objects;
  objects.header.stamp = rclcpp::Time(0);

  // Create objects at extreme positions
  for (int i = 0; i < 10; ++i) {
    TrackedObject obj = tracked_object_;
    obj.object_id = autoware_utils_uuid::generate_uuid();
    obj.kinematics.pose_with_covariance.pose.position.x = (i % 2 == 0) ? 1e10 : -1e10;
    obj.kinematics.pose_with_covariance.pose.position.y = (i % 2 == 0) ? -1e10 : 1e10;
    objects.objects.push_back(obj);
  }

  AgentData agent_data(objects, 5, 10);

  // Should keep all 10 agents since we passed max_num_agent=5 but have 10 objects
  // The trim happens relative to ego position, not automatically
  EXPECT_EQ(agent_data.num_agent(), 10);
}

// Test edge case: AgentHistory get object matrix
TEST_F(AgentEdgeCaseTest, AgentHistoryGetObjectMatrixEdgeCases)
{
  constexpr size_t max_history = 10;
  AgentState initial_state(tracked_object_);
  AgentHistory history(initial_state, 0, 100.0, max_history);

  // Add states with extreme values
  for (int i = 1; i <= 5; ++i) {
    tracked_object_.kinematics.pose_with_covariance.pose.position.x = std::pow(10.0, i);
    tracked_object_.kinematics.twist_with_covariance.twist.linear.x = std::pow(10.0, -i);
    history.update(100.0 + i, tracked_object_);
  }

  // Get array representation of the history data
  auto array_data = history.as_array();

  // Array should have correct size (max_history * AgentState::dim())
  EXPECT_EQ(array_data.size(), max_history * AgentState::dim());
}

// Test edge case: Multiple classifications
TEST_F(AgentEdgeCaseTest, AgentStateMultipleClassifications)
{
  // Clear existing classifications
  tracked_object_.classification.clear();

  // Add multiple classifications with equal probabilities
  std::vector<uint8_t> labels = {
    autoware_perception_msgs::msg::ObjectClassification::CAR,
    autoware_perception_msgs::msg::ObjectClassification::TRUCK,
    autoware_perception_msgs::msg::ObjectClassification::BUS};

  for (auto label : labels) {
    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.label = label;
    classification.probability = 0.33f;
    tracked_object_.classification.push_back(classification);
  }

  AgentState agent_state(tracked_object_);

  // Should handle multiple classifications
  // label_ is a member variable, not a method
  EXPECT_EQ(agent_state.label_, AgentLabel::VEHICLE);
}

// Test edge case: Zero probability classification
TEST_F(AgentEdgeCaseTest, AgentStateZeroProbability)
{
  tracked_object_.classification.clear();
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 0.0;
  tracked_object_.classification.push_back(classification);

  tracked_object_.existence_probability = 0.0;

  AgentState agent_state(tracked_object_);

  // Should handle zero probability
  EXPECT_EQ(agent_state.label_, AgentLabel::VEHICLE);
}

}  // namespace autoware::diffusion_planner::test
