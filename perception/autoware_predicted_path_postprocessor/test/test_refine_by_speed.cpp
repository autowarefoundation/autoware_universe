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

#include "autoware/predicted_path_postprocessor/processor/refine_by_speed.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace autoware::predicted_path_postprocessor::testing
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using processor::Context;
using processor::RefineBySpeed;

class RefineBySpeedTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    const auto package_dir =
      ament_index_cpp::get_package_share_directory("autoware_predicted_path_postprocessor");

    auto node_options = rclcpp::NodeOptions();
    node_options.arguments(
      {"--ros-args", "--params-file",
       package_dir + "/config/predicted_path_postprocessor.param.yaml"});

    node_ = std::make_shared<rclcpp::Node>("test_node", node_options);

    processor_ = std::make_unique<RefineBySpeed>(node_.get(), "refine_by_speed");
  }

  void TearDown() override { rclcpp::shutdown(); }

  PredictedObject createTestObject(
    double speed, const std::vector<std::array<double, 3>> & waypoints, double time_step = 0.1)
  {
    PredictedObject object;

    // Set initial kinematics with speed
    object.kinematics.initial_twist_with_covariance.twist.linear.x = speed;

    // Create predicted path
    PredictedPath path;
    path.time_step = rclcpp::Duration::from_seconds(time_step);

    for (const auto & wp : waypoints) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = wp[0];
      pose.position.y = wp[1];
      pose.position.z = wp[2];
      pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
      path.path.push_back(pose);
    }

    object.kinematics.predicted_paths.push_back(path);

    return object;
  }

  double calculateDistance(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
  {
    return std::sqrt(
      std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2) + std::pow(p2.z - p1.z, 2));
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<RefineBySpeed> processor_;
};

TEST_F(RefineBySpeedTest, ConstructorInitializesCorrectly)
{
  EXPECT_EQ(processor_->name(), "refine_by_speed");
}

TEST_F(RefineBySpeedTest, SkipsHighSpeedObjects)
{
  // Create object with speed above threshold (1.0 m/s)
  auto object = createTestObject(10.0, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}});
  auto original_path = object.kinematics.predicted_paths[0].path;

  Context context;
  ASSERT_TRUE(processor_->run(object, context));

  // Path should remain unchanged for high-speed objects
  auto & processed_path = object.kinematics.predicted_paths[0].path;
  ASSERT_EQ(processed_path.size(), original_path.size());

  for (size_t i = 0; i < processed_path.size(); ++i) {
    EXPECT_NEAR(processed_path[i].position.x, original_path[i].position.x, 1e-6);
    EXPECT_NEAR(processed_path[i].position.y, original_path[i].position.y, 1e-6);
    EXPECT_NEAR(processed_path[i].position.z, original_path[i].position.z, 1e-6);
  }
}

TEST_F(RefineBySpeedTest, ProcessesLowSpeedObjects)
{
  // Create object with speed below threshold (1.0 m/s)
  auto object =
    createTestObject(0.05, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}});
  auto original_path = object.kinematics.predicted_paths[0].path;

  Context context;
  processor_->run(object, context);

  // Path should be modified for low-speed objects
  auto & processed_path = object.kinematics.predicted_paths[0].path;
  ASSERT_EQ(processed_path.size(), original_path.size());

  // First waypoint should remain unchanged (object center)
  EXPECT_NEAR(processed_path[0].position.x, original_path[0].position.x, 1e-6);
  EXPECT_NEAR(processed_path[0].position.y, original_path[0].position.y, 1e-6);

  // Subsequent waypoints should be closer to the start due to low speed
  for (size_t i = 1; i < processed_path.size(); ++i) {
    double original_distance =
      calculateDistance(original_path[0].position, original_path[i].position);
    double processed_distance =
      calculateDistance(processed_path[0].position, processed_path[i].position);
    EXPECT_LT(processed_distance, original_distance);
  }
}

TEST_F(RefineBySpeedTest, HandlesZeroSpeed)
{
  // Create object with zero speed
  auto object = createTestObject(0.0, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}});

  Context context;
  ASSERT_TRUE(processor_->run(object, context));

  auto & processed_path = object.kinematics.predicted_paths[0].path;
  ASSERT_EQ(processed_path.size(), 3);

  // For zero speed, all waypoints except the first should collapse to the starting position
  EXPECT_NEAR(processed_path[0].position.x, 0.0, 1e-6);
  EXPECT_NEAR(processed_path[1].position.x, 0.0, 1e-6);
  EXPECT_NEAR(processed_path[2].position.x, 0.0, 1e-6);
}

TEST_F(RefineBySpeedTest, HandlesNegativeSpeed)
{
  // Create object with negative speed (should use absolute value)
  auto object = createTestObject(-0.05, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}});
  auto original_path = object.kinematics.predicted_paths[0].path;

  Context context;
  ASSERT_TRUE(processor_->run(object, context));

  // Should process the object (using absolute speed value)
  auto & processed_path = object.kinematics.predicted_paths[0].path;

  // Verify that refinement occurred (distances should be smaller)
  for (size_t i = 1; i < processed_path.size(); ++i) {
    double original_distance =
      calculateDistance(original_path[0].position, original_path[i].position);
    double processed_distance =
      calculateDistance(processed_path[0].position, processed_path[i].position);
    EXPECT_LT(processed_distance, original_distance);
  }
}

TEST_F(RefineBySpeedTest, HandlesEmptyPath)
{
  auto object = createTestObject(0.05, {});

  Context context;
  // Should not crash with empty path
  ASSERT_TRUE(processor_->run(object, context));
}

TEST_F(RefineBySpeedTest, HandlesSingleWaypoint)
{
  auto object = createTestObject(0.05, {{0.0, 0.0, 0.0}});

  Context context;
  ASSERT_TRUE(processor_->run(object, context));

  // Single waypoint should remain unchanged
  auto & processed_path = object.kinematics.predicted_paths[0].path;
  ASSERT_EQ(processed_path.size(), 1);
  EXPECT_NEAR(processed_path[0].position.x, 0.0, 1e-6);
}

TEST_F(RefineBySpeedTest, HandlesZeroTimeStep)
{
  auto object = createTestObject(0.05, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, 0.0);

  Context context;
  ASSERT_TRUE(processor_->run(object, context));

  // Should skip processing when time_step is zero
  auto & processed_path = object.kinematics.predicted_paths[0].path;
  EXPECT_NEAR(processed_path[1].position.x, 1.0, 1e-6);  // Should remain unchanged
}

TEST_F(RefineBySpeedTest, HandlesMultiplePredictedPaths)
{
  PredictedObject object;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = 0.05;  // Low speed

  // Add two predicted paths
  for (int path_idx = 0; path_idx < 2; ++path_idx) {
    PredictedPath path;
    path.time_step = rclcpp::Duration::from_seconds(0.1);

    std::vector<std::array<double, 3>> waypoints = {
      {0.0, static_cast<double>(path_idx), 0.0},
      {1.0, static_cast<double>(path_idx), 0.0},
      {2.0, static_cast<double>(path_idx), 0.0}};

    for (const auto & wp : waypoints) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = wp[0];
      pose.position.y = wp[1];
      pose.position.z = wp[2];
      pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
      path.path.push_back(pose);
    }

    object.kinematics.predicted_paths.push_back(path);
  }

  Context context;
  ASSERT_TRUE(processor_->run(object, context));

  // Both paths should be processed
  ASSERT_EQ(object.kinematics.predicted_paths.size(), 2);
  for (const auto & path : object.kinematics.predicted_paths) {
    // Verify refinement occurred
    double distance_1_2 = calculateDistance(path.path[0].position, path.path[1].position);
    double distance_2_3 = calculateDistance(path.path[1].position, path.path[2].position);
    EXPECT_LT(distance_1_2, 1.0);  // Should be less than original 1.0
    EXPECT_LT(distance_2_3, 1.0);  // Should be less than original 1.0
  }
}

TEST_F(RefineBySpeedTest, PreservesOrientationCalculation)
{
  auto object = createTestObject(0.05, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}});

  Context context;
  ASSERT_TRUE(processor_->run(object, context));

  auto & processed_path = object.kinematics.predicted_paths[0].path;

  // Verify that orientations are properly calculated based on direction
  for (size_t i = 1; i < processed_path.size(); ++i) {
    // Orientation should be valid quaternion
    auto & quat = processed_path[i].orientation;
    double norm = std::sqrt(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);
    EXPECT_NEAR(norm, 1.0, 1e-6);
  }
}

TEST_F(RefineBySpeedTest, MaintainsPathLength)
{
  // Create a curved path to test proper distance calculation
  auto object =
    createTestObject(0.05, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 1.0, 0.0}, {3.0, 2.0, 0.0}});

  Context context;
  ASSERT_TRUE(processor_->run(object, context));

  auto & processed_path = object.kinematics.predicted_paths[0].path;
  ASSERT_EQ(processed_path.size(), 4);

  // Verify that the path maintains reasonable progression
  // (refined waypoints should still progress forward, just at reduced distances)
  double prev_distance = 0.0;
  for (size_t i = 1; i < processed_path.size(); ++i) {
    double current_distance =
      calculateDistance(processed_path[0].position, processed_path[i].position);
    EXPECT_GE(current_distance, prev_distance);  // Should be monotonically increasing
    prev_distance = current_distance;
  }
}
}  // namespace autoware::predicted_path_postprocessor::testing
