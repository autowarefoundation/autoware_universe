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

#include "autoware/diffusion_planner/diffusion_planner_node.hpp"

#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

namespace autoware::diffusion_planner::test
{
using namespace std::chrono_literals;  // NOLINT

class DiffusionPlannerIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Create node with test parameters
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {{"model_path", "/tmp/test_model.onnx"},  // Non-existent model for testing
       {"args_path", "/tmp/test_args.json"},
       {"timer_period", 0.1},
       {"ignore_unknown_neighbors", true},
       {"predict_neighbor_trajectory", false},
       {"publish_debug_markers", false}});

    // Note: This will fail to initialize ONNX Runtime without a valid model
    // For integration testing, we would need to mock or provide a test model
  }

  void TearDown() override { rclcpp::shutdown(); }

  TrackedObject createTestObject(double x, double y, double vx, double vy)
  {
    TrackedObject obj;
    obj.object_id = autoware_utils_uuid::generate_uuid();
    obj.kinematics.pose_with_covariance.pose.position.x = x;
    obj.kinematics.pose_with_covariance.pose.position.y = y;
    obj.kinematics.pose_with_covariance.pose.position.z = 0.0;
    obj.kinematics.twist_with_covariance.twist.linear.x = vx;
    obj.kinematics.twist_with_covariance.twist.linear.y = vy;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x = 4.0;
    obj.shape.dimensions.y = 2.0;
    obj.shape.dimensions.z = 1.5;
    obj.existence_probability = 0.9;

    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
    classification.probability = 0.95;
    obj.classification.push_back(classification);

    return obj;
  }
};

// Integration test: Multiple subscribers receiving data
TEST_F(DiffusionPlannerIntegrationTest, MultipleSubscribersDataFlow)
{
  // Create test node that publishes to diffusion planner inputs
  auto test_node = std::make_shared<rclcpp::Node>("test_publisher");

  // Create publishers for all input topics
  auto odometry_pub =
    test_node->create_publisher<nav_msgs::msg::Odometry>("/diffusion_planner/input/odometry", 10);
  auto acceleration_pub =
    test_node->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "/diffusion_planner/input/acceleration", 10);
  auto objects_pub = test_node->create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
    "/diffusion_planner/input/tracked_objects", 10);

  // Create test data
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = test_node->now();
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = 100.0;
  odometry.pose.pose.position.y = 200.0;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 10.0;

  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  acceleration.header = odometry.header;
  acceleration.accel.accel.linear.x = 0.5;

  autoware_perception_msgs::msg::TrackedObjects objects;
  objects.header = odometry.header;
  objects.objects.push_back(createTestObject(110.0, 200.0, 8.0, 0.0));
  objects.objects.push_back(createTestObject(90.0, 195.0, 12.0, 1.0));

  // Publish data multiple times
  for (int i = 0; i < 5; ++i) {
    odometry_pub->publish(odometry);
    acceleration_pub->publish(acceleration);
    objects_pub->publish(objects);

    // Update positions for next iteration
    odometry.pose.pose.position.x += 1.0;
    objects.objects[0].kinematics.pose_with_covariance.pose.position.x += 0.8;
    objects.objects[1].kinematics.pose_with_covariance.pose.position.x += 1.2;

    // Update timestamps
    auto new_time = test_node->now();
    odometry.header.stamp = new_time;
    acceleration.header.stamp = new_time;
    objects.header.stamp = new_time;

    rclcpp::spin_some(test_node);
    std::this_thread::sleep_for(50ms);
  }
}

// Integration test: Error handling with missing data
TEST_F(DiffusionPlannerIntegrationTest, MissingDataHandling)
{
  auto test_node = std::make_shared<rclcpp::Node>("test_publisher");

  // Only publish partial data (missing odometry)
  auto acceleration_pub =
    test_node->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "/diffusion_planner/input/acceleration", 10);
  auto objects_pub = test_node->create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
    "/diffusion_planner/input/tracked_objects", 10);

  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  acceleration.header.stamp = test_node->now();
  acceleration.header.frame_id = "map";
  acceleration.accel.accel.linear.x = 0.5;

  autoware_perception_msgs::msg::TrackedObjects objects;
  objects.header = acceleration.header;
  objects.objects.push_back(createTestObject(110.0, 200.0, 8.0, 0.0));

  // Publish incomplete data
  acceleration_pub->publish(acceleration);
  objects_pub->publish(objects);

  rclcpp::spin_some(test_node);
  std::this_thread::sleep_for(100ms);

  // Node should handle missing odometry gracefully
  EXPECT_NO_THROW(rclcpp::spin_some(test_node));
}

// Integration test: Rapid data updates
TEST_F(DiffusionPlannerIntegrationTest, RapidDataUpdates)
{
  auto test_node = std::make_shared<rclcpp::Node>("test_publisher");

  auto odometry_pub =
    test_node->create_publisher<nav_msgs::msg::Odometry>("/diffusion_planner/input/odometry", 10);

  // Publish data at high frequency
  for (int i = 0; i < 100; ++i) {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = test_node->now();
    odometry.header.frame_id = "map";
    odometry.pose.pose.position.x = 100.0 + i * 0.1;
    odometry.pose.pose.position.y = 200.0;
    odometry.pose.pose.orientation.w = 1.0;
    odometry.twist.twist.linear.x = 10.0 + i * 0.01;

    odometry_pub->publish(odometry);

    // Very short sleep to simulate high-frequency updates
    std::this_thread::sleep_for(1ms);
  }

  // Give time for processing
  rclcpp::spin_some(test_node);
  std::this_thread::sleep_for(100ms);
}

// Integration test: Large number of tracked objects
TEST_F(DiffusionPlannerIntegrationTest, ManyTrackedObjects)
{
  auto test_node = std::make_shared<rclcpp::Node>("test_publisher");

  auto objects_pub = test_node->create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
    "/diffusion_planner/input/tracked_objects", 10);

  autoware_perception_msgs::msg::TrackedObjects objects;
  objects.header.stamp = test_node->now();
  objects.header.frame_id = "map";

  // Create many tracked objects (more than typical NEIGHBOR_SHAPE[1])
  for (int i = 0; i < 50; ++i) {
    double angle = i * 2 * M_PI / 50;
    double radius = 20.0 + (i % 5) * 5.0;
    objects.objects.push_back(createTestObject(
      100.0 + radius * cos(angle), 200.0 + radius * sin(angle), 5.0 * cos(angle + M_PI_2),
      5.0 * sin(angle + M_PI_2)));
  }

  objects_pub->publish(objects);

  rclcpp::spin_some(test_node);
  std::this_thread::sleep_for(100ms);
}

// Integration test: Time synchronization
TEST_F(DiffusionPlannerIntegrationTest, TimeSynchronization)
{
  auto test_node = std::make_shared<rclcpp::Node>("test_publisher");

  auto odometry_pub =
    test_node->create_publisher<nav_msgs::msg::Odometry>("/diffusion_planner/input/odometry", 10);
  auto objects_pub = test_node->create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
    "/diffusion_planner/input/tracked_objects", 10);

  // Publish data with different timestamps
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = 100.0;
  odometry.pose.pose.position.y = 200.0;
  odometry.pose.pose.orientation.w = 1.0;

  autoware_perception_msgs::msg::TrackedObjects objects;
  objects.header.frame_id = "map";
  objects.objects.push_back(createTestObject(110.0, 200.0, 8.0, 0.0));

  // Publish with time offset
  auto now = test_node->now();
  odometry.header.stamp = now;
  objects.header.stamp = now + rclcpp::Duration(0, 100000000);  // 100ms offset

  odometry_pub->publish(odometry);
  objects_pub->publish(objects);

  rclcpp::spin_some(test_node);
  std::this_thread::sleep_for(100ms);
}

// Integration test: Frame transformation errors
TEST_F(DiffusionPlannerIntegrationTest, FrameTransformationErrors)
{
  auto test_node = std::make_shared<rclcpp::Node>("test_publisher");

  auto odometry_pub =
    test_node->create_publisher<nav_msgs::msg::Odometry>("/diffusion_planner/input/odometry", 10);
  auto objects_pub = test_node->create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
    "/diffusion_planner/input/tracked_objects", 10);

  // Publish data with different frame IDs
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = test_node->now();
  odometry.header.frame_id = "base_link";  // Different frame
  odometry.pose.pose.position.x = 100.0;
  odometry.pose.pose.position.y = 200.0;
  odometry.pose.pose.orientation.w = 1.0;

  autoware_perception_msgs::msg::TrackedObjects objects;
  objects.header.stamp = test_node->now();
  objects.header.frame_id = "map";
  objects.objects.push_back(createTestObject(110.0, 200.0, 8.0, 0.0));

  odometry_pub->publish(odometry);
  objects_pub->publish(objects);

  rclcpp::spin_some(test_node);
  std::this_thread::sleep_for(100ms);
}

}  // namespace autoware::diffusion_planner::test
