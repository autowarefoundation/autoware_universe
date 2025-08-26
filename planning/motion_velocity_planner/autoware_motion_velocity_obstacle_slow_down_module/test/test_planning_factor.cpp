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

#include <autoware/motion_velocity_planner/test_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace autoware::motion_velocity_planner
{

TEST(PlanningFactorTest, NodeTestWithPredictedObjects)
{
  rclcpp::init(0, nullptr);

  const auto plugin_info_vec = {autoware::motion_velocity_planner::PluginInfo{
    "obstacle_slow_down", "autoware::motion_velocity_planner::ObstacleSlowDownModule"}};

  auto test_manager = autoware::motion_velocity_planner::generateTestManager();
  auto test_target_node = autoware::motion_velocity_planner::generateNode(plugin_info_vec);
  autoware::motion_velocity_planner::publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_trajectory_topic = "motion_velocity_planner/input/trajectory";
  const std::string input_odometry_topic = "motion_velocity_planner/input/vehicle_odometry";
  const std::string input_dynamic_objects_topic = "motion_velocity_planner/input/dynamic_objects";
  const std::string output_planning_factors_topic = "planning/planning_factors/obstacle_slow_down";

  const rclcpp::Node::SharedPtr test_node = test_manager->getTestNode();
  RCLCPP_INFO(rclcpp::get_logger("test_node"), "test");

  autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr planning_factor_msg;
  const auto test_sub =
    test_node->create_subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>(
      output_planning_factors_topic, rclcpp::QoS{1},
      [&planning_factor_msg](
        autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr msg) {
        planning_factor_msg = msg;
        RCLCPP_INFO_ONCE(
          rclcpp::get_logger("test_node"), "Received PlanningFactorArray with %zu factors",
          msg->factors.size());

        RCLCPP_INFO_ONCE(rclcpp::get_logger("test_node"), "Planning factors:");
        if (msg->factors.empty()) {
          RCLCPP_WARN(rclcpp::get_logger("test_node"), "No planning factors received.");
        }
      });

  // Create test data using autoware_test_utils
  auto objects = autoware_perception_msgs::msg::PredictedObjects{};
  objects.header.frame_id = "map";
  objects.header.stamp = test_node->get_clock()->now();

  // Add a simple pedestrian obstacle for testing
  autoware_perception_msgs::msg::PredictedObject pedestrian;
  pedestrian.existence_probability = 1.0;

  // Set classification
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
  classification.probability = 1.0;
  pedestrian.classification.push_back(classification);

  // Set pose - place pedestrian in front of ego
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.position.x = 20.0;
  initial_pose.position.y = 1.6;
  initial_pose.position.z = 0.0;
  initial_pose.orientation.w = 1.0;
  pedestrian.kinematics.initial_pose_with_covariance.pose = initial_pose;

  geometry_msgs::msg::Twist initial_twist;
  initial_twist.linear.x = 0.0;
  initial_twist.linear.y = 0.0;
  pedestrian.kinematics.initial_twist_with_covariance.twist = initial_twist;

  // Set velocity
  pedestrian.kinematics.initial_twist_with_covariance.twist.linear.x = 0.0;
  pedestrian.kinematics.initial_twist_with_covariance.twist.linear.y = 0.0;

  // Set shape
  pedestrian.shape.type = autoware_perception_msgs::msg::Shape::CYLINDER;
  pedestrian.shape.dimensions.x = 0.5;
  pedestrian.shape.dimensions.y = 0.5;
  pedestrian.shape.dimensions.z = 1.8;

  pedestrian.kinematics.predicted_paths.resize(1);
  auto & predicted_path = pedestrian.kinematics.predicted_paths.front();
  predicted_path.path.resize(10);
  predicted_path.time_step = rclcpp::Duration::from_seconds(0.5);
  predicted_path.confidence = 1.0;
  for (size_t i = 0; i < predicted_path.path.size(); ++i) {
    predicted_path.path[i].position.x = initial_pose.position.x + i * 0.5 * initial_twist.linear.x;
    predicted_path.path[i].position.y = initial_pose.position.y + i * 0.5 * initial_twist.linear.y;
    predicted_path.path[i].position.z = initial_pose.position.z;
    predicted_path.path[i].orientation = initial_pose.orientation;
  }

  objects.objects.push_back(pedestrian);

  auto odometry = nav_msgs::msg::Odometry{};
  odometry.header.frame_id = "map";
  odometry.header.stamp = test_node->get_clock()->now();
  odometry.pose.pose.position.x = 0.0;
  odometry.pose.pose.position.y = 0.0;
  odometry.pose.pose.position.z = 0.0;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 0.0;
  odometry.twist.twist.linear.y = 0.0;

  // Generate trajectory points along a straight line
  auto trajectory = autoware_planning_msgs::msg::Trajectory{};
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = test_node->get_clock()->now();

  for (double x = 0.0; x <= 100.0; x += 1.0) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = 0.0;
    point.pose.position.z = 0.0;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 10.0;
    trajectory.points.push_back(point);
  }

  test_manager->publishInput(test_target_node, input_dynamic_objects_topic, objects, 1);
  test_manager->publishInput(test_target_node, input_odometry_topic, odometry, 1);
  test_manager->publishInput(test_target_node, input_trajectory_topic, trajectory, 1);

  rclcpp::spin_some(test_target_node);
  rclcpp::spin_some(test_manager->getTestNode());

  for (size_t i = 0; i < 10; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    objects.header.stamp = test_node->get_clock()->now();
    odometry.header.stamp = test_node->get_clock()->now();
    trajectory.header.stamp = test_node->get_clock()->now();
    test_manager->publishInput(test_target_node, input_dynamic_objects_topic, objects, 1);
    test_manager->publishInput(test_target_node, input_odometry_topic, odometry, 1);
    test_manager->publishInput(test_target_node, input_trajectory_topic, trajectory, 1);

    rclcpp::spin_some(test_target_node);
    rclcpp::spin_some(test_manager->getTestNode());
    // make sure motion_velocity_planner is running
    EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

    // make sure planning_factor_msg is received
    EXPECT_NE(planning_factor_msg, nullptr);
  }

  // make sure planning_factor_msg is not empty
  EXPECT_EQ(planning_factor_msg->factors.size(), 1);

  const auto & planning_factor = planning_factor_msg->factors.front();
  EXPECT_EQ(
    planning_factor.behavior, autoware_internal_planning_msgs::msg::PlanningFactor::SLOW_DOWN);
  EXPECT_NEAR(planning_factor.control_points.front().pose.position.x, 10.0, 5.0);
  EXPECT_NEAR(planning_factor.control_points.front().pose.position.y, 0.0, 1.0);
  EXPECT_NEAR(planning_factor.control_points.front().pose.position.z, 0.0, 1.0);
  EXPECT_EQ(planning_factor.safety_factors.factors.size(), 1);

  const auto & safety_factor = planning_factor.safety_factors.factors.front();
  // current implementation does not set object type
  // EXPECT_EQ(safety_factor.type, expected_object_type);
  EXPECT_FALSE(safety_factor.is_safe);
  EXPECT_EQ(safety_factor.points.size(), 1);
  EXPECT_NEAR(safety_factor.points.front().x, initial_pose.position.x, 1.0);
  EXPECT_NEAR(safety_factor.points.front().y, initial_pose.position.y, 1.0);
  EXPECT_NEAR(safety_factor.points.front().z, initial_pose.position.z, 1.0);
  EXPECT_EQ(safety_factor.object_id, objects.objects.front().object_id);

  rclcpp::shutdown();
}
}  // namespace autoware::motion_velocity_planner
