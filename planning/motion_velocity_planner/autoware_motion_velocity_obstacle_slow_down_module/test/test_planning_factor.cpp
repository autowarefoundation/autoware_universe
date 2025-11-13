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

#include "obstacle_slow_down_module.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

class PlanningFactorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    auto node_options = rclcpp::NodeOptions{};
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    std::vector<std::string> yaml_files = {
      autoware_test_utils_dir + "/config/test_motion_velocity_planner.param.yaml",
      autoware_test_utils_dir + "/config/test_common.param.yaml",
      autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
      autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
      ament_index_cpp::get_package_share_directory(
        "autoware_motion_velocity_obstacle_slow_down_module") +
        "/config/obstacle_slow_down.param.yaml"};
    autoware::test_utils::updateNodeOptions(node_options, yaml_files);

    node_ = std::make_shared<rclcpp::Node>("test_node", node_options);
    module_ = std::make_unique<autoware::motion_velocity_planner::ObstacleSlowDownModule>();

    module_->init(*node_, "obstacle_slow_down");
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::unique_ptr<autoware::motion_velocity_planner::ObstacleSlowDownModule> module_;
  std::shared_ptr<rclcpp::Node> node_;
  nav_msgs::msg::Odometry odometry_;
  autoware_planning_msgs::msg::Trajectory trajectory_;

  void InitializeEgoData()
  {
    // Set odometry
    odometry_.header.frame_id = "map";
    odometry_.header.stamp = node_->get_clock()->now();
    odometry_.pose.pose.position.x = 0.0;
    odometry_.pose.pose.position.y = 0.0;
    odometry_.pose.pose.position.z = 0.0;
    odometry_.pose.pose.orientation.w = 1.0;

    // Generate trajectory points along a straight line
    trajectory_.header.frame_id = "map";
    trajectory_.header.stamp = node_->get_clock()->now();

    for (double x = 0.0; x <= 100.0; x += 1.0) {
      autoware_planning_msgs::msg::TrajectoryPoint point;
      point.pose.position.x = x;
      point.pose.position.y = 0.0;
      point.pose.position.z = 0.0;
      point.pose.orientation.w = 1.0;
      point.longitudinal_velocity_mps = 10.0;
      trajectory_.points.push_back(point);
    }
  }
};

TEST_F(PlanningFactorTest, TestWithPredictedObjects)
{
  InitializeEgoData();

  // Add a simple pedestrian obstacle for testing
  autoware_perception_msgs::msg::PredictedObject pedestrian;
  pedestrian.existence_probability = 1.0;

  // Set classification
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
  classification.probability = 1.0;
  pedestrian.classification.resize(1);
  pedestrian.classification.at(0) = classification;

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

  autoware_perception_msgs::msg::PredictedObjects predicted_objects;
  predicted_objects.header.frame_id = "map";
  predicted_objects.header.stamp = node_->get_clock()->now();
  predicted_objects.objects.push_back(pedestrian);

  auto planner_data = std::make_shared<autoware::motion_velocity_planner::PlannerData>(*node_);
  planner_data->current_odometry = odometry_;
  planner_data->process_predicted_objects(predicted_objects);

  const auto count_param_str =
    "obstacle_slow_down.obstacle_filtering.successive_num_to_entry_slow_down_condition";
  const size_t count = node_->get_parameter(count_param_str).as_int();
  for (size_t i = 0; i < count + 5; ++i) {
    module_->publish_planning_factor();
    module_->plan(trajectory_.points, trajectory_.points, planner_data);
  }

  auto planning_factors = module_->get_planning_factors();
  ASSERT_EQ(planning_factors.size(), 1);
  const auto & planning_factor = planning_factors.front();
  EXPECT_EQ(
    planning_factor.behavior, autoware_internal_planning_msgs::msg::PlanningFactor::SLOW_DOWN);
  EXPECT_NEAR(planning_factor.control_points.front().pose.position.x, 10.0, 5.0);
  EXPECT_NEAR(planning_factor.control_points.front().pose.position.y, 0.0, 1.0);
  EXPECT_NEAR(planning_factor.control_points.front().pose.position.z, 0.0, 1.0);

  ASSERT_EQ(planning_factor.safety_factors.factors.size(), 1);
  const auto & safety_factor = planning_factor.safety_factors.factors.front();
  EXPECT_EQ(safety_factor.type, autoware_internal_planning_msgs::msg::SafetyFactor::UNKNOWN);
  EXPECT_FALSE(safety_factor.is_safe);
  ASSERT_EQ(safety_factor.points.size(), 1);
  EXPECT_NEAR(safety_factor.points.front().x, initial_pose.position.x, 1.0);
  EXPECT_NEAR(safety_factor.points.front().y, initial_pose.position.y, 1.0);
  EXPECT_NEAR(safety_factor.points.front().z, initial_pose.position.z, 1.0);

  // TODO(takagi): split the following test into another TEST_F
  predicted_objects.objects.front().kinematics.initial_twist_with_covariance.twist.linear.y = -10.0;
  planner_data->process_predicted_objects(predicted_objects);
  for (size_t i = 0; i < count + 5; ++i) {
    module_->publish_planning_factor();
    module_->plan(trajectory_.points, trajectory_.points, planner_data);
  }
  planning_factors = module_->get_planning_factors();
  ASSERT_EQ(planning_factors.size(), 0);
}
