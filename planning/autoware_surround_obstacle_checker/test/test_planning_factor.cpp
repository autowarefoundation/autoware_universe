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

#include "../src/node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::surround_obstacle_checker
{

using autoware::planning_test_manager::PlanningInterfaceTestManager;
using autoware_internal_planning_msgs::msg::SafetyFactor;

class SurroundObstacleCheckerPlanningFactorTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    test_node_ = std::make_shared<rclcpp::Node>("planning_interface_test_node");
    test_target_node_ = generateTestTargetNode();

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(test_node_);

    const std::string output_planning_factors_topic =
      "planning/planning_factors/surround_obstacle_checker";
    sub_planning_factor_ =
      test_node_->create_subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>(
        output_planning_factors_topic, rclcpp::QoS{1},
        [this](autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr msg) {
          planning_factor_msg_ = msg;
        });

    pub_odometry_ = test_node_->create_publisher<nav_msgs::msg::Odometry>(
      "/surround_obstacle_checker_node/input/odometry", 1);
    pub_kinematic_state_ =
      test_node_->create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 1);
    pub_dynamic_objects_ = test_node_->create_publisher<PredictedObjects>(
      "/surround_obstacle_checker_node/input/objects", 1);
    pub_pointcloud_ = test_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/surround_obstacle_checker_node/input/pointcloud", 1);
  }

  void setEnableCheck(const std::string & type, const bool enable)
  {
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(test_target_node_);
    while (!param_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          test_target_node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(test_target_node_->get_logger(), "service not available, waiting again...");
    }

    std::vector<rclcpp::Parameter> new_parameters;
    new_parameters.push_back(rclcpp::Parameter(type + ".enable_check", enable));
    param_client->set_parameters(new_parameters);
  }

  std::shared_ptr<SurroundObstacleCheckerNode> generateTestTargetNode()
  {
    auto node_options = rclcpp::NodeOptions{};
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");

    autoware::test_utils::updateNodeOptions(
      node_options,
      {autoware_test_utils_dir + "/config/test_common.param.yaml",
       autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
       autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
       ament_index_cpp::get_package_share_directory("autoware_surround_obstacle_checker") +
         "/config/surround_obstacle_checker.param.yaml"});

    return std::make_shared<SurroundObstacleCheckerNode>(node_options);
  }

  void publishStaticTransforms()
  {
    const auto odometry = autoware::test_utils::makeInitialPose();
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = test_node_->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = odometry.pose.pose.position.x;
    transform.transform.translation.y = odometry.pose.pose.position.y;
    transform.transform.translation.z = odometry.pose.pose.position.z;
    transform.transform.rotation = odometry.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void publishMandatoryTopics()
  {
    auto odometry = autoware::test_utils::makeInitialPose();
    odometry.header.stamp = test_target_node_->now();
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "base_link";

    pub_odometry_->publish(odometry);
    pub_kinematic_state_->publish(odometry);
  }

  void publishDynamicObject(const unique_identifier_msgs::msg::UUID & object_id)
  {
    autoware_perception_msgs::msg::PredictedObject object;
    object.object_id = object_id;
    object.existence_probability = 1.0f;
    object.classification.resize(1);
    object.classification[0].label =
      autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
    object.kinematics.initial_pose_with_covariance.pose.position.x = 3722.16015625 + 0.5,
    object.kinematics.initial_pose_with_covariance.pose.position.y = 73723.515625;
    object.kinematics.initial_pose_with_covariance.pose.position.z = 0.0;
    object.kinematics.initial_pose_with_covariance.pose.orientation =
      autoware_utils_geometry::create_quaternion_from_yaw(0.0);

    autoware_perception_msgs::msg::PredictedObjects dynamic_objects;
    dynamic_objects.header.stamp = test_target_node_->now();
    dynamic_objects.header.frame_id = "map";
    dynamic_objects.objects.push_back(object);

    pub_dynamic_objects_->publish(dynamic_objects);
  }

  void publishPointCloud()
  {
    sensor_msgs::msg::PointCloud2 pointcloud;
    pointcloud.header.stamp = test_target_node_->now();
    pointcloud.header.frame_id = "base_link";
    pointcloud.height = 1;
    pointcloud.width = 1;
    pointcloud.is_dense = true;
    pointcloud.is_bigendian = false;
    pointcloud.fields.resize(3);
    pointcloud.fields[0].name = "x";
    pointcloud.fields[0].offset = 0;
    pointcloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud.fields[0].count = 1;
    pointcloud.fields[1].name = "y";
    pointcloud.fields[1].offset = 4;
    pointcloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud.fields[1].count = 1;
    pointcloud.fields[2].name = "z";
    pointcloud.fields[2].offset = 8;
    pointcloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud.fields[2].count = 1;
    pointcloud.point_step = 16;  // 4 bytes for each field
    pointcloud.row_step = pointcloud.point_step * pointcloud.width;
    pointcloud.data.resize(pointcloud.row_step * pointcloud.height);

    const auto odometry = autoware::test_utils::makeInitialPose();
    const float expected_base_link_x =
      0.5f * std::cos(-tf2::getYaw(odometry.pose.pose.orientation));
    const float expected_base_link_y =
      0.5f * std::sin(-tf2::getYaw(odometry.pose.pose.orientation));

    std::array<float, 4> point = {
      expected_base_link_x, expected_base_link_y, 0.0f,
      1.0f};  // x, y, z, intensity in base_link frame
    std::memcpy(pointcloud.data.data(), point.data(), point.size() * sizeof(float));
    pub_pointcloud_->publish(pointcloud);
  }

  void validatePlanningFactor(
    const unique_identifier_msgs::msg::UUID & validate_object_id,
    const uint16_t expected_object_type)
  {
    // make sure planning_factor_msg_ is received
    EXPECT_NE(planning_factor_msg_, nullptr);

    // make sure planning_factor_msg_ is not empty
    EXPECT_EQ(planning_factor_msg_->factors.size(), 1);

    for (const auto & factor : planning_factor_msg_->factors) {
      EXPECT_FALSE(factor.safety_factors.is_safe);

      // make sure control_points is not empty
      EXPECT_GE(factor.control_points.size(), 1);
      for (auto & control_point : factor.control_points) {
        EXPECT_LT(control_point.distance, 200.0);
      }

      // make sure safety_factors is not empty
      EXPECT_EQ(factor.safety_factors.factors.size(), 1);

      const auto & safety_factor = factor.safety_factors.factors.at(0);

      EXPECT_EQ(safety_factor.type, expected_object_type);
      EXPECT_FALSE(safety_factor.is_safe);
      EXPECT_EQ(safety_factor.points.size(), 1);

      const double expected_x = 3722.16015625 + 0.5;
      const double expected_y = 73723.515625;

      Point2d validate_point_2d(expected_x, expected_y);
      Point2d safety_factor_point_2d(safety_factor.points.at(0).x, safety_factor.points.at(0).y);
      EXPECT_NEAR(bg::distance(validate_point_2d, safety_factor_point_2d), 0.0, 1e-6);

      EXPECT_EQ(safety_factor.object_id, validate_object_id);
    }
  }
  void TearDown() override { rclcpp::shutdown(); }

  void spinSome()
  {
    rclcpp::spin_some(test_target_node_);
    rclcpp::spin_some(test_node_);
  }

private:
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_kinematic_state_;
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr
    pub_dynamic_objects_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
  std::shared_ptr<SurroundObstacleCheckerNode> test_target_node_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>::SharedPtr
    sub_planning_factor_;
  autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr planning_factor_msg_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

TEST_F(SurroundObstacleCheckerPlanningFactorTest, TestByDynamicObject)
{
  const auto object_id = autoware_utils_uuid::generate_uuid();
  for (size_t i = 0; i < 5; i++) {
    publishMandatoryTopics();
    publishDynamicObject(object_id);
    spinSome();
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  validatePlanningFactor(object_id, SafetyFactor::OBJECT);
}

TEST_F(SurroundObstacleCheckerPlanningFactorTest, TestByPointCloud)
{
  setEnableCheck("pointcloud", true);
  const auto default_id = unique_identifier_msgs::msg::UUID{};
  for (size_t i = 0; i < 5; i++) {
    publishStaticTransforms();
    publishMandatoryTopics();
    publishPointCloud();
    spinSome();
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  validatePlanningFactor(default_id, SafetyFactor::POINTCLOUD);
}

}  // namespace autoware::surround_obstacle_checker
