// Copyright 2026 TIER IV, Inc.
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

#include "autoware/trajectory_processor/trajectory_modifier_plugins/surround_obstacle_stop.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_trajectory_processor/trajectory_modifier_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
using autoware::trajectory_modifier::TrajectoryModifierContext;
using autoware::trajectory_modifier::plugin::InputData;
using autoware::trajectory_modifier::plugin::SurroundObstacleStop;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

TrajectoryPoint create_trajectory_point(double x, double y, double velocity)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = 0.0;
  point.pose.orientation.x = 0.0;
  point.pose.orientation.y = 0.0;
  point.pose.orientation.z = 0.0;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = static_cast<float>(velocity);
  point.acceleration_mps2 = 0.0F;
  return point;
}

TrajectoryPoints create_straight_trajectory(
  double length, double init_velocity, double spacing = 1.0)
{
  constexpr double max_velocity = 5.0;
  TrajectoryPoints trajectory;
  auto vel = init_velocity;
  for (double x = 0.0; x <= length + 1e-6; x += spacing) {
    trajectory.push_back(create_trajectory_point(x, 0.0, vel));
    vel = std::min(vel + 0.5, max_velocity);
  }
  return trajectory;
}

Odometry::ConstSharedPtr make_odometry(
  double x, double y, double velocity, const rclcpp::Time & stamp)
{
  Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.header.stamp = stamp;
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.position.y = y;
  odometry.pose.pose.position.z = 0.0;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = velocity;
  return std::make_shared<const Odometry>(odometry);
}

AccelWithCovarianceStamped::ConstSharedPtr make_acceleration(double accel_x)
{
  AccelWithCovarianceStamped acceleration;
  acceleration.accel.accel.linear.x = accel_x;
  return std::make_shared<const AccelWithCovarianceStamped>(acceleration);
}

PredictedObject create_box_object(
  double x, double y, double size_x, double size_y, double size_z, uint8_t classification_label)
{
  PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose.position.x = x;
  object.kinematics.initial_pose_with_covariance.pose.position.y = y;
  object.kinematics.initial_pose_with_covariance.pose.position.z = 0.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = 0.0;

  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = size_x;
  object.shape.dimensions.y = size_y;
  object.shape.dimensions.z = size_z;

  ObjectClassification classification;
  classification.label = classification_label;
  classification.probability = 1.0;
  object.classification.push_back(classification);

  return object;
}

PredictedObjects::ConstSharedPtr make_predicted_objects(
  double x, double y, uint8_t classification_label = ObjectClassification::CAR)
{
  constexpr double size_x = 2.0;
  constexpr double size_y = 2.0;
  constexpr double size_z = 1.5;

  PredictedObjects predicted_objects;
  predicted_objects.header.frame_id = "map";
  predicted_objects.objects.push_back(
    create_box_object(x, y, size_x, size_y, size_z, classification_label));
  return std::make_shared<const PredictedObjects>(predicted_objects);
}

sensor_msgs::msg::PointCloud2::ConstSharedPtr make_pointcloud_in_base_link(
  double x, double y, double z)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  *iter_x = static_cast<float>(x);
  *iter_y = static_cast<float>(y);
  *iter_z = static_cast<float>(z);

  return std::make_shared<const sensor_msgs::msg::PointCloud2>(cloud);
}

InputData create_input_data(
  Odometry::ConstSharedPtr current_odometry,
  AccelWithCovarianceStamped::ConstSharedPtr current_acceleration,
  PredictedObjects::ConstSharedPtr predicted_objects = nullptr,
  sensor_msgs::msg::PointCloud2::ConstSharedPtr obstacle_pointcloud = nullptr)
{
  InputData input;
  input.current_odometry = std::move(current_odometry);
  input.current_acceleration = std::move(current_acceleration);
  input.predicted_objects = std::move(predicted_objects);
  input.obstacle_pointcloud = std::move(obstacle_pointcloud);
  return input;
}

void expect_stop_trajectory_at_ego(const TrajectoryPoints & trajectory)
{
  ASSERT_EQ(trajectory.size(), 3U);

  EXPECT_FLOAT_EQ(trajectory.front().pose.position.x, 0.0);
  EXPECT_FLOAT_EQ(trajectory.front().pose.position.y, 0.0);
  EXPECT_FLOAT_EQ(trajectory.front().longitudinal_velocity_mps, 0.0F);
  EXPECT_FLOAT_EQ(trajectory.front().acceleration_mps2, 0.0F);
  EXPECT_NEAR(trajectory.back().pose.position.x, 0.0, 1e-2);
  EXPECT_NEAR(trajectory.back().pose.position.y, 0.0, 1e-2);
  EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
  EXPECT_FLOAT_EQ(trajectory.back().acceleration_mps2, 0.0F);
}

}  // namespace

class SurroundObstacleStopIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    auto node_options = rclcpp::NodeOptions{};
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    autoware::test_utils::updateNodeOptions(
      node_options, {autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<autoware::agnocast_wrapper::Node>(
      "test_surround_obstacle_stop_node", node_options);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();

    set_up_default_params();

    context_ = std::make_shared<TrajectoryModifierContext>(node_.get());
    plugin_ = std::make_unique<SurroundObstacleStop>();
    plugin_->initialize(
      "test_surround_obstacle_stop", node_.get(), time_keeper_, context_, params_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    plugin_.reset();
    context_.reset();
    node_.reset();
  }

  void set_up_default_params()
  {
    params_.use_stop_point_fixer = false;
    params_.use_obstacle_stop = false;
    params_.use_velocity_modifier = false;
    params_.use_surround_obstacle_stop = true;
    params_.trajectory_time_step = 0.1;

    auto & p = params_.surround_obstacle_stop;
    p.use_objects = true;
    p.use_pointcloud = true;
    p.object_types = {"car"};
    p.hysteresis_distance = 0.5;
    p.hysteresis_time = 0.0;
    p.ego_stopped_vel_th = 0.1;
    p.side_distance_th.car = 1.0;
    p.side_distance_th.pointcloud = 1.0;
  }

  InputData make_stopped_input(
    PredictedObjects::ConstSharedPtr predicted_objects = nullptr,
    sensor_msgs::msg::PointCloud2::ConstSharedPtr obstacle_pointcloud = nullptr)
  {
    const auto current_time = node_->now();
    return create_input_data(
      make_odometry(0.0, 0.0, 0.0, current_time), make_acceleration(0.0),
      std::move(predicted_objects), std::move(obstacle_pointcloud));
  }

  std::shared_ptr<autoware::agnocast_wrapper::Node> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::unique_ptr<SurroundObstacleStop> plugin_;
  trajectory_modifier_params::Params params_;
  std::shared_ptr<TrajectoryModifierContext> context_;
};

TEST_F(SurroundObstacleStopIntegrationTest, TrajectoryNotModifiedWhenDisabled)
{
  params_.use_surround_obstacle_stop = false;
  plugin_->update_params(params_);
  auto trajectory = create_straight_trajectory(10.0, 0.0);
  const auto input = make_stopped_input(make_predicted_objects(0.0, 4.0));

  const bool modified = plugin_->modify_trajectory(trajectory, input);

  EXPECT_FALSE(modified);
}

TEST_F(SurroundObstacleStopIntegrationTest, TrajectoryNotModifiedWhenEgoMoving)
{
  auto trajectory = create_straight_trajectory(10.0, 5.0);
  const auto objects = make_predicted_objects(0.0, 2.5);
  const auto current_time = node_->now();
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 5.0, current_time), make_acceleration(0.0), objects);

  const bool modified = plugin_->modify_trajectory(trajectory, input);

  EXPECT_FALSE(modified);
}

TEST_F(SurroundObstacleStopIntegrationTest, TrajectoryNotModifiedWhenNoObstaclesDetected)
{
  auto trajectory = create_straight_trajectory(10.0, 0.0);
  const auto input = make_stopped_input();

  const bool modified = plugin_->modify_trajectory(trajectory, input);

  EXPECT_FALSE(modified);
}

TEST_F(SurroundObstacleStopIntegrationTest, TrajectoryNotModifiedWhenObstacleIsFarAway)
{
  auto trajectory = create_straight_trajectory(10.0, 0.0);
  auto objects = make_predicted_objects(0.0, 10.0);
  const auto input = make_stopped_input(objects);

  const bool modified = plugin_->modify_trajectory(trajectory, input);

  EXPECT_FALSE(modified);
}

TEST_F(SurroundObstacleStopIntegrationTest, TrajectoryModifiedWhenNearObjectDetected)
{
  auto trajectory = create_straight_trajectory(10.0, 0.0);
  auto objects = make_predicted_objects(0.0, 2.5);
  const auto input = make_stopped_input(objects);

  const bool modified = plugin_->modify_trajectory(trajectory, input);

  ASSERT_TRUE(modified);
  expect_stop_trajectory_at_ego(trajectory);
}

TEST_F(SurroundObstacleStopIntegrationTest, TrajectoryModifiedWhenNearObjectDetectedWithHysteresis)
{
  auto trajectory = create_straight_trajectory(10.0, 0.0);
  auto objects = make_predicted_objects(0.0, 2.5);
  auto input = make_stopped_input(objects);
  const bool modified = plugin_->modify_trajectory(trajectory, input);
  ASSERT_TRUE(modified);
  expect_stop_trajectory_at_ego(trajectory);

  trajectory = create_straight_trajectory(10.0, 0.0);
  objects = make_predicted_objects(0.0, 3.0);
  input = make_stopped_input(objects);
  const bool modified_with_hysteresis = plugin_->modify_trajectory(trajectory, input);
  ASSERT_TRUE(modified_with_hysteresis);
  expect_stop_trajectory_at_ego(trajectory);
}

TEST_F(SurroundObstacleStopIntegrationTest, TrajectoryModifiedWhenNearbyPointcloudDetected)
{
  auto trajectory = create_straight_trajectory(10.0, 5.0);
  const auto pointcloud = make_pointcloud_in_base_link(0.0, 1.5, 0.5);
  const auto input = make_stopped_input(nullptr, pointcloud);

  const bool modified = plugin_->modify_trajectory(trajectory, input);

  ASSERT_TRUE(modified);
  expect_stop_trajectory_at_ego(trajectory);
}

TEST_F(SurroundObstacleStopIntegrationTest, StopStatePersistsDuringHysteresisTime)
{
  params_.surround_obstacle_stop.hysteresis_time = 0.2;
  plugin_->update_params(params_);
  auto trajectory = create_straight_trajectory(10.0, 5.0);
  auto objects = make_predicted_objects(0.0, 2.5);
  const auto nearby_input = make_stopped_input(objects);
  ASSERT_TRUE(plugin_->modify_trajectory(trajectory, nearby_input));
  expect_stop_trajectory_at_ego(trajectory);

  trajectory = create_straight_trajectory(10.0, 5.0);
  objects = make_predicted_objects(0.0, 10.0);
  const auto far_input = make_stopped_input(objects);
  const bool modified_within_hysteresis = plugin_->modify_trajectory(trajectory, far_input);

  ASSERT_TRUE(modified_within_hysteresis);
  expect_stop_trajectory_at_ego(trajectory);

  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  trajectory = create_straight_trajectory(10.0, 5.0);
  const bool modified_after_hysteresis = plugin_->modify_trajectory(trajectory, far_input);

  EXPECT_FALSE(modified_after_hysteresis);
}
