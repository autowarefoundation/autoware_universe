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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/obstacle_stop.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

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
using autoware::trajectory_modifier::plugin::ObstacleStop;
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

TrajectoryPoints create_straight_trajectory(double length, double velocity, double spacing = 1.0)
{
  TrajectoryPoints trajectory;
  for (double x = 0.0; x <= length + 1e-6; x += spacing) {
    trajectory.push_back(create_trajectory_point(x, 0.0, velocity));
  }
  return trajectory;
}

Odometry::ConstSharedPtr make_odometry(double x, double y, double velocity)
{
  Odometry odometry;
  odometry.header.frame_id = "map";
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

PredictedObjects::ConstSharedPtr make_blocking_car(double x, double y)
{
  constexpr double car_size_x = 2.0;
  constexpr double car_size_y = 2.0;
  constexpr double car_size_z = 1.5;

  PredictedObjects predicted_objects;
  predicted_objects.header.frame_id = "map";
  predicted_objects.objects.push_back(
    create_box_object(x, y, car_size_x, car_size_y, car_size_z, ObjectClassification::CAR));
  return std::make_shared<const PredictedObjects>(predicted_objects);
}

// Build a dense pointcloud cluster the obstacle_stop pipeline will detect.
sensor_msgs::msg::PointCloud2::ConstSharedPtr make_blocking_pointcloud_cluster(
  double center_x, double center_y, double height)
{
  constexpr int voxel_count_x = 4;
  constexpr int voxel_count_y = 3;
  constexpr int points_per_voxel = 3;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(voxel_count_x * voxel_count_y * points_per_voxel);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (int xi = 0; xi < voxel_count_x; ++xi) {
    for (int yi = 0; yi < voxel_count_y; ++yi) {
      for (int p = 0; p < points_per_voxel; ++p) {
        *iter_x = static_cast<float>(center_x + 0.2 * xi + 0.05 + 0.01 * p);
        *iter_y = static_cast<float>(center_y + 0.2 * yi + 0.05);
        *iter_z = static_cast<float>(height);
        ++iter_x;
        ++iter_y;
        ++iter_z;
      }
    }
  }

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

}  // namespace

class ObstacleStopIntegrationTest : public ::testing::Test
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

    node_ = std::make_shared<rclcpp::Node>("test_obstacle_stop_node", node_options);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();

    set_up_default_params();

    // Create the context and the plugin once. Tests build per-frame InputData inline,
    // and inject any required TF directly into context_->tf_buffer.
    context_ = std::make_shared<TrajectoryModifierContext>(node_.get());
    plugin_ = std::make_unique<ObstacleStop>();
    plugin_->initialize("test_obstacle_stop", node_.get(), time_keeper_, context_, params_);
  }

  void TearDown() override
  {
    plugin_.reset();
    context_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void set_up_default_params()
  {
    params_.use_obstacle_stop = true;
    params_.use_stop_point_fixer = false;
    params_.trajectory_time_step = 0.1;

    auto & p = params_.obstacle_stop;
    p.use_objects = true;
    p.use_pointcloud = true;
    p.enable_stop_for_objects = true;
    p.enable_stop_for_pointcloud = true;
    p.stop_margin = 6.0;
    p.nominal_stopping_decel = 1.0;
    p.maximum_stopping_decel = 4.0;
    p.stopping_jerk = 3.0;
    p.lateral_margin = 0.5;
    p.arrived_distance_threshold = 0.5;

    p.obstacle_tracking.on_time_buffer = 0.01;
    p.obstacle_tracking.off_time_buffer = 1.0;
    p.obstacle_tracking.object_distance_th = 1.0;
    p.obstacle_tracking.object_yaw_th = 0.1745;
    p.obstacle_tracking.pcd_distance_th = 0.5;
    p.obstacle_tracking.grace_period = 0.5;

    p.objects.object_types = {"car"};
    p.objects.max_velocity_th = 1.0;

    p.pointcloud.height_buffer = 0.5;
    p.pointcloud.min_height = 0.2;
    p.pointcloud.voxel_grid_filter.x = 0.2;
    p.pointcloud.voxel_grid_filter.y = 0.2;
    p.pointcloud.voxel_grid_filter.z = 0.2;
    p.pointcloud.voxel_grid_filter.min_size = 3;
    p.pointcloud.clustering.tolerance = 0.3;
    p.pointcloud.clustering.min_height = 0.5;
    p.pointcloud.clustering.min_size = 10;
    p.pointcloud.clustering.max_size = 10000;

    p.rss_params.enable = true;
    p.rss_params.object_decel.car = 1.5;
    p.rss_params.reaction_time = 0.2;
    p.rss_params.safety_margin = 2.0;
    p.rss_params.min_vel_th = 0.5;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::unique_ptr<ObstacleStop> plugin_;
  trajectory_modifier_params::Params params_;
  std::shared_ptr<TrajectoryModifierContext> context_;
};

TEST_F(ObstacleStopIntegrationTest, TrajectoryNotModifiedWhenDisabled)
{
  // Arrange
  params_.use_obstacle_stop = false;
  plugin_->update_params(params_);
  TrajectoryPoints trajectory;

  // Act
  const bool modified = plugin_->modify_trajectory(trajectory, InputData{});

  // Assert
  EXPECT_FALSE(modified);
}

TEST_F(ObstacleStopIntegrationTest, TrajectoryNotModifiedForEmptyTrajectory)
{
  // Arrange
  TrajectoryPoints empty_trajectory;

  // Act
  const bool modified = plugin_->modify_trajectory(empty_trajectory, InputData{});

  // Assert
  EXPECT_FALSE(modified);
}

TEST_F(ObstacleStopIntegrationTest, TrajectoryNotModifiedWhenNoObstaclesDetected)
{
  // Arrange: no predicted objects and no obstacle pointcloud.
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto input = create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0));

  // Act
  const bool modified = plugin_->modify_trajectory(trajectory, input);

  // Assert
  EXPECT_FALSE(modified);
}

TEST_F(ObstacleStopIntegrationTest, TrajectoryNotModifiedWhenObjectIsBesidePath)
{
  // Arrange: object 10 m off the trajectory polygon (lateral margin 0.5 + half-width ~0.95).
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto car_beside_path = make_blocking_car(20.0, 10.0);
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), car_beside_path);

  // Act
  plugin_->modify_trajectory(trajectory, input);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const bool modified = plugin_->modify_trajectory(trajectory, input);

  // Assert
  EXPECT_FALSE(modified);
}

TEST_F(ObstacleStopIntegrationTest, TrajectoryModifiedWhenObjectBlocksPath)
{
  // Arrange
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto car_blocking_path = make_blocking_car(20.0, 0.0);
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), car_blocking_path);

  // Act: obstacle tracker requires `on_time_buffer` of continuous observation
  //      before becoming active
  plugin_->modify_trajectory(trajectory, input);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const bool modified = plugin_->modify_trajectory(trajectory, input);

  // Assert
  EXPECT_TRUE(modified);
}

TEST_F(ObstacleStopIntegrationTest, StopPointInsertedBeforeObject)
{
  // Arrange
  constexpr double object_x = 20.0;
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto car_blocking_path = make_blocking_car(object_x, 0.0);
  const auto input =
    create_input_data(make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), car_blocking_path);

  // Act: obstacle tracker requires `on_time_buffer` of continuous observation
  //      before becoming active
  plugin_->modify_trajectory(trajectory, input);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const bool modified = plugin_->modify_trajectory(trajectory, input);

  // Assert
  ASSERT_TRUE(modified);
  EXPECT_NEAR(trajectory.back().longitudinal_velocity_mps, 0.0F, 0.1F);
  EXPECT_LT(trajectory.back().pose.position.x, object_x);
}

TEST_F(ObstacleStopIntegrationTest, StopPointInsertedForBlockingPointcloudCluster)
{
  // Arrange
  constexpr double cluster_center_x = 15.0;
  auto trajectory = create_straight_trajectory(30.0, 8.0);
  const auto pointcloud_blocking_path =
    make_blocking_pointcloud_cluster(cluster_center_x, 0.0, 0.7);
  const auto input = create_input_data(
    make_odometry(0.0, 0.0, 8.0), make_acceleration(0.0), nullptr, pointcloud_blocking_path);

  // Act: obstacle tracker requires `on_time_buffer` of continuous observation
  //      before becoming active
  plugin_->modify_trajectory(trajectory, input);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const bool modified = plugin_->modify_trajectory(trajectory, input);

  // Assert
  ASSERT_TRUE(modified);
  EXPECT_NEAR(trajectory.back().longitudinal_velocity_mps, 0.0F, 0.1F);
  EXPECT_LT(trajectory.back().pose.position.x, cluster_center_x);
}
