// Copyright 2024 TIER IV
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

#include "test.hpp"

#include "autoware/autonomous_emergency_braking/node.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/utils.hpp>

#include <autoware_perception_msgs/msg/detail/shape__struct.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>

#include <gtest/gtest.h>
#include <pcl/memory.h>

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

namespace autoware::motion::control::autonomous_emergency_braking::test
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_utils::Polygon2d;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::Header;

Header get_header(const char * const frame_id, rclcpp::Time t)
{
  std_msgs::msg::Header header;
  header.stamp = t;
  header.frame_id = frame_id;
  return header;
};

Odometry make_odometry_message(const Header & header, const double angular_velocity_z)
{
  Odometry odom_msg;
  odom_msg.header = header;
  odom_msg.child_frame_id = "base_link";
  odom_msg.twist.twist.angular.z = angular_velocity_z;
  return odom_msg;
};

VelocityReport make_velocity_report_msg(
  const Header & header, const double lat_velocity, const double long_velocity,
  const double heading_rate)
{
  VelocityReport velocity_msg;
  velocity_msg.header = header;
  velocity_msg.lateral_velocity = lat_velocity;
  velocity_msg.longitudinal_velocity = long_velocity;
  velocity_msg.heading_rate = heading_rate;
  return velocity_msg;
}

std::shared_ptr<AEB> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};

  const auto aeb_dir =
    ament_index_cpp::get_package_share_directory("autoware_autonomous_emergency_braking");
  const auto vehicle_info_util_dir =
    ament_index_cpp::get_package_share_directory("autoware_vehicle_info_utils");

  node_options.arguments(
    {"--ros-args", "--params-file", aeb_dir + "/config/autonomous_emergency_braking.param.yaml",
     "--ros-args", "--params-file", vehicle_info_util_dir + "/config/vehicle_info.param.yaml"});
  return std::make_shared<AEB>(node_options);
};

std::shared_ptr<PubSubNode> generatePubSubNode()
{
  auto node_options = rclcpp::NodeOptions{};
  node_options.arguments({"--ros-args"});
  return std::make_shared<PubSubNode>(node_options);
};

PubSubNode::PubSubNode(const rclcpp::NodeOptions & node_options)
: Node("test_aeb_pubsub", node_options)
{
  rclcpp::QoS qos{1};
  qos.transient_local();

  // Publish on the AEB node's fully-qualified input topics (its node name is "AEB") so that the
  // node's polling subscribers actually receive these messages; a relative "~/input/..." name would
  // resolve into this publisher node's own namespace and never reach the AEB node.
  pub_kinematic_state_ = create_publisher<Odometry>("/AEB/input/kinematic_state", qos);
  pub_point_cloud_ = create_publisher<PointCloud2>("/AEB/input/pointcloud", qos);
  pub_velocity_ = create_publisher<VelocityReport>("/AEB/input/velocity", qos);
  pub_predicted_traj_ = create_publisher<Trajectory>("/AEB/input/predicted_trajectory", qos);
  pub_predicted_objects_ = create_publisher<PredictedObjects>("/AEB/input/objects", qos);
  pub_autoware_state_ = create_publisher<AutowareState>("/autoware/state", qos);
}

TEST_F(TestAEB, checkCollision)
{
  constexpr double longitudinal_velocity = 3.0;
  ObjectData object_collision;
  object_collision.distance_to_object = 0.5;
  object_collision.velocity = 0.1;
  object_collision.position.x = 1.0;
  object_collision.position.y = 1.0;
  ASSERT_TRUE(aeb_node_->hasCollision(longitudinal_velocity, object_collision));

  ObjectData object_no_collision;
  object_no_collision.distance_to_object = 10.0;
  object_no_collision.velocity = 0.1;
  ASSERT_FALSE(aeb_node_->hasCollision(longitudinal_velocity, object_no_collision));
}

TEST_F(TestAEB, getObjectOnPathData)
{
  constexpr double longitudinal_velocity = 3.0;
  constexpr double yaw_rate = 0.0;
  const auto imu_path = aeb_node_->generateEgoPath(longitudinal_velocity, yaw_rate);
  ASSERT_FALSE(imu_path.empty());

  const double path_width = 2.0;
  const auto path_length = autoware::motion_utils::calcArcLength(imu_path);
  ASSERT_TRUE(
    path_length < aeb_node_->max_generated_imu_path_length_ +
                    aeb_node_->imu_prediction_time_interval_ * longitudinal_velocity +
                    std::numeric_limits<double>::epsilon());

  const auto stamp = rclcpp::Time();

  const auto longitudinal_offset_opt = utils::getLongitudinalOffset(imu_path, 1.0, -1.0);
  ASSERT_TRUE(longitudinal_offset_opt.has_value());
  const auto longitudinal_offset = longitudinal_offset_opt.value();
  ASSERT_DOUBLE_EQ(longitudinal_offset, 1.0);

  // Object in path if longitudinal_offset is considered
  Point obj_position;
  double path_expansion;

  {
    obj_position.x = path_length + std::numeric_limits<double>::epsilon();
    obj_position.y = 1.0;
    path_expansion = 0.0;
    auto obj_data_opt = utils::getObjectOnPathData(
      imu_path, obj_position, stamp, path_length, path_width, path_expansion, longitudinal_offset,
      0.0);
    ASSERT_TRUE(obj_data_opt.has_value());
    ASSERT_TRUE(obj_data_opt.value().is_target);
  }

  // Object outside of path
  {
    obj_position.x = path_length + std::numeric_limits<double>::epsilon();
    obj_position.y = 3.0;
    path_expansion = 0.0;
    auto obj_data_opt = utils::getObjectOnPathData(
      imu_path, obj_position, stamp, path_length, path_width, path_expansion, longitudinal_offset,
      0.0);
    ASSERT_FALSE(obj_data_opt.has_value());
  }

  // Object outside of path
  {
    obj_position.x = -1.0;
    obj_position.y = 0.0;
    path_expansion = 0.0;
    auto obj_data_opt = utils::getObjectOnPathData(
      imu_path, obj_position, stamp, path_length, path_width, path_expansion, longitudinal_offset,
      0.0);
    ASSERT_FALSE(obj_data_opt.has_value());
  }

  // Object is covered by path expansion
  {
    obj_position.x = path_length + std::numeric_limits<double>::epsilon();
    obj_position.y = 3.0;
    path_expansion = 2.0;
    auto obj_data_opt = utils::getObjectOnPathData(
      imu_path, obj_position, stamp, path_length, path_width, 2.0, longitudinal_offset, 0.0);
    ASSERT_TRUE(obj_data_opt.has_value());
    ASSERT_FALSE(obj_data_opt.value().is_target);
  }
}

TEST_F(TestAEB, checkImuPathGeneration)
{
  constexpr double longitudinal_velocity = 3.0;
  constexpr double yaw_rate = 0.05;
  const auto imu_path = aeb_node_->generateEgoPath(longitudinal_velocity, yaw_rate);
  ASSERT_FALSE(imu_path.empty());

  const double dt = aeb_node_->imu_prediction_time_interval_;
  const double horizon = aeb_node_->imu_prediction_time_horizon_;
  ASSERT_TRUE(imu_path.size() >= static_cast<size_t>(horizon / dt));

  const auto footprint = aeb_node_->generatePathFootprint(imu_path, 0.0);
  ASSERT_FALSE(footprint.empty());
  ASSERT_TRUE(footprint.size() == imu_path.size() - 1);

  const auto stamp = rclcpp::Time();
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_points_ptr =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  {
    const double x_start{0.5};
    const double y_start{0.0};

    for (size_t i = 0; i < 15; ++i) {
      pcl::PointXYZ p1(
        x_start + static_cast<double>(i / 100.0), y_start - static_cast<double>(i / 100.0), 0.5);
      pcl::PointXYZ p2(
        x_start + static_cast<double>((i + 10) / 100.0), y_start - static_cast<double>(i / 100.0),
        0.5);
      obstacle_points_ptr->push_back(p1);
      obstacle_points_ptr->push_back(p2);
    }
  }
  PointCloud::Ptr points_belonging_to_cluster_hulls = pcl::make_shared<PointCloud>();
  MarkerArray debug_markers;
  aeb_node_->getPointsBelongingToClusterHulls(
    obstacle_points_ptr, points_belonging_to_cluster_hulls, debug_markers);
  ASSERT_FALSE(points_belonging_to_cluster_hulls->empty());
  std::vector<ObjectData> objects;
  aeb_node_->getClosestObjectsOnPath(imu_path, stamp, points_belonging_to_cluster_hulls, objects);
  ASSERT_FALSE(objects.empty());
}

TEST_F(TestAEB, checkIncompleteImuPathGeneration)
{
  const double dt = aeb_node_->imu_prediction_time_interval_;
  const double horizon = aeb_node_->imu_prediction_time_horizon_;
  const double min_generated_path_length = aeb_node_->min_generated_imu_path_length_;
  const double slow_velocity = min_generated_path_length / (2.0 * horizon);
  constexpr double yaw_rate = 0.05;
  const auto imu_path = aeb_node_->generateEgoPath(slow_velocity, yaw_rate);

  ASSERT_FALSE(imu_path.empty());
  ASSERT_TRUE(imu_path.size() >= static_cast<size_t>(horizon / dt));
  ASSERT_TRUE(autoware::motion_utils::calcArcLength(imu_path) >= min_generated_path_length);

  const auto footprint = aeb_node_->generatePathFootprint(imu_path, 0.0);
  ASSERT_FALSE(footprint.empty());
  ASSERT_TRUE(footprint.size() == imu_path.size() - 1);
}

TEST_F(TestAEB, checkImuPathGenerationIsCut)
{
  const double dt = aeb_node_->imu_prediction_time_interval_;
  const double horizon = aeb_node_->imu_prediction_time_horizon_;
  const double max_generated_path_length = aeb_node_->max_generated_imu_path_length_;
  const double fast_velocity = 2.0 * max_generated_path_length / (horizon);
  constexpr double yaw_rate = 0.05;
  const auto imu_path = aeb_node_->generateEgoPath(fast_velocity, yaw_rate);

  ASSERT_FALSE(imu_path.empty());
  constexpr double epsilon{1e-3};
  ASSERT_TRUE(
    autoware::motion_utils::calcArcLength(imu_path) <=
    max_generated_path_length + dt * fast_velocity + epsilon);

  const auto footprint = aeb_node_->generatePathFootprint(imu_path, 0.0);
  ASSERT_FALSE(footprint.empty());
  ASSERT_TRUE(footprint.size() == imu_path.size() - 1);
}

TEST_F(TestAEB, checkEmptyPathAtZeroSpeed)
{
  const double velocity = 0.0;
  constexpr double yaw_rate = 0.0;
  const auto imu_path = aeb_node_->generateEgoPath(velocity, yaw_rate);
  ASSERT_EQ(imu_path.size(), 0);
}

TEST_F(TestAEB, checkParamUpdate)
{
  std::vector<rclcpp::Parameter> parameters{rclcpp::Parameter("param")};
  const auto result = aeb_node_->onParameter(parameters);
  ASSERT_TRUE(result.successful);
}

TEST_F(TestAEB, checkEmptyFetchData)
{
  ASSERT_FALSE(aeb_node_->fetchLatestData());
}

TEST_F(TestAEB, checkConvertObjectToPolygon)
{
  using autoware_perception_msgs::msg::Shape;
  PredictedObject obj_cylinder;
  obj_cylinder.shape.type = Shape::CYLINDER;
  obj_cylinder.shape.dimensions.x = 1.0;
  Pose obj_cylinder_pose;
  obj_cylinder_pose.position.x = 1.0;
  obj_cylinder_pose.position.y = 1.0;
  obj_cylinder.kinematics.initial_pose_with_covariance.pose = obj_cylinder_pose;
  const auto cylinder_polygon = utils::convertObjToPolygon(obj_cylinder);
  ASSERT_FALSE(cylinder_polygon.outer().empty());

  PredictedObject obj_box;
  obj_box.shape.type = Shape::BOUNDING_BOX;
  obj_box.shape.dimensions.x = 1.0;
  obj_box.shape.dimensions.y = 2.0;
  Pose obj_box_pose;
  obj_box_pose.position.x = 1.0;
  obj_box_pose.position.y = 1.0;
  obj_box.kinematics.initial_pose_with_covariance.pose = obj_box_pose;
  const auto box_polygon = utils::convertObjToPolygon(obj_box);
  ASSERT_FALSE(box_polygon.outer().empty());

  geometry_msgs::msg::TransformStamped tf_stamped;
  geometry_msgs::msg::Transform transform;

  constexpr double yaw{0.0};
  transform.rotation = autoware_utils::create_quaternion_from_yaw(yaw);
  geometry_msgs::msg::Vector3 translation;
  translation.x = 1.0;
  translation.y = 0.0;
  translation.z = 0.0;
  transform.translation = translation;
  tf_stamped.set__transform(transform);
  const auto t_obj_box = utils::transformObjectFrame(obj_box, tf_stamped);
  const auto t_pose = t_obj_box.kinematics.initial_pose_with_covariance.pose;
  Pose expected_pose;
  expected_pose.position.x = obj_box_pose.position.x + translation.x;
  expected_pose.position.y = obj_box_pose.position.y + translation.y;
  expected_pose.position.z = obj_box_pose.position.z + translation.z;

  ASSERT_DOUBLE_EQ(expected_pose.position.x, t_pose.position.x);
  ASSERT_DOUBLE_EQ(expected_pose.position.y, t_pose.position.y);
  ASSERT_DOUBLE_EQ(expected_pose.position.z, t_pose.position.z);
}

TEST_F(TestAEB, CollisionDataKeeper)
{
  using namespace std::literals::chrono_literals;
  constexpr double collision_keeping_sec{1.0}, previous_obstacle_keep_time{1.0};
  CollisionDataKeeper collision_data_keeper_(aeb_node_->get_clock());
  collision_data_keeper_.setTimeout(collision_keeping_sec, previous_obstacle_keep_time);
  ASSERT_TRUE(collision_data_keeper_.checkCollisionExpired());
  ASSERT_TRUE(collision_data_keeper_.checkPreviousObjectDataExpired());

  ObjectData obj;
  obj.stamp = aeb_node_->now();
  obj.velocity = 0.0;
  obj.position.x = 0.0;
  rclcpp::sleep_for(100ms);

  ObjectData obj2;
  obj2.stamp = aeb_node_->now();
  obj2.velocity = 0.0;
  obj2.position.x = 0.1;
  rclcpp::sleep_for(100ms);

  constexpr double ego_longitudinal_velocity = 3.0;
  constexpr double yaw_rate = 0.0;
  const auto imu_path = aeb_node_->generateEgoPath(ego_longitudinal_velocity, yaw_rate);

  const auto speed_null =
    collision_data_keeper_.calcObjectSpeedFromHistory(obj, imu_path, ego_longitudinal_velocity);
  ASSERT_FALSE(speed_null.has_value());

  const auto median_velocity =
    collision_data_keeper_.calcObjectSpeedFromHistory(obj2, imu_path, ego_longitudinal_velocity);
  ASSERT_TRUE(median_velocity.has_value());

  // object speed is 1.0 m/s greater than ego's = 0.1 [m] / 0.1 [s] + longitudinal_velocity
  ASSERT_TRUE(std::abs(median_velocity.value() - 4.0) < 1e-2);
  rclcpp::sleep_for(1100ms);
  ASSERT_TRUE(collision_data_keeper_.checkCollisionExpired());
}

TEST_F(TestAEB, TestCropPointCloud)
{
  constexpr double longitudinal_velocity = 3.0;
  constexpr double yaw_rate = 0.05;
  const auto imu_path = aeb_node_->generateEgoPath(longitudinal_velocity, yaw_rate);
  ASSERT_FALSE(imu_path.empty());

  constexpr size_t n_points{15};
  // Create n_points inside the path and 1 point outside.
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_points_ptr =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  {
    constexpr double x_start{0.0};
    constexpr double y_start{0.0};

    for (size_t i = 0; i < n_points; ++i) {
      const double offset_1 = static_cast<double>(i / 100.0);
      const double offset_2 = static_cast<double>((i + 10) / 100.0);
      pcl::PointXYZ p1(x_start + offset_1, y_start - offset_1, 0.5);
      pcl::PointXYZ p2(x_start + offset_2, y_start - offset_1, 0.5);
      obstacle_points_ptr->push_back(p1);
      obstacle_points_ptr->push_back(p2);
    }
    pcl::PointXYZ p_out(x_start + 100.0, y_start + 100, 0.5);
    obstacle_points_ptr->push_back(p_out);
  }
  aeb_node_->obstacle_ros_pointcloud_ptr_ = std::make_shared<PointCloud2>();
  pcl::toROSMsg(*obstacle_points_ptr, *aeb_node_->obstacle_ros_pointcloud_ptr_);
  const auto footprint = aeb_node_->generatePathFootprint(imu_path, 0.0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_objects =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  aeb_node_->cropPointCloudWithEgoFootprintPath(footprint, filtered_objects);
  // Check if the point outside the path was excluded
  ASSERT_TRUE(filtered_objects->points.size() == 2 * n_points);
}

// Repeatedly publish then pump both executors, giving the AEB node's polling subscribers time to
// discover the publishers and receive the (transient-local) samples before fetchLatestData reads.
// AEB derives from autoware::agnocast_wrapper::Node, which is not an rclcpp::Node, so it is pumped
// through its node base interface.
void deliver(
  const rclcpp::Node::SharedPtr & pub_node, const std::shared_ptr<AEB> & sub_node,
  const std::function<void()> & publish_fn, const int iterations = 40)
{
  for (int i = 0; i < iterations; ++i) {
    publish_fn();
    rclcpp::spin_some(pub_node);
    rclcpp::spin_some(sub_node->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

// Pin generateEgoPath(v, w) against an INDEPENDENT closed-form bicycle-model recurrence written
// directly in this test (never calling the SUT and never reading its output back). The stop
// condition is constrained to the time horizon alone so the pose count is deterministic.
TEST_F(TestAEB, imuPathBicycleModelOracle)
{
  aeb_node_->limit_imu_path_length_ = false;
  aeb_node_->limit_imu_path_lat_dev_ = false;
  aeb_node_->min_generated_imu_path_length_ = 0.0;
  aeb_node_->imu_prediction_time_interval_ = 0.1;
  aeb_node_->imu_prediction_time_horizon_ = 0.5;
  constexpr double dt = 0.1;

  // Independent recurrence: position advances with the PREVIOUS yaw, then yaw integrates. The loop
  // breaks once t exceeds the horizon (0.5 s), matching the node's break-before-push ordering.
  const auto expected_poses = [&](const double v, const double w) {
    std::vector<std::array<double, 3>> poses;  // {x, y, yaw}
    poses.push_back({0.0, 0.0, 0.0});
    double x = 0.0, y = 0.0, yaw = 0.0, t = 0.0;
    while (true) {
      const double next_x = x + v * std::cos(yaw) * dt;
      const double next_y = y + v * std::sin(yaw) * dt;
      const double next_yaw = yaw + w * dt;
      t += dt;
      if (t > 0.5) break;
      x = next_x;
      y = next_y;
      yaw = next_yaw;
      poses.push_back({x, y, yaw});
    }
    return poses;
  };

  // Turning case.
  {
    constexpr double v = 1.0;
    constexpr double w = 0.2;
    const auto path = aeb_node_->generateEgoPath(v, w);
    const auto oracle = expected_poses(v, w);
    ASSERT_EQ(path.size(), 6u);
    ASSERT_EQ(path.size(), oracle.size());
    // Anchor the first integrated pose with pure numeric literals.
    EXPECT_NEAR(path.at(1).position.x, 0.1, 1e-9);
    EXPECT_NEAR(path.at(1).position.y, 0.0, 1e-9);
    EXPECT_NEAR(tf2::getYaw(path.at(1).orientation), 0.02, 1e-9);
    for (size_t i = 0; i < path.size(); ++i) {
      EXPECT_NEAR(path.at(i).position.x, oracle.at(i).at(0), 1e-9);
      EXPECT_NEAR(path.at(i).position.y, oracle.at(i).at(1), 1e-9);
      EXPECT_NEAR(tf2::getYaw(path.at(i).orientation), oracle.at(i).at(2), 1e-9);
    }
  }

  // Degenerate straight-line case (w = 0): pure-literal oracle, y and yaw stay zero.
  {
    const auto path = aeb_node_->generateEgoPath(1.0, 0.0);
    ASSERT_EQ(path.size(), 6u);
    const std::array<double, 6> expected_x{0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
    for (size_t i = 0; i < path.size(); ++i) {
      EXPECT_NEAR(path.at(i).position.x, expected_x.at(i), 1e-9);
      EXPECT_NEAR(path.at(i).position.y, 0.0, 1e-12);
      EXPECT_NEAR(tf2::getYaw(path.at(i).orientation), 0.0, 1e-12);
    }
  }
}

// The yaw rate must be copied verbatim from the published kinematic_state twist into
// angular_velocity_ptr_->z (a direct field copy, so exact equality is the correct oracle).
TEST_F(TestAEB, kinematicStateYawRatePlumbing)
{
  aeb_node_->check_autoware_state_ = false;
  aeb_node_->use_pointcloud_data_ = false;
  aeb_node_->use_predicted_object_data_ = true;  // satisfies the object-detection-method gate
  aeb_node_->use_predicted_trajectory_ = false;  // isolate the IMU (kinematic_state) path
  aeb_node_->use_imu_path_ = true;

  constexpr double yaw_rate = -0.1234;
  const auto publish = [&]() {
    const auto header = get_header("base_link", pub_sub_node_->now());
    pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
    pub_sub_node_->pub_predicted_objects_->publish(PredictedObjects{});
    pub_sub_node_->pub_kinematic_state_->publish(make_odometry_message(header, yaw_rate));
  };
  deliver(pub_sub_node_, aeb_node_, publish);

  ASSERT_TRUE(aeb_node_->fetchLatestData());
  ASSERT_NE(aeb_node_->angular_velocity_ptr_, nullptr);
  ASSERT_DOUBLE_EQ(aeb_node_->angular_velocity_ptr_->z, yaw_rate);
}

// Intake guard: with the IMU path enabled and no other path, a missing kinematic_state makes
// fetchLatestData fail; supplying kinematic_state flips it to succeed (positive control).
TEST_F(TestAEB, missingKinematicStateTripsImuPathGuard)
{
  aeb_node_->check_autoware_state_ = false;
  aeb_node_->use_pointcloud_data_ = false;
  aeb_node_->use_predicted_object_data_ = true;
  aeb_node_->use_predicted_trajectory_ = false;
  aeb_node_->use_imu_path_ = true;

  const auto publish_without_kinematic_state = [&]() {
    const auto header = get_header("base_link", pub_sub_node_->now());
    pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
    pub_sub_node_->pub_predicted_objects_->publish(PredictedObjects{});
  };
  deliver(pub_sub_node_, aeb_node_, publish_without_kinematic_state);
  EXPECT_FALSE(aeb_node_->fetchLatestData());

  const auto publish_with_kinematic_state = [&]() {
    const auto header = get_header("base_link", pub_sub_node_->now());
    pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
    pub_sub_node_->pub_predicted_objects_->publish(PredictedObjects{});
    pub_sub_node_->pub_kinematic_state_->publish(make_odometry_message(header, 0.05));
  };
  deliver(pub_sub_node_, aeb_node_, publish_with_kinematic_state);
  EXPECT_TRUE(aeb_node_->fetchLatestData());
}

// With use_imu_path=false, the kinematic_state subscription must not be consulted at all: the path
// comes solely from the predicted trajectory and angular_velocity_ptr_ stays null.
TEST_F(TestAEB, useImuPathFalseIgnoresKinematicState)
{
  aeb_node_->check_autoware_state_ = false;
  aeb_node_->use_pointcloud_data_ = false;
  aeb_node_->use_predicted_object_data_ = true;
  aeb_node_->use_predicted_trajectory_ = true;
  aeb_node_->use_imu_path_ = false;

  const auto publish = [&]() {
    const auto header = get_header("base_link", pub_sub_node_->now());
    pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
    pub_sub_node_->pub_predicted_objects_->publish(PredictedObjects{});
    pub_sub_node_->pub_predicted_traj_->publish(Trajectory{});
    // kinematic_state deliberately withheld.
  };
  deliver(pub_sub_node_, aeb_node_, publish);

  ASSERT_TRUE(aeb_node_->fetchLatestData());
  EXPECT_EQ(aeb_node_->angular_velocity_ptr_, nullptr);
}

// The kinematic_state yaw source (topic ~/input/kinematic_state) has no producer-side heartbeat
// guarantee, and the Latest polling subscriber returns the last Odometry forever, so a frozen or
// diverged localization would feed a stale yaw rate into the integrated ego-path prediction
// indefinitely. A kinematic_state older than kinematic_state_timeout_sec_ must make the IMU-path
// yaw source read as UNAVAILABLE (angular_velocity_ptr_ reset to null so the IMU path is skipped),
// never as a usable estimate and never degraded to a zero-yaw straight-line estimate. Independent
// oracle: the age is hand-set through the Odometry header stamp and compared against the pinned
// package default; a second (predicted-trajectory) path is supplied so fetchLatestData can still
// succeed and the assertion isolates the yaw source alone.
TEST_F(TestAEB, kinematicStateStalenessWatchdog)
{
  // Pin the shipped default so it cannot change silently. This is an initial conservative value,
  // NOT one calibrated against measured on-vehicle jitter -- see the PR description.
  ASSERT_DOUBLE_EQ(aeb_node_->kinematic_state_timeout_sec_, 0.5);
  aeb_node_->check_autoware_state_ = false;
  aeb_node_->use_pointcloud_data_ = false;
  aeb_node_->use_predicted_object_data_ = true;  // satisfies the object-detection-method gate
  aeb_node_->use_predicted_trajectory_ = true;   // yaw-independent second path -> fetch can succeed
  aeb_node_->use_imu_path_ = true;

  constexpr double yaw_rate = 0.05;

  // Fresh kinematic_state (stamp = publish time): age ~0 < 0.5 s -> yaw source accepted and
  // angular_velocity_ptr_ is populated with the published yaw rate.
  const auto publish_fresh = [&]() {
    const auto header = get_header("base_link", pub_sub_node_->now());
    pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
    pub_sub_node_->pub_predicted_objects_->publish(PredictedObjects{});
    pub_sub_node_->pub_predicted_traj_->publish(Trajectory{});
    pub_sub_node_->pub_kinematic_state_->publish(make_odometry_message(header, yaw_rate));
  };
  deliver(pub_sub_node_, aeb_node_, publish_fresh);
  ASSERT_TRUE(aeb_node_->fetchLatestData());
  ASSERT_NE(aeb_node_->angular_velocity_ptr_, nullptr);
  ASSERT_DOUBLE_EQ(aeb_node_->angular_velocity_ptr_->z, yaw_rate);

  // Stale kinematic_state (stamp = 5 s in the past): age 5 s > 0.5 s -> the watchdog must trip; the
  // yaw source is treated as unavailable so angular_velocity_ptr_ is reset to null and the IMU path
  // is skipped this cycle. fetchLatestData still SUCCEEDS because the predicted trajectory supplies
  // a second, yaw-independent path -> a stale twist is never integrated into the ego-path
  // prediction.
  const auto publish_stale = [&]() {
    const auto header = get_header("base_link", pub_sub_node_->now());
    pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
    pub_sub_node_->pub_predicted_objects_->publish(PredictedObjects{});
    pub_sub_node_->pub_predicted_traj_->publish(Trajectory{});
    const auto stale_stamp =
      rclcpp::Time(pub_sub_node_->now()) - rclcpp::Duration::from_seconds(5.0);
    const auto stale_header = get_header("base_link", stale_stamp);
    pub_sub_node_->pub_kinematic_state_->publish(make_odometry_message(stale_header, yaw_rate));
  };
  deliver(pub_sub_node_, aeb_node_, publish_stale);
  EXPECT_TRUE(aeb_node_->fetchLatestData());
  EXPECT_EQ(aeb_node_->angular_velocity_ptr_, nullptr);  // stale yaw must NOT feed prediction
}

// A stamp AHEAD of the node clock must read as unusable too. The watchdog computes a SIGNED age, so
// a one-sided (age > timeout) test would see a negative age here and accept a frozen future-stamped
// twist forever (ECU clock skew, or a faulty producer) -- the same fail-open class the watchdog
// exists to close. This test discriminates the symmetric +/- tolerance from both a one-sided check
// and from an over-strict "reject any negative age", which would trip on ordinary clock jitter:
//   - stamp 5.0 s in the FUTURE, far outside the 0.5 s tolerance -> must be rejected;
//   - stamp 0.2 s in the future, inside the tolerance            -> must still be accepted.
// Independent oracle: both offsets are hand-set on the Odometry header and compared against the
// pinned package default, and a predicted-trajectory path is supplied so fetchLatestData still
// succeeds and each assertion isolates the yaw source alone.
TEST_F(TestAEB, futureStampedKinematicStateTripsWatchdog)
{
  ASSERT_DOUBLE_EQ(aeb_node_->kinematic_state_timeout_sec_, 0.5);
  aeb_node_->check_autoware_state_ = false;
  aeb_node_->use_pointcloud_data_ = false;
  aeb_node_->use_predicted_object_data_ = true;
  aeb_node_->use_predicted_trajectory_ = true;  // yaw-independent second path -> fetch can succeed
  aeb_node_->use_imu_path_ = true;

  constexpr double yaw_rate = 0.05;

  const auto publish_with_stamp_offset = [&](const double offset_sec) {
    return [&, offset_sec]() {
      const auto header = get_header("base_link", pub_sub_node_->now());
      pub_sub_node_->pub_velocity_->publish(make_velocity_report_msg(header, 0.0, 3.0, 0.0));
      pub_sub_node_->pub_predicted_objects_->publish(PredictedObjects{});
      pub_sub_node_->pub_predicted_traj_->publish(Trajectory{});
      const auto shifted_stamp =
        rclcpp::Time(pub_sub_node_->now()) + rclcpp::Duration::from_seconds(offset_sec);
      const auto shifted_header = get_header("base_link", shifted_stamp);
      pub_sub_node_->pub_kinematic_state_->publish(make_odometry_message(shifted_header, yaw_rate));
    };
  };

  // Far-future stamp: |age| = 5 s > 0.5 s -> yaw source unavailable, IMU path skipped.
  deliver(pub_sub_node_, aeb_node_, publish_with_stamp_offset(5.0));
  EXPECT_TRUE(aeb_node_->fetchLatestData());
  EXPECT_EQ(aeb_node_->angular_velocity_ptr_, nullptr);  // future yaw must NOT feed prediction

  // Slightly-future stamp within tolerance: still accepted, so healthy clock jitter does not
  // spuriously disable the IMU path.
  deliver(pub_sub_node_, aeb_node_, publish_with_stamp_offset(0.2));
  ASSERT_TRUE(aeb_node_->fetchLatestData());
  ASSERT_NE(aeb_node_->angular_velocity_ptr_, nullptr);
  EXPECT_DOUBLE_EQ(aeb_node_->angular_velocity_ptr_->z, yaw_rate);
}

}  // namespace autoware::motion::control::autonomous_emergency_braking::test
