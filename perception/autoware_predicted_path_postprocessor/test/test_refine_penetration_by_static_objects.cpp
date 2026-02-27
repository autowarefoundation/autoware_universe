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

#include "autoware/predicted_path_postprocessor/processor/refine_penetration_by_static_objects.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::predicted_path_postprocessor::testing
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::Shape;
using processor::Context;
using processor::RefinePenetrationByStaticObjects;

class RefinePenetrationByStaticObjectsTest : public ::testing::Test
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

    processor_ = std::make_unique<RefinePenetrationByStaticObjects>(
      node_.get(), "refine_penetration_by_static_objects");
  }

  void TearDown() override { rclcpp::shutdown(); }

  unique_identifier_msgs::msg::UUID make_uuid(uint8_t seed)
  {
    unique_identifier_msgs::msg::UUID uuid;
    uuid.uuid[0] = seed;
    return uuid;
  }

  PredictedObject create_target_object(
    double speed, const std::vector<std::array<double, 3>> & waypoints, double time_step = 1.0,
    uint8_t uuid_seed = 1)
  {
    PredictedObject object;
    object.object_id = make_uuid(uuid_seed);
    object.shape.type = Shape::BOUNDING_BOX;
    // Give some size (not used for target collision check itself)
    object.shape.dimensions.x = 1.0;
    object.shape.dimensions.y = 1.0;
    object.shape.dimensions.z = 1.0;

    object.kinematics.initial_twist_with_covariance.twist.linear.x = speed;

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

  PredictedObject create_static_obstacle(
    const geometry_msgs::msg::Point & center, double length_x, double length_y, uint8_t uuid_seed,
    double speed = 0.0)
  {
    PredictedObject obstacle;
    obstacle.object_id = make_uuid(uuid_seed);
    obstacle.shape.type = Shape::BOUNDING_BOX;
    obstacle.shape.dimensions.x = length_x;
    obstacle.shape.dimensions.y = length_y;
    obstacle.shape.dimensions.z = 1.0;
    obstacle.kinematics.initial_pose_with_covariance.pose.position = center;
    obstacle.kinematics.initial_pose_with_covariance.pose.orientation =
      autoware_utils_geometry::create_quaternion_from_yaw(0.0);
    obstacle.kinematics.initial_twist_with_covariance.twist.linear.x = speed;

    // Provide at least one predicted path (not required by collision fn but keeps consistency)
    PredictedPath path;
    path.time_step = rclcpp::Duration::from_seconds(1.0);
    path.path.push_back(obstacle.kinematics.initial_pose_with_covariance.pose);
    obstacle.kinematics.predicted_paths.push_back(path);
    return obstacle;
  }

  double distance_2d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
  {
    return std::hypot(b.x - a.x, b.y - a.y);
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<RefinePenetrationByStaticObjects> processor_;
};

// --- Basic Construction ---
TEST_F(RefinePenetrationByStaticObjectsTest, ConstructorInitializesName)
{
  EXPECT_EQ(processor_->name(), "refine_penetration_by_static_objects");
}

// --- Context Validation ---
TEST_F(RefinePenetrationByStaticObjectsTest, FailsWithoutObjectsInContext)
{
  auto target =
    create_target_object(0.5, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}});
  Context empty_context;
  auto result = processor_->run(target, empty_context);
  EXPECT_FALSE(result);
  EXPECT_TRUE(result.is_err());
  EXPECT_NE(result.err().find("Context does not contain objects"), std::string::npos)
    << "Error message should mention missing context objects";
}

// --- High Speed Skip ---
TEST_F(RefinePenetrationByStaticObjectsTest, RefinesHighSpeedTargetOnCollision)
{
  // Path from 0 to 5 on x-axis (base_keys: 0,1,2,3,4,5)
  auto target =
    create_target_object(5.0, {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}});

  // Static obstacle causes collision at x=2.0 (segment between 2 and 3 clipped)
  geometry_msgs::msg::Point center;
  center.x = 2.5;
  center.y = 0.0;
  center.z = 0.0;
  auto obstacle = create_static_obstacle(center, 1.0, 2.0, 99);

  PredictedObjects obstacles_msg;
  obstacles_msg.objects.push_back(obstacle);
  Context context;
  context.update(std::make_shared<PredictedObjects>(obstacles_msg));

  ASSERT_TRUE(processor_->run(target, context));

  // High-speed target is still refined; query_keys: [0,5,10,15,20,25] -> all clamped to 2.0 except
  // index 0
  const auto & path = target.kinematics.predicted_paths[0].path;
  ASSERT_EQ(path.size(), 6);
  EXPECT_NEAR(path[0].position.x, 0.0, 1e-6);
  for (size_t i = 1; i < path.size(); ++i) {
    EXPECT_NEAR(path[i].position.x, 2.0, 1e-6);
    EXPECT_NEAR(path[i].position.y, 0.0, 1e-6);
  }
}

// --- Simple Collision Refinement ---
TEST_F(RefinePenetrationByStaticObjectsTest, RefinesPathAtFirstCollision)
{
  // Target speed below threshold (1.0)
  auto target =
    create_target_object(0.5, {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}});

  // Obstacle intersects path between 2.0 and 3.0 (center 2.5 length 1.0)
  geometry_msgs::msg::Point center;
  center.x = 2.5;
  center.y = 0.0;
  center.z = 0.0;
  auto obstacle = create_static_obstacle(center, 1.0, 2.0, 2);

  PredictedObjects obstacles_msg;
  obstacles_msg.objects.push_back(obstacle);
  Context context;
  context.update(std::make_shared<PredictedObjects>(obstacles_msg));

  auto original_path = target.kinematics.predicted_paths[0].path;
  ASSERT_TRUE(processor_->run(target, context));

  const auto & processed_path = target.kinematics.predicted_paths[0].path;
  ASSERT_EQ(processed_path.size(), original_path.size());

  // Collision expected at x=2.0 -> points after that should clamp to 2.0
  EXPECT_NEAR(processed_path[0].position.x, 0.0, 1e-6);
  EXPECT_NEAR(processed_path[1].position.x, 0.5, 1e-3);  // interpolated 0.5
  EXPECT_NEAR(processed_path[2].position.x, 1.0, 1e-6);
  EXPECT_NEAR(processed_path[3].position.x, 1.5, 1e-6);
  EXPECT_NEAR(processed_path[4].position.x, 2.0, 1e-6);
  EXPECT_NEAR(processed_path[5].position.x, 2.0, 1e-6);  // clamped from original 5.0

  // All y should remain 0
  for (const auto & p : processed_path) {
    EXPECT_NEAR(p.position.y, 0.0, 1e-6);
  }
}

// --- No Collision ---
TEST_F(RefinePenetrationByStaticObjectsTest, LeavesPathUnchangedIfNoCollision)
{
  auto target =
    create_target_object(0.5, {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}});

  // Place obstacle off to the side (y=5)
  geometry_msgs::msg::Point center;
  center.x = 2.5;
  center.y = 5.0;
  center.z = 0.0;
  auto obstacle = create_static_obstacle(center, 1.0, 2.0, 3);

  PredictedObjects obstacles_msg;
  obstacles_msg.objects.push_back(obstacle);
  Context context;
  context.update(std::make_shared<PredictedObjects>(obstacles_msg));

  auto original_path = target.kinematics.predicted_paths[0].path;
  ASSERT_TRUE(processor_->run(target, context));

  const auto & processed_path = target.kinematics.predicted_paths[0].path;
  ASSERT_EQ(processed_path.size(), original_path.size());
  for (size_t i = 0; i < processed_path.size(); ++i) {
    EXPECT_NEAR(processed_path[i].position.x, original_path[i].position.x, 1e-6);
    EXPECT_NEAR(processed_path[i].position.y, original_path[i].position.y, 1e-6);
  }
}

// --- Collision Late In Path ---
TEST_F(RefinePenetrationByStaticObjectsTest, RefinesProgressWithNoClampIfCollisionBeyondQueryRange)
{
  auto target =
    create_target_object(0.5, {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}});

  // Obstacle intersection at x=4.0, but speed * dt produces query keys only up to 2.5:
  // speed=0.5, dt=1.0 -> query_keys: [0,0.5,1.0,1.5,2.0,2.5] (< 4.0 no clamping occurs)
  geometry_msgs::msg::Point center;
  center.x = 4.5;
  center.y = 0.0;
  center.z = 0.0;
  auto obstacle = create_static_obstacle(center, 1.0, 2.0, 4);

  PredictedObjects obstacles_msg;
  obstacles_msg.objects.push_back(obstacle);
  Context context;
  context.update(std::make_shared<PredictedObjects>(obstacles_msg));

  ASSERT_TRUE(processor_->run(target, context));
  const auto & path = target.kinematics.predicted_paths[0].path;
  ASSERT_EQ(path.size(), 6);
  EXPECT_NEAR(path[0].position.x, 0.0, 1e-6);
  EXPECT_NEAR(path[1].position.x, 0.5, 1e-6);
  EXPECT_NEAR(path[2].position.x, 1.0, 1e-6);
  EXPECT_NEAR(path[3].position.x, 1.5, 1e-6);
  EXPECT_NEAR(path[4].position.x, 2.0, 1e-6);
  EXPECT_NEAR(path[5].position.x, 2.5, 1e-6);
}

// --- Multiple Obstacles Earliest Collision ---
TEST_F(RefinePenetrationByStaticObjectsTest, UsesEarliestCollisionAmongMultipleObstacles)
{
  auto target =
    create_target_object(0.5, {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}});

  // Two obstacles: one causing collision at x=2.0, another at x=4.0
  geometry_msgs::msg::Point center1;
  center1.x = 2.5;  // collision at 2.0
  center1.y = 0.0;
  center1.z = 0.0;
  auto obstacle1 = create_static_obstacle(center1, 1.0, 2.0, 10);

  geometry_msgs::msg::Point center2;
  center2.x = 4.5;  // collision at 4.0
  center2.y = 0.0;
  center2.z = 0.0;
  auto obstacle2 = create_static_obstacle(center2, 1.0, 2.0, 11);

  PredictedObjects obstacles_msg;
  obstacles_msg.objects.push_back(obstacle2);
  obstacles_msg.objects.push_back(obstacle1);  // Order reversed intentionally
  Context context;
  context.update(std::make_shared<PredictedObjects>(obstacles_msg));

  ASSERT_TRUE(processor_->run(target, context));
  const auto & processed_path = target.kinematics.predicted_paths[0].path;
  // Tail should clamp at first collision (x=2.0)
  EXPECT_NEAR(processed_path[4].position.x, 2.0, 1e-6);
  EXPECT_NEAR(processed_path[5].position.x, 2.0, 1e-6);
}

// --- Negative Speed (Absolute Value Handling) ---
TEST_F(RefinePenetrationByStaticObjectsTest, HandlesNegativeSpeedSameAsPositive)
{
  auto target =
    create_target_object(-0.5, {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}});

  geometry_msgs::msg::Point center;
  center.x = 2.5;
  center.y = 0.0;
  center.z = 0.0;
  auto obstacle = create_static_obstacle(center, 1.0, 2.0, 20);

  PredictedObjects obstacles_msg;
  obstacles_msg.objects.push_back(obstacle);
  Context context;
  context.update(std::make_shared<PredictedObjects>(obstacles_msg));

  ASSERT_TRUE(processor_->run(target, context));
  const auto & processed_path = target.kinematics.predicted_paths[0].path;
  EXPECT_NEAR(processed_path[5].position.x, 2.0, 1e-6);
}

// --- Zero Time Step (Skip Processing) ---
TEST_F(RefinePenetrationByStaticObjectsTest, SkipsWhenTimeStepIsZero)
{
  auto target =
    create_target_object(0.5, {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}}, 0.0, 30);  // time_step = 0.0

  geometry_msgs::msg::Point center;
  center.x = 1.5;
  center.y = 0.0;
  center.z = 0.0;
  auto obstacle = create_static_obstacle(center, 1.0, 2.0, 31);

  PredictedObjects obstacles_msg;
  obstacles_msg.objects.push_back(obstacle);
  Context context;
  context.update(std::make_shared<PredictedObjects>(obstacles_msg));

  auto original_path = target.kinematics.predicted_paths[0].path;
  ASSERT_TRUE(processor_->run(target, context));
  const auto & processed_path = target.kinematics.predicted_paths[0].path;
  ASSERT_EQ(processed_path.size(), original_path.size());
  for (size_t i = 0; i < processed_path.size(); ++i) {
    EXPECT_NEAR(processed_path[i].position.x, original_path[i].position.x, 1e-6);
  }
}

// --- Single Waypoint (Skip Processing) ---
TEST_F(RefinePenetrationByStaticObjectsTest, SkipsWhenOnlyOneWaypoint)
{
  auto target = create_target_object(0.5, {{0, 0, 0}}, 1.0, 40);

  geometry_msgs::msg::Point center;
  center.x = 0.0;
  center.y = 0.0;
  center.z = 0.0;
  auto obstacle = create_static_obstacle(center, 1.0, 2.0, 41);

  PredictedObjects obstacles_msg;
  obstacles_msg.objects.push_back(obstacle);
  Context context;
  context.update(std::make_shared<PredictedObjects>(obstacles_msg));

  ASSERT_TRUE(processor_->run(target, context));
  const auto & processed_path = target.kinematics.predicted_paths[0].path;
  ASSERT_EQ(processed_path.size(), 1);
  EXPECT_NEAR(processed_path[0].position.x, 0.0, 1e-6);
}

// --- Orientation Validity After Refinement ---
TEST_F(RefinePenetrationByStaticObjectsTest, ProducesNormalizedOrientations)
{
  auto target =
    create_target_object(0.5, {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}});

  geometry_msgs::msg::Point center;
  center.x = 2.5;
  center.y = 0.0;
  center.z = 0.0;
  auto obstacle = create_static_obstacle(center, 1.0, 2.0, 50);

  PredictedObjects obstacles_msg;
  obstacles_msg.objects.push_back(obstacle);
  Context context;
  context.update(std::make_shared<PredictedObjects>(obstacles_msg));

  ASSERT_TRUE(processor_->run(target, context));
  const auto & processed_path = target.kinematics.predicted_paths[0].path;

  for (size_t i = 1; i < processed_path.size(); ++i) {
    const auto & q = processed_path[i].orientation;
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    EXPECT_NEAR(norm, 1.0, 1e-6);
  }
}

// --- Multiple Predicted Paths ---
TEST_F(RefinePenetrationByStaticObjectsTest, ProcessesAllPredictedPaths)
{
  PredictedObject target;
  target.object_id = make_uuid(60);
  target.shape.type = Shape::BOUNDING_BOX;
  target.shape.dimensions.x = 1.0;
  target.shape.dimensions.y = 1.0;
  target.shape.dimensions.z = 1.0;
  target.kinematics.initial_twist_with_covariance.twist.linear.x = 0.5;  // low speed

  // Create two paths; both collide at x=2.0
  for (int path_idx = 0; path_idx < 2; ++path_idx) {
    PredictedPath path;
    path.time_step = rclcpp::Duration::from_seconds(1.0);
    std::vector<double> ys = {static_cast<double>(path_idx), static_cast<double>(path_idx),
                              static_cast<double>(path_idx), static_cast<double>(path_idx),
                              static_cast<double>(path_idx), static_cast<double>(path_idx)};
    std::vector<double> xs = {0, 1, 2, 3, 4, 5};
    for (size_t i = 0; i < xs.size(); ++i) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = xs[i];
      pose.position.y = ys[i];
      pose.position.z = 0.0;
      pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
      path.path.push_back(pose);
    }
    target.kinematics.predicted_paths.push_back(path);
  }

  geometry_msgs::msg::Point center;
  center.x = 2.5;
  center.y = 0.0;
  center.z = 0.0;
  auto obstacle = create_static_obstacle(center, 1.0, 4.0, 61);  // spans both paths laterally

  PredictedObjects obstacles_msg;
  obstacles_msg.objects.push_back(obstacle);
  Context context;
  context.update(std::make_shared<PredictedObjects>(obstacles_msg));

  ASSERT_TRUE(processor_->run(target, context));

  for (const auto & path : target.kinematics.predicted_paths) {
    ASSERT_EQ(path.path.size(), 6);
    EXPECT_NEAR(path.path[5].position.x, 2.0, 1e-6);
  }
}

// --- Z Coordinate Preservation ---
TEST_F(RefinePenetrationByStaticObjectsTest, PreservesZCoordinates)
{
  auto target =
    create_target_object(0.5, {{0, 0, 1}, {1, 0, 1}, {2, 0, 1}, {3, 0, 1}, {4, 0, 1}, {5, 0, 1}});

  geometry_msgs::msg::Point center;
  center.x = 2.5;
  center.y = 0.0;
  center.z = 1.0;
  auto obstacle = create_static_obstacle(center, 1.0, 2.0, 70);

  PredictedObjects obstacles_msg;
  obstacles_msg.objects.push_back(obstacle);
  Context context;
  context.update(std::make_shared<PredictedObjects>(obstacles_msg));

  ASSERT_TRUE(processor_->run(target, context));
  const auto & path = target.kinematics.predicted_paths[0].path;
  for (const auto & pose : path) {
    EXPECT_NEAR(pose.position.z, 1.0, 1e-6);
  }
}

// --- Monotonicity Of Distances Up To Collision ---
TEST_F(RefinePenetrationByStaticObjectsTest, MaintainsMonotonicDistanceUntilCollision)
{
  auto target =
    create_target_object(0.5, {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}});

  geometry_msgs::msg::Point center;
  center.x = 2.5;
  center.y = 0.0;
  center.z = 0.0;
  auto obstacle = create_static_obstacle(center, 1.0, 2.0, 80);

  PredictedObjects obstacles_msg;
  obstacles_msg.objects.push_back(obstacle);
  Context context;
  context.update(std::make_shared<PredictedObjects>(obstacles_msg));

  ASSERT_TRUE(processor_->run(target, context));
  const auto & path = target.kinematics.predicted_paths[0].path;

  double prev_dist = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    double current_dist = distance_2d(path[0].position, path[i].position);
    EXPECT_GE(current_dist, prev_dist - 1e-6);
    prev_dist = current_dist;
  }
  // Final distance should equal collision distance (2.0)
  EXPECT_NEAR(prev_dist, 2.0, 1e-6);
}

// --- No Refinement When Obstacle Is Moving Faster Than Threshold ---
TEST_F(RefinePenetrationByStaticObjectsTest, SkipsObstacleMovingFasterThanThreshold)
{
  auto target =
    create_target_object(0.5, {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}});

  geometry_msgs::msg::Point center;
  center.x = 2.5;
  center.y = 0.0;
  center.z = 0.0;
  // Obstacle moving fast -> should be ignored; no collision considered
  auto fast_obstacle = create_static_obstacle(center, 1.0, 2.0, 90, 10.0);

  PredictedObjects obstacles_msg;
  obstacles_msg.objects.push_back(fast_obstacle);
  Context context;
  context.update(std::make_shared<PredictedObjects>(obstacles_msg));

  auto original_path = target.kinematics.predicted_paths[0].path;
  ASSERT_TRUE(processor_->run(target, context));
  const auto & processed_path = target.kinematics.predicted_paths[0].path;

  for (size_t i = 0; i < processed_path.size(); ++i) {
    EXPECT_NEAR(processed_path[i].position.x, original_path[i].position.x, 1e-6);
  }
}
}  // namespace autoware::predicted_path_postprocessor::testing
