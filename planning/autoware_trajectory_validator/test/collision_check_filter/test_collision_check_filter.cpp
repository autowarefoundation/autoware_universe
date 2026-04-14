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

#include "../../src/filters/safety/collision_check_filter.cpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{

class CollisionCheckFilterTest : public ::testing::Test
{
protected:
  std::unique_ptr<CollisionCheckFilter> filter_;

  void SetUp() override
  {
    filter_ = std::make_unique<CollisionCheckFilter>();

    autoware::vehicle_info_utils::VehicleInfo vehicle_info;

    vehicle_info.max_longitudinal_offset_m = 4.0;
    vehicle_info.min_longitudinal_offset_m = -1.0;
    vehicle_info.vehicle_width_m = 2.0;

    filter_->set_vehicle_info(vehicle_info);
  }
  geometry_msgs::msg::Twist create_twist(double linear_x, double angular_z)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = linear_x;
    t.angular.z = angular_z;
    return t;
  }

  geometry_msgs::msg::Pose create_pose(double x, double y, double yaw)
  {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = 0.0;
    p.orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);
    return p;
  }

  TrajectoryPoints create_ego_path()
  {
    TrajectoryPoints traj;
    for (size_t i = 0; i <= 100; ++i) {
      TrajectoryPoint pt;
      pt.pose = create_pose(i * 1.0, 0.0, 0.0);
      // do not initialize pt.twist
      traj.push_back(pt);
    }
    return traj;
  }

  autoware_perception_msgs::msg::PredictedPath create_predicted_path(
    const geometry_msgs::msg::Pose & initial_pose, const geometry_msgs::msg::Twist & twist)
  {
    const double assumed_lag = 0.0;
    const double assumed_acceleration = 0.0;
    const double max_time = 10.0;

    auto [times, distances] = trajectory::time_distance::compute_motion_profile_1d(
      twist, assumed_lag, assumed_acceleration, 0.0, max_time);
    auto pose_trajectory =
      trajectory::pose::constant_curvature_predictor::compute(initial_pose, twist, distances);

    autoware_perception_msgs::msg::PredictedPath predicted_path;
    predicted_path.confidence = 1.0;

    predicted_path.time_step = rclcpp::Duration::from_seconds(TIME_RESOLUTION);

    for (const auto & pose : pose_trajectory) {
      predicted_path.path.push_back(pose);
    }

    predicted_path.confidence = 1.0;

    return predicted_path;
  }

  autoware_perception_msgs::msg::Shape create_object_shape(double x, double y)
  {
    autoware_perception_msgs::msg::Shape shape;
    shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    shape.dimensions.x = x;
    shape.dimensions.y = y;
    shape.dimensions.z = 1.5;
    return shape;
  }

  autoware_perception_msgs::msg::PredictedObject create_dummy_object(
    geometry_msgs::msg::Pose pose, geometry_msgs::msg::Twist twist,
    autoware_perception_msgs::msg::PredictedPath predicted_path,
    autoware_perception_msgs::msg::Shape shape)
  {
    autoware_perception_msgs::msg::PredictedObject obj;

    obj.object_id = autoware_utils_uuid::generate_uuid();
    obj.kinematics.initial_pose_with_covariance.pose = pose;
    obj.kinematics.initial_twist_with_covariance.twist = twist;

    obj.kinematics.predicted_paths.resize(1);
    obj.kinematics.predicted_paths[0] = predicted_path;

    obj.shape = shape;

    obj.classification.resize(1);
    obj.classification.at(0).label = autoware_perception_msgs::msg::ObjectClassification::CAR;

    obj.existence_probability = 1.0;

    return obj;
  }
};

TEST_F(CollisionCheckFilterTest, EmptyObjects)
{
  const auto ego_path = create_ego_path();

  FilterContext context;

  auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->pose.pose = create_pose(0.0, 0.0, 0.0);
  odom_msg->twist.twist = create_twist(10.0, 0.0);
  context.odometry = odom_msg;

  context.predicted_objects = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();

  const auto result = filter_->is_feasible(ego_path, context);

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result.value().is_feasible);
}

TEST_F(CollisionCheckFilterTest, StoppedObjectInPath)
{
  const auto ego_path = create_ego_path();

  FilterContext context;

  auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->pose.pose = create_pose(0.0, 0.0, 0.0);
  odom_msg->twist.twist = create_twist(10.0, 0.0);
  context.odometry = odom_msg;

  auto predicted_objects_msg = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  auto pose = create_pose(20.0, 0.0, 0.0);
  auto twist = create_twist(0.0, 0.0);
  predicted_objects_msg->objects.push_back(create_dummy_object(
    pose, twist, create_predicted_path(pose, twist), create_object_shape(5.0, 1.0)));
  context.predicted_objects = predicted_objects_msg;

  const auto result = filter_->is_feasible(ego_path, context);

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result.value().is_feasible);
}

TEST_F(CollisionCheckFilterTest, ObjectWillDepartFromPath)
{
  const auto ego_path = create_ego_path();

  FilterContext context;

  auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->pose.pose = create_pose(0.0, 0.0, 0.0);
  odom_msg->twist.twist = create_twist(10.0, 0.0);
  context.odometry = odom_msg;

  auto predicted_objects_msg = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  auto pose = create_pose(20.0, 0.0, M_PI_2);
  auto twist = create_twist(10.0, 0.0);
  predicted_objects_msg->objects.push_back(create_dummy_object(
    pose, twist, create_predicted_path(pose, twist), create_object_shape(5.0, 1.0)));
  context.predicted_objects = predicted_objects_msg;

  const auto result = filter_->is_feasible(ego_path, context);

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result.value().is_feasible);
}

TEST_F(CollisionCheckFilterTest, ObjectWillEnterPath)
{
  const auto ego_path = create_ego_path();

  FilterContext context;

  auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->pose.pose = create_pose(0.0, 0.0, 0.0);
  odom_msg->twist.twist = create_twist(10.0, 0.0);
  context.odometry = odom_msg;

  auto predicted_objects_msg = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  auto pose = create_pose(20.0, -10.0, M_PI_2);
  auto twist = create_twist(10.0, 0.0);
  predicted_objects_msg->objects.push_back(create_dummy_object(
    pose, twist, create_predicted_path(pose, twist), create_object_shape(5.0, 1.0)));
  context.predicted_objects = predicted_objects_msg;

  const auto result = filter_->is_feasible(ego_path, context);

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result.value().is_feasible);
}

}  // namespace autoware::trajectory_validator::plugin::safety
