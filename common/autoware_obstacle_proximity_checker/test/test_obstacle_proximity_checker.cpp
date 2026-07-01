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

#include "autoware/obstacle_proximity_checker/obstacle_proximity_checker.hpp"

#include <Eigen/Geometry>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::obstacle_proximity_checker
{
namespace
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;

vehicle_info_utils::VehicleInfo make_vehicle_info()
{
  return vehicle_info_utils::createVehicleInfo(0.3, 0.2, 2.5, 1.6, 1.0, 1.0, 0.5, 0.5, 1.8, 0.7);
}

ObstacleTypeParameters make_obstacle_type_parameters(
  const double front = 0.5, const double side = 0.0, const double back = 0.0)
{
  ObstacleTypeParameters parameters;
  parameters.surround_check_front_distance = front;
  parameters.surround_check_side_distance = side;
  parameters.surround_check_back_distance = back;
  return parameters;
}

Parameters make_default_parameters()
{
  Parameters parameters;
  parameters.pointcloud_enable_check = true;
  parameters.object_type_enable_check["car"] = true;
  parameters.object_type_enable_check["pedestrian"] = false;

  const auto obstacle_params = make_obstacle_type_parameters();
  parameters.obstacle_types_map["pointcloud"] = obstacle_params;
  parameters.obstacle_types_map["car"] = obstacle_params;
  parameters.obstacle_types_map["pedestrian"] = obstacle_params;

  return parameters;
}

geometry_msgs::msg::Pose make_pose(const double x, const double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
  return pose;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr make_pointcloud(
  const std::vector<std::array<float, 3>> & points)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  for (const auto & point : points) {
    pcl_cloud.push_back(pcl::PointXYZ(point[0], point[1], point[2]));
  }

  return pcl_cloud.makeShared();
}

PredictedObjects::SharedPtr make_predicted_objects(const std::vector<PredictedObject> & objects)
{
  auto objects_msg = std::make_shared<PredictedObjects>();
  objects_msg->objects = objects;
  return objects_msg;
}

PredictedObject make_car_object(const double x, const double y, const uint8_t id = 1)
{
  PredictedObject object;
  object.object_id.uuid = {id};
  object.classification.resize(1);
  object.classification.front().label = ObjectClassification::CAR;
  object.classification.front().probability = 1.0F;
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 2.0;
  object.shape.dimensions.y = 2.0;
  object.shape.dimensions.z = 1.5;
  object.kinematics.initial_pose_with_covariance.pose = make_pose(x, y);
  return object;
}

Inputs make_inputs(
  const geometry_msgs::msg::Pose & ego_pose,
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud = nullptr,
  const PredictedObjects::ConstSharedPtr & objects = nullptr)
{
  Inputs inputs;
  inputs.ego_pose = ego_pose;
  inputs.pointcloud_in_base_link = pointcloud;
  inputs.objects = objects;
  return inputs;
}

}  // namespace

class ProximityCheckerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    vehicle_info_ = make_vehicle_info();
    parameters_ = make_default_parameters();
    checker_ = std::make_unique<ProximityChecker>(parameters_, vehicle_info_);
    ego_pose_ = make_pose(0.0, 0.0);
  }

  vehicle_info_utils::VehicleInfo vehicle_info_;
  Parameters parameters_;
  std::unique_ptr<ProximityChecker> checker_;
  geometry_msgs::msg::Pose ego_pose_;
};

TEST_F(ProximityCheckerTest, NoObstacleWhenInputsAreEmpty)
{
  const auto result = checker_->check(make_inputs(ego_pose_), 1e-3);
  EXPECT_FALSE(result.is_obstacle_found);
  EXPECT_FALSE(result.nearest_obstacle.has_value());
}

TEST_F(ProximityCheckerTest, NoObstacleWhenPointcloudCheckDisabled)
{
  parameters_.pointcloud_enable_check = false;
  checker_->update_parameters(parameters_);

  const auto pointcloud = make_pointcloud({{0.0F, 0.0F, 0.0F}});
  const auto result = checker_->check(make_inputs(ego_pose_, pointcloud), 1e-3);

  EXPECT_FALSE(result.is_obstacle_found);
  EXPECT_FALSE(result.nearest_obstacle.has_value());
}

TEST_F(ProximityCheckerTest, DetectsNearbyPointInPointcloud)
{
  const auto pointcloud = make_pointcloud({{4.2F, 0.0F, 0.0F}});
  const auto result = checker_->check(make_inputs(ego_pose_, pointcloud), 0.5);

  ASSERT_TRUE(result.nearest_obstacle.has_value());
  EXPECT_TRUE(result.nearest_obstacle->is_point_cloud);
  EXPECT_NEAR(result.nearest_obstacle->nearest_distance, 0.2, 1e-3);
  EXPECT_TRUE(result.is_obstacle_found);
}

TEST_F(ProximityCheckerTest, NoObstacleForFarPointInPointcloud)
{
  const auto pointcloud = make_pointcloud({{20.0F, 0.0F, 0.0F}});
  const auto result = checker_->check(make_inputs(ego_pose_, pointcloud), 1e-3);

  EXPECT_FALSE(result.is_obstacle_found);
}

TEST_F(ProximityCheckerTest, RespectsContactDistanceThreshold)
{
  const auto pointcloud = make_pointcloud({{4.2F, 0.0F, 0.0F}});
  const auto inputs = make_inputs(ego_pose_, pointcloud);

  const auto result_below_threshold = checker_->check(inputs, 0.1);
  EXPECT_FALSE(result_below_threshold.is_obstacle_found);
  ASSERT_TRUE(result_below_threshold.nearest_obstacle.has_value());

  const auto result_above_threshold = checker_->check(inputs, 0.5);
  EXPECT_TRUE(result_above_threshold.is_obstacle_found);
}

TEST_F(ProximityCheckerTest, DetectsNearbyDynamicObject)
{
  const auto objects = make_predicted_objects({make_car_object(5.2, 0.0)});
  const auto result = checker_->check(make_inputs(ego_pose_, nullptr, objects), 0.5);

  ASSERT_TRUE(result.nearest_obstacle.has_value());
  EXPECT_FALSE(result.nearest_obstacle->is_point_cloud);
  EXPECT_NEAR(result.nearest_obstacle->nearest_distance, 0.2, 1e-3);
  EXPECT_TRUE(result.is_obstacle_found);
}

TEST_F(ProximityCheckerTest, IgnoresDisabledObjectTypes)
{
  parameters_.object_type_enable_check["car"] = false;
  checker_->update_parameters(parameters_);

  const auto objects = make_predicted_objects({make_car_object(5.2, 0.0)});
  const auto result = checker_->check(make_inputs(ego_pose_, nullptr, objects), 0.5);

  EXPECT_FALSE(result.is_obstacle_found);
  EXPECT_FALSE(result.nearest_obstacle.has_value());
}

TEST_F(ProximityCheckerTest, SelectsNearestBetweenPointcloudAndObject)
{
  const auto pointcloud = make_pointcloud({{4.5F, 0.0F, 0.0F}});
  const auto objects = make_predicted_objects({make_car_object(5.2, 0.0)});

  const auto result = checker_->check(make_inputs(ego_pose_, pointcloud, objects), 0.5);

  ASSERT_TRUE(result.nearest_obstacle.has_value());
  EXPECT_FALSE(result.nearest_obstacle->is_point_cloud);
  EXPECT_NEAR(result.nearest_obstacle->nearest_distance, 0.2, 1e-3);
}

TEST_F(ProximityCheckerTest, UpdateParametersDisablesPointcloudCheck)
{
  const auto pointcloud = make_pointcloud({{0.0F, 0.0F, 0.0F}});
  const auto inputs = make_inputs(ego_pose_, pointcloud);

  {
    const auto result = checker_->check(inputs, 1e-3);
    EXPECT_TRUE(result.is_obstacle_found);
  }

  parameters_.pointcloud_enable_check = false;
  checker_->update_parameters(parameters_);

  {
    const auto result = checker_->check(inputs, 1e-3);
    EXPECT_FALSE(result.is_obstacle_found);
  }
}

}  // namespace autoware::obstacle_proximity_checker
