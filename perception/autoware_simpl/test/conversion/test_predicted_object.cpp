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

#include "autoware/simpl/archetype/prediction.hpp"
#include "autoware/simpl/conversion/predicted_object.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>
#include <rclcpp/duration.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace autoware::simpl
{
namespace
{
std::vector<autoware_perception_msgs::msg::ObjectClassification> construct_classification()
{
  std::vector<autoware_perception_msgs::msg::ObjectClassification> classifications;
  autoware_perception_msgs::msg::ObjectClassification class1;
  class1.probability = 0.5;
  class1.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  classifications.emplace_back(class1);

  autoware_perception_msgs::msg::ObjectClassification class2;
  class2.probability = 1.0;
  class2.label = autoware_perception_msgs::msg::ObjectClassification::BUS;
  classifications.emplace_back(class2);

  return classifications;
}

autoware_perception_msgs::msg::Shape construct_shape()
{
  autoware_perception_msgs::msg::Shape shape;
  shape.dimensions.x = 1.0;
  shape.dimensions.y = 1.0;
  shape.dimensions.z = 1.0;
  shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  return shape;
}

autoware_perception_msgs::msg::TrackedObject construct_tracked_object()
{
  autoware_perception_msgs::msg::TrackedObject object;

  // object id
  object.object_id = autoware_utils::generate_default_uuid();

  // existence probability
  object.existence_probability = 1.0;

  // shape
  object.shape = construct_shape();

  // classification
  object.classification = construct_classification();

  // position
  object.kinematics.pose_with_covariance.pose.position.x = 1.0;
  object.kinematics.pose_with_covariance.pose.position.y = 1.0;
  object.kinematics.pose_with_covariance.pose.position.z = 1.0;

  // orientation
  object.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
  object.kinematics.pose_with_covariance.pose.orientation.x = 0.0;
  object.kinematics.pose_with_covariance.pose.orientation.y = 0.0;
  object.kinematics.pose_with_covariance.pose.orientation.z = 0.0;

  // velocity
  object.kinematics.twist_with_covariance.twist.linear.x = 1.0;
  object.kinematics.twist_with_covariance.twist.linear.y = 1.0;
  object.kinematics.twist_with_covariance.twist.linear.z = 1.0;

  return object;
}
}  // namespace

TEST(TestPredictedObject, testToPredictedMode)
{
  constexpr double confidence = 1.0;
  const std::vector<archetype::PredictedState> waypoints{
    {1.0, 1.0, 1.0, 1.0}, {2.0, 2.0, 2.0, 2.0}, {3.0, 3.0, 3.0, 3.0}};
  const auto result =
    conversion::to_predicted_mode(confidence, waypoints, construct_tracked_object());

  EXPECT_EQ(result.confidence, confidence);
  EXPECT_EQ(result.time_step, rclcpp::Duration::from_seconds(0.1));
  EXPECT_EQ(result.path.size(), 3);
}

TEST(TestPredictedObject, testFromTrackedObject)
{
  const auto result = conversion::from_tracked_object(construct_tracked_object());

  EXPECT_DOUBLE_EQ(result.existence_probability, 1.0);
  EXPECT_EQ(result.shape, construct_shape());
}

TEST(TestPredictedObject, testToPredictedPose)
{
  const archetype::PredictedState predicted_state(1.0, 1.0, 1.0, 1.0);
  const auto result = conversion::to_predicted_pose(predicted_state, construct_tracked_object());

  EXPECT_DOUBLE_EQ(result.position.x, 1.0);
  EXPECT_DOUBLE_EQ(result.position.y, 1.0);
  EXPECT_DOUBLE_EQ(result.position.z, 1.0);
  EXPECT_DOUBLE_EQ(result.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(result.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(result.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(result.orientation.z, 0.0);
}
}  // namespace autoware::simpl
