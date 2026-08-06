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

#include "autoware/ptv3/ros_utils.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace autoware::ptv3
{
namespace test
{
using autoware_perception_msgs::msg::ObjectClassification;

TEST(RosUtilsTest, MapsDetectionClassNames)
{
  EXPECT_EQ(get_classification_type("CAR"), ObjectClassification::CAR);
  EXPECT_EQ(get_classification_type("MOTORBIKE"), ObjectClassification::MOTORCYCLE);
  EXPECT_EQ(get_classification_type("TRAFFIC_CONE"), ObjectClassification::HAZARD);
  EXPECT_EQ(get_classification_type("UNKNOWN_CLASS"), ObjectClassification::UNKNOWN);
}

TEST(RosUtilsTest, ConvertsBox3DToDetectedObject)
{
  constexpr float pi = 3.14159265358979323846F;
  const Box3D box{1, 0.75F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 0.5F * pi, 1.0F, 2.0F};
  autoware_perception_msgs::msg::DetectedObject object;

  box3d_to_detected_object(box, {"CAR", "PEDESTRIAN"}, true, object);

  ASSERT_EQ(object.classification.size(), 1U);
  EXPECT_EQ(object.classification.front().label, ObjectClassification::PEDESTRIAN);
  EXPECT_FLOAT_EQ(object.existence_probability, 0.75F);
  EXPECT_FLOAT_EQ(object.kinematics.pose_with_covariance.pose.position.x, 1.0F);
  EXPECT_FLOAT_EQ(object.shape.dimensions.x, 4.0F);
  EXPECT_TRUE(object.kinematics.has_twist);
  EXPECT_NEAR(object.kinematics.twist_with_covariance.twist.linear.x, 2.0F, 1e-5F);
  EXPECT_NEAR(object.kinematics.twist_with_covariance.twist.linear.y, -1.0F, 1e-5F);
}

TEST(RosUtilsTest, ConvertsUnknownBoxLabelWithoutTwist)
{
  const Box3D box{99, 0.5F, 0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 0.0F, 1.0F, 1.0F};
  autoware_perception_msgs::msg::DetectedObject object;

  box3d_to_detected_object(box, {"CAR"}, false, object);

  ASSERT_EQ(object.classification.size(), 1U);
  EXPECT_EQ(object.classification.front().label, ObjectClassification::UNKNOWN);
  EXPECT_FALSE(object.kinematics.has_twist);
}

}  // namespace test
}  // namespace autoware::ptv3
