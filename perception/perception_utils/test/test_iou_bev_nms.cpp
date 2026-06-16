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

#include "perception_utils/iou_bev_nms.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{

using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;

DetectedObject make_object(
  const double x, const double y, const float length, const float width, const std::uint8_t label,
  const float score)
{
  DetectedObject object;
  object.existence_probability = score;
  object.classification.push_back(ObjectClassification{}.set__label(label).set__probability(1.0F));
  object.kinematics.pose_with_covariance.pose.position =
    autoware_utils_geometry::create_point(x, y, 0.0);
  object.kinematics.pose_with_covariance.pose.orientation =
    autoware_utils_geometry::create_quaternion_from_yaw(0.0);
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions = autoware_utils_geometry::create_translation(length, width, 1.0);
  return object;
}

TEST(IouBevNmsTest, PreservesLegacySuppressionBehavior)
{
  perception_utils::IouBevNms nms;
  nms.setParameters({10.0, 0.2});

  const std::vector<DetectedObject> objects{
    make_object(0.0, 0.0, 2.0F, 2.0F, ObjectClassification::CAR, 0.9F),
    make_object(1.0, 0.0, 2.0F, 2.0F, ObjectClassification::CAR, 0.8F),
    make_object(2.0, 0.0, 2.0F, 2.0F, ObjectClassification::CAR, 0.7F)};

  const auto output = nms.apply(objects);
  ASSERT_EQ(output.size(), 1U);
  EXPECT_FLOAT_EQ(output.front().existence_probability, 0.9F);
}

TEST(IouBevNmsTest, SortsByExistenceProbabilityOnlyWhenRequested)
{
  perception_utils::IouBevNms nms;
  nms.setParameters({10.0, 0.2});

  const std::vector<DetectedObject> objects{
    make_object(0.0, 0.0, 4.0F, 2.0F, ObjectClassification::CAR, 0.2F),
    make_object(0.2, 0.0, 4.0F, 2.0F, ObjectClassification::CAR, 0.9F)};

  const auto unsorted_output = nms.apply(objects);
  ASSERT_EQ(unsorted_output.size(), 1U);
  EXPECT_FLOAT_EQ(unsorted_output.front().existence_probability, 0.2F);

  const auto sorted_output = nms.apply(objects, true);
  ASSERT_EQ(sorted_output.size(), 1U);
  EXPECT_FLOAT_EQ(sorted_output.front().existence_probability, 0.9F);
}

TEST(IouBevNmsTest, DoesNotSuppressPedestrianAgainstAnotherClass)
{
  perception_utils::IouBevNms nms;
  nms.setParameters({10.0, 0.0});

  const std::vector<DetectedObject> objects{
    make_object(0.0, 0.0, 2.0F, 2.0F, ObjectClassification::CAR, 0.9F),
    make_object(0.0, 0.0, 2.0F, 2.0F, ObjectClassification::PEDESTRIAN, 0.8F)};

  EXPECT_EQ(nms.apply(objects).size(), 2U);
}

TEST(IouBevNmsTest, SuppressesOverlappingSameClassPedestrians)
{
  perception_utils::IouBevNms nms;
  nms.setParameters({10.0, 0.0});

  // Pedestrians are exempt only from cross-class suppression; overlapping
  // same-class pedestrians are still reduced to the highest-scoring one.
  const std::vector<DetectedObject> objects{
    make_object(0.0, 0.0, 2.0F, 2.0F, ObjectClassification::PEDESTRIAN, 0.9F),
    make_object(0.1, 0.0, 2.0F, 2.0F, ObjectClassification::PEDESTRIAN, 0.8F),
    make_object(0.2, 0.0, 2.0F, 2.0F, ObjectClassification::PEDESTRIAN, 0.7F)};

  EXPECT_EQ(nms.apply(objects).size(), 1U);
}

TEST(IouBevNmsTest, RejectsNonFiniteParameters)
{
  perception_utils::IouBevNms nms;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(nms.setParameters({nan, 0.2}), std::invalid_argument);
  EXPECT_THROW(nms.setParameters({10.0, nan}), std::invalid_argument);
}

}  // namespace
