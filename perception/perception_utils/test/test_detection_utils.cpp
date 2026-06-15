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

#include "perception_utils/detection_class_remapper.hpp"
#include "perception_utils/iou_bev_nms.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace
{

using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
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

TEST(DetectionClassRemapperTest, RemapsEveryClassificationByBevArea)
{
  perception_utils::DetectionClassRemapper remapper;
  const std::vector<std::int64_t> allow{0, 0, 0,  //
                                        0, 0, 1,  //
                                        0, 1, 0};
  const std::vector<double> min_area{0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0};
  const std::vector<double> max_area{0.0, 0.0, 0.0, 0.0, 0.0, 999.0, 0.0, 10.0, 0.0};
  remapper.setParameters(allow, min_area, max_area);

  DetectedObjects objects;
  objects.objects = {
    make_object(0.0, 0.0, 2.0F, 2.0F, 0, 1.0F), make_object(0.0, 0.0, 8.0F, 2.0F, 1, 1.0F),
    make_object(0.0, 0.0, 3.0F, 3.0F, 2, 1.0F), make_object(0.0, 0.0, 4.0F, 3.0F, 2, 1.0F)};

  remapper.mapClasses(objects);

  EXPECT_EQ(objects.objects.at(0).classification.at(0).label, 0);
  EXPECT_EQ(objects.objects.at(1).classification.at(0).label, 2);
  EXPECT_EQ(objects.objects.at(2).classification.at(0).label, 1);
  EXPECT_EQ(objects.objects.at(3).classification.at(0).label, 2);
}

TEST(DetectionClassRemapperTest, RejectsInvalidMatrixDimensions)
{
  perception_utils::DetectionClassRemapper remapper;
  EXPECT_THROW(remapper.setParameters({0, 0}, {0.0, 0.0}, {0.0, 0.0}), std::invalid_argument);
}

}  // namespace
