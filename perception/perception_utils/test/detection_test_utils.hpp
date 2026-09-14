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

#ifndef DETECTION_TEST_UTILS_HPP_
#define DETECTION_TEST_UTILS_HPP_

#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>

#include <cstdint>

namespace perception_utils::test
{

inline autoware_perception_msgs::msg::DetectedObject make_object(
  const double x, const double y, const float length, const float width, const std::uint8_t label,
  const float score)
{
  using autoware_perception_msgs::msg::DetectedObject;
  using autoware_perception_msgs::msg::ObjectClassification;
  using autoware_perception_msgs::msg::Shape;

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

}  // namespace perception_utils::test

#endif  // DETECTION_TEST_UTILS_HPP_
