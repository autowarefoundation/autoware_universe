// Copyright 2024 TIER IV, Inc.
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

#include "autoware/detected_object_feature_remover/convert.hpp"

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <tier4_perception_msgs/msg/detected_object_with_feature.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace
{
using autoware::detected_object_feature_remover::convert::ConvertParams;
using autoware::detected_object_feature_remover::convert::convertToDetectedObjects;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;

DetectedObject generate_detected_object()
{
  DetectedObject output;
  output.kinematics.pose_with_covariance.pose.position.x = 1.0;
  output.kinematics.pose_with_covariance.pose.position.y = 2.0;
  output.kinematics.pose_with_covariance.pose.position.z = 3.0;
  output.kinematics.pose_with_covariance.pose.orientation.x = 0.0;
  output.kinematics.pose_with_covariance.pose.orientation.y = 0.0;
  output.kinematics.pose_with_covariance.pose.orientation.z = 0.0;
  output.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
  return output;
}

DetectedObjectWithFeature generate_feature_object()
{
  DetectedObjectWithFeature output;
  output.object = generate_detected_object();
  return output;
}

DetectedObjectsWithFeature generate_feature_objects(bool as_empty)
{
  DetectedObjectsWithFeature output;
  output.header.frame_id = "base_link";
  if (!as_empty) {
    auto object = generate_feature_object();
    output.feature_objects.emplace_back(object);
  }
  return output;
}
}  // namespace

TEST(ConvertTest, TestArbitraryObject)
{
  ConvertParams params;
  params.run_convex_hull_conversion = false;

  constexpr bool as_empty = false;
  const auto input = generate_feature_objects(as_empty);
  DetectedObjects output;

  convertToDetectedObjects(input, output, params);

  auto expect = generate_detected_object();
  EXPECT_EQ(1U, output.objects.size());
  EXPECT_EQ(expect, output.objects.front());
}

TEST(ConvertTest, TestEmptyObject)
{
  ConvertParams params;
  params.run_convex_hull_conversion = false;

  constexpr bool as_empty = true;
  const auto input = generate_feature_objects(as_empty);
  DetectedObjects output;

  convertToDetectedObjects(input, output, params);

  EXPECT_EQ(0U, output.objects.size());
}
