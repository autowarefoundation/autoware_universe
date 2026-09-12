// Copyright 2025 Instituto de Telecomunições-Porto Branch, Inc. All rights reserved.
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

#include "../src/spheric_collision_detector_node/spheric_collision_detector.cpp"  // NOLINT
#include "gtest/gtest.h"

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <vector>

// The functions exercised below live in the anonymous namespace of
// spheric_collision_detector.cpp, which is pulled into this translation unit by the
// include above. Every expected value is an independent, hand-computed oracle; none is
// derived by re-running the function under test.

namespace
{

TEST(SphericCollisionDetectorCore, CalcBrakingDistanceStationary)
{
  // v = 0 -> idling (v * delay) = 0 and braking (v^2 / (2 * decel)) = 0 -> total 0.
  EXPECT_DOUBLE_EQ(calcBrakingDistance(0.0, 2.0, 0.3), 0.0);
}

TEST(SphericCollisionDetectorCore, CalcBrakingDistanceMoving)
{
  // v = 10, decel = 2, delay = 0.5 -> idling = 5.0, braking = 100 / 4 = 25.0 -> total 30.0.
  EXPECT_DOUBLE_EQ(calcBrakingDistance(10.0, 2.0, 0.5), 30.0);
  // v = 4, decel = 8, delay = 0.25 -> idling = 1.0, braking = 16 / 16 = 1.0 -> total 2.0.
  EXPECT_DOUBLE_EQ(calcBrakingDistance(4.0, 8.0, 0.25), 2.0);
}

TEST(SphericCollisionDetectorCore, ComputeLargestDistFootprintSelectsEachAxis)
{
  // d1 = |x_front - x_center|, d2 = |x_center - x_rear|, d3 = |y_left - y_right|.
  // Returns 0.5 * max(d1, d2, d3), with ties resolving to d3.
  spheric_collision_detector::FootprintCoords fc_d1{10.0, 0.0, 1.0, 0.0, 1.0};
  // d1 = 10, d2 = 1, d3 = 1 -> 0.5 * 10 = 5.0.
  EXPECT_DOUBLE_EQ(computeLargestDistFootprint(fc_d1), 5.0);

  spheric_collision_detector::FootprintCoords fc_d2{0.0, 0.0, -8.0, 1.0, 0.0};
  // d1 = 0, d2 = 8, d3 = 1 -> 0.5 * 8 = 4.0.
  EXPECT_DOUBLE_EQ(computeLargestDistFootprint(fc_d2), 4.0);

  spheric_collision_detector::FootprintCoords fc_d3{1.0, 0.0, 0.0, 3.0, -3.0};
  // d1 = 1, d2 = 0, d3 = 6 -> neither branch wins, else -> 0.5 * 6 = 3.0.
  EXPECT_DOUBLE_EQ(computeLargestDistFootprint(fc_d3), 3.0);
}

TEST(SphericCollisionDetectorCore, CreateObstacleSpheresSingleObject)
{
  // One object at the local origin (identity orientation), transformed by a pure
  // translation (10, 5, 2). dim_x (length) = 2.0, dim_y (width) = 1.0.
  // sphere_radius = dim_y * 0.5 = 0.5, x_front = 0.35 * dim_x = 0.7, x_rear = -0.7.
  // Local centre x offsets: {0.7, 0.35, 0, -0.35, -0.7}; the translation shifts them to
  // x + 10, y + 5, z = 2 (the transformed pose's z).
  autoware_perception_msgs::msg::DetectedObjects objects;
  autoware_perception_msgs::msg::DetectedObject obj;
  obj.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
  obj.shape.dimensions.x = 2.0;
  obj.shape.dimensions.y = 1.0;
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = 1;
  obj.classification.push_back(classification);
  objects.objects.push_back(obj);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform.rotation.w = 1.0;
  transform.transform.translation.x = 10.0;
  transform.transform.translation.y = 5.0;
  transform.transform.translation.z = 2.0;

  std::vector<autoware_utils::LinearRing2d> object_area;
  const auto obstacles = createObstacleSpheres(objects, transform, object_area);

  ASSERT_EQ(obstacles.size(), 1u);
  ASSERT_EQ(obstacles.front().size(), 5u);

  const std::vector<double> expected_x{10.7, 10.35, 10.0, 9.65, 9.3};
  for (size_t i = 0; i < expected_x.size(); ++i) {
    const auto & sphere = obstacles.front().at(i);
    EXPECT_NEAR(sphere->center_.x(), expected_x.at(i), 1e-6);
    EXPECT_NEAR(sphere->center_.y(), 5.0, 1e-6);
    EXPECT_NEAR(sphere->center_.z(), 2.0, 1e-6);
    EXPECT_NEAR(sphere->radius_, 0.5, 1e-6);
    EXPECT_EQ(sphere->tag_, 1);
  }

  ASSERT_EQ(object_area.size(), 1u);
  ASSERT_EQ(object_area.front().size(), 5u);
  for (size_t i = 0; i < expected_x.size(); ++i) {
    EXPECT_NEAR(object_area.front().at(i).x(), expected_x.at(i), 1e-6);
    EXPECT_NEAR(object_area.front().at(i).y(), 5.0, 1e-6);
  }
}

TEST(SphericCollisionDetectorCore, CreateObstacleSpheresEmptyInput)
{
  // No objects -> the loop body never runs; both outputs stay empty and nothing crashes.
  autoware_perception_msgs::msg::DetectedObjects objects;
  geometry_msgs::msg::TransformStamped transform;
  transform.transform.rotation.w = 1.0;

  std::vector<autoware_utils::LinearRing2d> object_area;
  const auto obstacles = createObstacleSpheres(objects, transform, object_area);

  EXPECT_TRUE(obstacles.empty());
  EXPECT_TRUE(object_area.empty());
}

}  // namespace
