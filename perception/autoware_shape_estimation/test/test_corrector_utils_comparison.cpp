// Copyright 2024 TierIV. All rights reserved.
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

#include "autoware/shape_estimation/corrector/utils.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <cmath>
#include <vector>

namespace autoware::shape_estimation::corrector_utils
{

class CorrectorUtilsComparisonTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Car default parameters
    default_param_.min_width = 1.2;
    default_param_.max_width = 2.5;
    default_param_.default_width =
      (default_param_.min_width + default_param_.max_width) * 0.5;  // 1.85
    default_param_.min_length = 3.0;
    default_param_.max_length = 5.8;
    default_param_.default_length =
      (default_param_.min_length + default_param_.max_length) * 0.5;  // 4.4
  }

  // Helper function to create a shape with given dimensions
  autoware_perception_msgs::msg::Shape createShape(double x, double y, double z = 1.0)
  {
    autoware_perception_msgs::msg::Shape shape;
    shape.dimensions.x = x;
    shape.dimensions.y = y;
    shape.dimensions.z = z;
    return shape;
  }

  // Helper function to create a pose at given position and yaw
  geometry_msgs::msg::Pose createPose(double x = 0.0, double y = 0.0, double yaw = 0.0)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    pose.orientation = autoware_utils::create_quaternion_from_rpy(0.0, 0.0, yaw);
    return pose;
  }

  // Helper function to compare two shapes with tolerance
  bool shapesEqual(
    const autoware_perception_msgs::msg::Shape & s1,
    const autoware_perception_msgs::msg::Shape & s2, double tolerance = 1e-6)
  {
    return std::abs(s1.dimensions.x - s2.dimensions.x) < tolerance &&
           std::abs(s1.dimensions.y - s2.dimensions.y) < tolerance &&
           std::abs(s1.dimensions.z - s2.dimensions.z) < tolerance;
  }

  // Helper function to compare two poses with tolerance
  bool posesEqual(
    const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2,
    double tolerance = 1e-6)
  {
    return std::abs(p1.position.x - p2.position.x) < tolerance &&
           std::abs(p1.position.y - p2.position.y) < tolerance &&
           std::abs(p1.position.z - p2.position.z) < tolerance &&
           std::abs(p1.orientation.x - p2.orientation.x) < tolerance &&
           std::abs(p1.orientation.y - p2.orientation.y) < tolerance &&
           std::abs(p1.orientation.z - p2.orientation.z) < tolerance &&
           std::abs(p1.orientation.w - p2.orientation.w) < tolerance;
  }

  CorrectionBBParameters default_param_;
};

// Test comparison with basic case
TEST_F(CorrectorUtilsComparisonTest, BasicComparison)
{
  auto shape_before = createShape(2.0, 1.5);
  auto pose_before = createPose(0.0, 0.0, 0.0);
  auto param = default_param_;

  auto shape_after = shape_before;
  auto pose_after = pose_before;

  bool result_before = correctWithDefaultValueBefore(param, shape_before, pose_before);
  bool result_after = correctWithDefaultValue(param, shape_after, pose_after);

  EXPECT_EQ(result_before, result_after);
  if (result_before && result_after) {
    EXPECT_TRUE(shapesEqual(shape_before, shape_after));
    EXPECT_TRUE(posesEqual(pose_before, pose_after));
  }
}

// Test comparison with Case 4 real log parameters
TEST_F(CorrectorUtilsComparisonTest, Case4Comparison)
{
  auto shape_before = createShape(0.857224, 2.44392);
  auto pose_before = createPose(35.687, -3.47156, 1.3439);
  auto param = default_param_;

  auto shape_after = shape_before;
  auto pose_after = pose_before;

  bool result_before = correctWithDefaultValueBefore(param, shape_before, pose_before);
  bool result_after = correctWithDefaultValue(param, shape_after, pose_after);

  EXPECT_EQ(result_before, result_after);
  if (result_before && result_after) {
    EXPECT_TRUE(shapesEqual(shape_before, shape_after));
    EXPECT_TRUE(posesEqual(pose_before, pose_after));
  }
}

// Test comparison with Case 5 real log parameters
TEST_F(CorrectorUtilsComparisonTest, Case5Comparison)
{
  auto shape_before = createShape(1.97749, 3.98609);
  auto pose_before = createPose(-11.1428, -3.12857, 1.55334);
  auto param = default_param_;

  auto shape_after = shape_before;
  auto pose_after = pose_before;

  bool result_before = correctWithDefaultValueBefore(param, shape_before, pose_before);
  bool result_after = correctWithDefaultValue(param, shape_after, pose_after);

  EXPECT_EQ(result_before, result_after);
  if (result_before && result_after) {
    EXPECT_TRUE(shapesEqual(shape_before, shape_after));
    EXPECT_TRUE(posesEqual(pose_before, pose_after));
  }
}

// Test comparison with Case 6 real log parameters
TEST_F(CorrectorUtilsComparisonTest, Case6Comparison)
{
  auto shape_before = createShape(0.944625, 3.47609);
  auto pose_before = createPose(-20.6513, -35.3065, 0.0523599);
  auto param = default_param_;

  auto shape_after = shape_before;
  auto pose_after = pose_before;

  bool result_before = correctWithDefaultValueBefore(param, shape_before, pose_before);
  bool result_after = correctWithDefaultValue(param, shape_after, pose_after);

  EXPECT_EQ(result_before, result_after);
  if (result_before && result_after) {
    EXPECT_TRUE(shapesEqual(shape_before, shape_after));
    EXPECT_TRUE(posesEqual(pose_before, pose_after));
  }
}

// Test comparison with failure case
TEST_F(CorrectorUtilsComparisonTest, FailureCaseComparison)
{
  auto shape_before = createShape(0.54676, 1.12872);
  auto pose_before = createPose(59.4785, -11.7461, 0.10472);
  auto param = default_param_;

  auto shape_after = shape_before;
  auto pose_after = pose_before;

  bool result_before = correctWithDefaultValueBefore(param, shape_before, pose_before);
  bool result_after = correctWithDefaultValue(param, shape_after, pose_after);

  EXPECT_EQ(result_before, result_after);
  EXPECT_FALSE(result_before);
  EXPECT_FALSE(result_after);

  // Both functions should return false and leave shapes/poses unchanged
  EXPECT_TRUE(shapesEqual(shape_before, shape_after));
  EXPECT_TRUE(posesEqual(pose_before, pose_after));
}

// Test comparison with opposite edges case
TEST_F(CorrectorUtilsComparisonTest, OppositeEdgesComparison)
{
  auto shape_before = createShape(3.0, 1.0);
  auto pose_before = createPose(0.0, 0.0, 0.0);
  auto param = default_param_;

  auto shape_after = shape_before;
  auto pose_after = pose_before;

  bool result_before = correctWithDefaultValueBefore(param, shape_before, pose_before);
  bool result_after = correctWithDefaultValue(param, shape_after, pose_after);

  EXPECT_EQ(result_before, result_after);
  if (result_before && result_after) {
    EXPECT_TRUE(shapesEqual(shape_before, shape_after));
    EXPECT_TRUE(posesEqual(pose_before, pose_after));
  }
}

// Test comparison with adjacent edges case
TEST_F(CorrectorUtilsComparisonTest, AdjacentEdgesComparison)
{
  auto shape_before = createShape(3.0, 3.0);
  auto pose_before = createPose(0.0, 0.0, M_PI / 4);
  auto param = default_param_;

  auto shape_after = shape_before;
  auto pose_after = pose_before;

  bool result_before = correctWithDefaultValueBefore(param, shape_before, pose_before);
  bool result_after = correctWithDefaultValue(param, shape_after, pose_after);

  EXPECT_EQ(result_before, result_after);
  if (result_before && result_after) {
    EXPECT_TRUE(shapesEqual(shape_before, shape_after));
    EXPECT_TRUE(posesEqual(pose_before, pose_after));
  }
}

// Test comparison with dimension swapping case
TEST_F(CorrectorUtilsComparisonTest, DimensionSwappingComparison)
{
  auto shape_before = createShape(2.0, 1.0);
  auto pose_before = createPose(0.0, 0.0, 0.0);
  auto param = default_param_;

  auto shape_after = shape_before;
  auto pose_after = pose_before;

  bool result_before = correctWithDefaultValueBefore(param, shape_before, pose_before);
  bool result_after = correctWithDefaultValue(param, shape_after, pose_after);

  EXPECT_EQ(result_before, result_after);
  if (result_before && result_after) {
    EXPECT_TRUE(shapesEqual(shape_before, shape_after));
    EXPECT_TRUE(posesEqual(pose_before, pose_after));
  }
}

// Test comparison with various rotation angles
TEST_F(CorrectorUtilsComparisonTest, VariousRotationAnglesComparison)
{
  auto param = default_param_;
  std::vector<double> test_angles = {0.0,      M_PI / 6, M_PI / 4,    M_PI / 3,
                                     M_PI / 2, M_PI,     3 * M_PI / 2};

  for (double angle : test_angles) {
    auto shape_before = createShape(2.0, 1.5);
    auto pose_before = createPose(0.0, 0.0, angle);

    auto shape_after = shape_before;
    auto pose_after = pose_before;

    bool result_before = correctWithDefaultValueBefore(param, shape_before, pose_before);
    bool result_after = correctWithDefaultValue(param, shape_after, pose_after);

    EXPECT_EQ(result_before, result_after) << "Failed at angle: " << angle;
    if (result_before && result_after) {
      EXPECT_TRUE(shapesEqual(shape_before, shape_after)) << "Shape mismatch at angle: " << angle;
      EXPECT_TRUE(posesEqual(pose_before, pose_after)) << "Pose mismatch at angle: " << angle;
    }
  }
}

// Test comparison with edge cases
TEST_F(CorrectorUtilsComparisonTest, EdgeCasesComparison)
{
  auto param = default_param_;

  // Test zero dimensions
  {
    auto shape_before = createShape(0.0, 0.0);
    auto pose_before = createPose(0.0, 0.0, 0.0);

    auto shape_after = shape_before;
    auto pose_after = pose_before;

    bool result_before = correctWithDefaultValueBefore(param, shape_before, pose_before);
    bool result_after = correctWithDefaultValue(param, shape_after, pose_after);

    EXPECT_EQ(result_before, result_after);
  }

  // Test large dimensions
  {
    auto shape_before = createShape(10.0, 10.0);
    auto pose_before = createPose(0.0, 0.0, 0.0);

    auto shape_after = shape_before;
    auto pose_after = pose_before;

    bool result_before = correctWithDefaultValueBefore(param, shape_before, pose_before);
    bool result_after = correctWithDefaultValue(param, shape_after, pose_after);

    EXPECT_EQ(result_before, result_after);
    if (result_before && result_after) {
      EXPECT_TRUE(shapesEqual(shape_before, shape_after));
      EXPECT_TRUE(posesEqual(pose_before, pose_after));
    }
  }

  // Test large pose translation
  {
    auto shape_before = createShape(2.0, 1.5);
    auto pose_before = createPose(100.0, 200.0, 0.0);

    auto shape_after = shape_before;
    auto pose_after = pose_before;

    bool result_before = correctWithDefaultValueBefore(param, shape_before, pose_before);
    bool result_after = correctWithDefaultValue(param, shape_after, pose_after);

    EXPECT_EQ(result_before, result_after);
    if (result_before && result_after) {
      EXPECT_TRUE(shapesEqual(shape_before, shape_after));
      EXPECT_TRUE(posesEqual(pose_before, pose_after));
    }
  }
}

// Comprehensive test with many random cases
TEST_F(CorrectorUtilsComparisonTest, ComprehensiveRandomComparison)
{
  auto param = default_param_;

  // Test data: various realistic car-like dimensions and poses
  std::vector<std::tuple<double, double, double, double, double>> test_cases = {
    {1.0, 2.0, 0.0, 0.0, 0.0},
    {1.5, 3.0, 10.0, -5.0, 0.5},
    {2.0, 4.0, -10.0, 15.0, 1.0},
    {0.8, 1.8, 50.0, -20.0, 1.5},
    {1.8, 4.5, -30.0, 40.0, 2.0},
    {2.2, 3.5, 0.0, 0.0, 2.5},
    {1.2, 2.8, 25.0, -35.0, 3.0},
    {0.9, 1.5, -15.0, 25.0, -0.5},
    {1.7, 3.8, 35.0, -10.0, -1.0},
    {2.5, 5.0, -40.0, 30.0, 0.25},
    // Add some boundary cases
    {1.2, 3.0, 0.0, 0.0, 0.0},   // min_width, min_length
    {2.5, 5.8, 0.0, 0.0, 0.0},   // max_width, max_length
    {1.85, 4.4, 0.0, 0.0, 0.0},  // default_width, default_length
  };

  for (const auto & test_case : test_cases) {
    auto [dim_x, dim_y, pos_x, pos_y, yaw] = test_case;

    auto shape_before = createShape(dim_x, dim_y);
    auto pose_before = createPose(pos_x, pos_y, yaw);

    auto shape_after = shape_before;
    auto pose_after = pose_before;

    bool result_before = correctWithDefaultValueBefore(param, shape_before, pose_before);
    bool result_after = correctWithDefaultValue(param, shape_after, pose_after);

    EXPECT_EQ(result_before, result_after)
      << "Result mismatch for case: (" << dim_x << ", " << dim_y << ", " << pos_x << ", " << pos_y
      << ", " << yaw << ")";

    if (result_before && result_after) {
      EXPECT_TRUE(shapesEqual(shape_before, shape_after))
        << "Shape mismatch for case: (" << dim_x << ", " << dim_y << ", " << pos_x << ", " << pos_y
        << ", " << yaw << ")";
      EXPECT_TRUE(posesEqual(pose_before, pose_after))
        << "Pose mismatch for case: (" << dim_x << ", " << dim_y << ", " << pos_x << ", " << pos_y
        << ", " << yaw << ")";
    }
  }
}

}  // namespace autoware::shape_estimation::corrector_utils
