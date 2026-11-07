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

#include "autoware/camera_streampetr/ego_pose_bindings.hpp"

#include <tf2/LinearMath/Quaternion.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <vector>

namespace autoware::camera_streampetr
{
namespace
{

tf2::Transform make_pose(const double x, const double y, const double z, const double yaw)
{
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, yaw);
  return tf2::Transform(rotation, tf2::Vector3(x, y, z));
}

tf2::Vector3 apply(const std::vector<float> & matrix, const tf2::Vector3 & point)
{
  tf2::Vector3 result;
  for (int row = 0; row < 3; ++row) {
    result[row] = matrix[row * 4] * point.x() + matrix[row * 4 + 1] * point.y() +
                  matrix[row * 4 + 2] * point.z() + matrix[row * 4 + 3];
  }
  return result;
}

void expect_identity(const std::vector<float> & matrix)
{
  ASSERT_EQ(matrix.size(), 16U);
  for (int index = 0; index < 16; ++index) {
    EXPECT_FLOAT_EQ(matrix[index], index % 5 == 0 ? 1.0f : 0.0f) << "entry " << index;
  }
}

}  // namespace

TEST(EgoPoseBindings, RowMajorLayoutPutsTheTranslationInTheLastColumn)
{
  const std::vector<float> matrix = to_row_major(make_pose(1.0, 2.0, 3.0, M_PI / 2.0));

  ASSERT_EQ(matrix.size(), 16U);
  EXPECT_NEAR(matrix[0], 0.0f, 1e-6);   // cos
  EXPECT_NEAR(matrix[1], -1.0f, 1e-6);  // -sin
  EXPECT_NEAR(matrix[4], 1.0f, 1e-6);   // sin
  EXPECT_FLOAT_EQ(matrix[3], 1.0f);
  EXPECT_FLOAT_EQ(matrix[7], 2.0f);
  EXPECT_FLOAT_EQ(matrix[11], 3.0f);
  EXPECT_FLOAT_EQ(matrix[15], 1.0f);
}

TEST(EgoPoseBindings, StartOfAStreamBindsTheIdentity)
{
  const auto [ego_pose, ego_pose_inv] =
    compute_ego_pose_bindings(std::nullopt, make_pose(89000.0, 42000.0, 40.0, 0.3));

  expect_identity(ego_pose);
  expect_identity(ego_pose_inv);
}

TEST(EgoPoseBindings, MovesAPointOfThePreviousVehicleFrameIntoTheCurrentOne)
{
  // 65 km from the map origin, 1.4 m and 0.02 rad between the frames.
  const tf2::Transform previous = make_pose(65460.0, 675.0, 40.0, 0.30);
  const tf2::Transform current = make_pose(65461.3, 675.5, 40.0, 0.32);
  const tf2::Vector3 point_in_previous(20.0, -3.0, 0.5);

  const auto [ego_pose, ego_pose_inv] = compute_ego_pose_bindings(previous, current);

  expect_identity(ego_pose);
  const tf2::Vector3 expected = current.inverse() * (previous * point_in_previous);
  const tf2::Vector3 actual = apply(ego_pose_inv, point_in_previous);
  EXPECT_NEAR(actual.x(), expected.x(), 1e-5);
  EXPECT_NEAR(actual.y(), expected.y(), 1e-5);
  EXPECT_NEAR(actual.z(), expected.z(), 1e-5);
}

TEST(EgoPoseBindings, BindsTheSameSmallNumbersAtAnyDistanceFromTheMapOrigin)
{
  // The same motion near the map origin and 65 km away: the head sees the same numbers, all
  // within the motion between the frames, which an FP16 engine represents to the millimeter.
  const auto near =
    compute_ego_pose_bindings(make_pose(0.0, 0.0, 0.0, 0.30), make_pose(1.3, 0.5, 0.0, 0.32));
  const auto far = compute_ego_pose_bindings(
    make_pose(65460.0, 675.0, 40.0, 0.30), make_pose(65461.3, 675.5, 40.0, 0.32));

  for (int index = 0; index < 16; ++index) {
    EXPECT_NEAR(near.second[index], far.second[index], 1e-6) << "entry " << index;
  }
  for (const int index : {3, 7, 11}) {
    EXPECT_LT(std::abs(far.second[index]), 2.0f) << "translation entry " << index;
  }
}

}  // namespace autoware::camera_streampetr
