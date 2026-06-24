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

#include "autoware/pointcloud_preprocessor/utility/conversion.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
using autoware::pointcloud_preprocessor::utils::to_eigen_matrix;
using autoware::pointcloud_preprocessor::utils::to_nanoseconds;
using autoware::pointcloud_preprocessor::utils::to_seconds;
using autoware::pointcloud_preprocessor::utils::to_tf2_transform;

geometry_msgs::msg::Transform make_transform(
  double tx, double ty, double tz, double qx, double qy, double qz, double qw)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = tx;
  transform.translation.y = ty;
  transform.translation.z = tz;
  transform.rotation.x = qx;
  transform.rotation.y = qy;
  transform.rotation.z = qz;
  transform.rotation.w = qw;
  return transform;
}
}  // namespace

TEST(ConversionTest, ToNanoseconds)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 10;
  stamp.nanosec = 100000000;
  EXPECT_EQ(to_nanoseconds(stamp), 10'100'000'000LL);

  builtin_interfaces::msg::Time zero;
  EXPECT_EQ(to_nanoseconds(zero), 0);
}

TEST(ConversionTest, ToSeconds)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 10;
  stamp.nanosec = 100000000;
  EXPECT_NEAR(to_seconds(stamp), 10.1, 1e-9);
}

TEST(ConversionTest, ToTf2Transform)
{
  // Translation (1, 2, 3) and a +90 deg rotation about the z-axis.
  const double s = std::sqrt(0.5);
  const tf2::Transform tf = to_tf2_transform(make_transform(1.0, 2.0, 3.0, 0.0, 0.0, s, s));

  EXPECT_NEAR(tf.getOrigin().x(), 1.0, 1e-9);
  EXPECT_NEAR(tf.getOrigin().y(), 2.0, 1e-9);
  EXPECT_NEAR(tf.getOrigin().z(), 3.0, 1e-9);

  // The rotation alone maps the x-axis onto the y-axis.
  const tf2::Vector3 rotated = tf.getBasis() * tf2::Vector3(1.0, 0.0, 0.0);
  EXPECT_NEAR(rotated.x(), 0.0, 1e-6);
  EXPECT_NEAR(rotated.y(), 1.0, 1e-6);
  EXPECT_NEAR(rotated.z(), 0.0, 1e-6);
}

TEST(ConversionTest, ToEigenMatrix)
{
  // Translation (1, 2, 3) and a +90 deg rotation about the z-axis.
  const double s = std::sqrt(0.5);
  const Eigen::Matrix4f matrix = to_eigen_matrix(make_transform(1.0, 2.0, 3.0, 0.0, 0.0, s, s));

  EXPECT_NEAR(matrix(0, 3), 1.0f, 1e-6f);
  EXPECT_NEAR(matrix(1, 3), 2.0f, 1e-6f);
  EXPECT_NEAR(matrix(2, 3), 3.0f, 1e-6f);

  // (1, 0, 0) rotated by +90 deg about z is (0, 1, 0), then translated by (1, 2, 3) -> (1, 3, 3).
  const Eigen::Vector4f transformed = matrix * Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f);
  EXPECT_NEAR(transformed.x(), 1.0f, 1e-6f);
  EXPECT_NEAR(transformed.y(), 3.0f, 1e-6f);
  EXPECT_NEAR(transformed.z(), 3.0f, 1e-6f);
}
