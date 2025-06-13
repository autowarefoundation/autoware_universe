// Copyright 2022 Tier IV, Inc. All rights reserved.
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

#include "autoware/dummy_perception_publisher/signed_distance_function.hpp"

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <limits>
#include <memory>

namespace autoware::dummy_perception_publisher
{

TEST(SignedDistanceFunctionTest, BoxSDF)
{
  const double eps = 1e-5;

  {
    // test with identity transform
    const auto q = tf2::Quaternion(tf2::Vector3(0, 0, 1.0), 0.0);
    const auto tf_global2local = tf2::Transform(q);
    const auto func = BoxSDF(1., 2., tf_global2local);
    ASSERT_NEAR(func(0.0, 0.0), -0.5, eps);
    ASSERT_NEAR(func(0.0, 1.0), 0.0, eps);
    ASSERT_NEAR(func(0.0, 1.5), 0.5, eps);
    ASSERT_NEAR(func(0.5, 0.0), 0.0, eps);
    ASSERT_NEAR(func(1.0, 0.0), 0.5, eps);
    ASSERT_NEAR(func(1.0, 2.0), sqrt(0.5 * 0.5 + 1.0 * 1.0), eps);

    ASSERT_NEAR(func.getSphereTracingDist(2.0, 0.0, M_PI * -1.0), 1.5, eps);
    ASSERT_NEAR(func.getSphereTracingDist(1.0, 1.5, M_PI * 1.25), sqrt(2.0) * 0.5, eps);
    ASSERT_TRUE(std::isinf(func.getSphereTracingDist(2.0, 0.0, 0.0)));
  }

  {
    // test with rotation (90 deg) and translation
    const auto q = tf2::Quaternion(tf2::Vector3(0, 0, 1.0), M_PI * 0.5);
    const auto tf_global2local = tf2::Transform(q, tf2::Vector3(1.0, 1.0, 0.0));
    const auto func = BoxSDF(1., 2., tf_global2local);
    ASSERT_NEAR(func(1.0, 1.0), -0.5, eps);
    ASSERT_NEAR(func(0.0, 0.0), 0.5, eps);

    // ASSERT_NEAR(func.getSphereTracingDist(0.0, 0.0, M_PI * 0.25, eps), sqrt(2.0) * 0.5, eps);
  }
}

TEST(SignedDistanceFunctionTest, CompositeSDF)
{
  const double eps = 1e-5;
  const auto q_identity = tf2::Quaternion(tf2::Vector3(0, 0, 1.0), 0.0);
  const auto f1 =
    std::make_shared<BoxSDF>(1., 1., tf2::Transform(q_identity, tf2::Vector3(0, 0, 0)));
  const auto f2 =
    std::make_shared<BoxSDF>(1., 1., tf2::Transform(q_identity, tf2::Vector3(0, 2.0, 0)));
  const auto func =
    CompositeSDF(std::vector<std::shared_ptr<AbstractSignedDistanceFunction>>{f1, f2});
  ASSERT_NEAR(func(0.0, 0.9), 0.4, eps);
  ASSERT_NEAR(func(0.0, 1.1), 0.4, eps);
  ASSERT_NEAR(func(0.0, 0.1), -0.4, eps);
  ASSERT_NEAR(func(0.0, 1.6), -0.1, eps);

  // ASSERT_NEAR(func.getSphereTracingDist(0.0, 1.0, M_PI * 0.5, eps), 0.5, eps);
  // ASSERT_NEAR(func.getSphereTracingDist(0.0, 1.0, M_PI * -0.5, eps), 0.5, eps);
}

}  // namespace autoware::dummy_perception_publisher

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
