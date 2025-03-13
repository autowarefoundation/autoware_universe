// Copyright 2025 TIER IV, Inc.
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

#include "autoware/simpl/processing/preprocessor.hpp"
#include "autoware/simpl/processing/rpe.hpp"

#include <gtest/gtest.h>

#include <utility>

namespace autoware::simpl
{
namespace
{
std::pair<processing::NodePoint, processing::NodePoint> construct_points()
{
  return {{1.0, 1.0}, {2.0, 2.0}};
}
}  // namespace

TEST(TestRPE, testCosinePeWithPoints)
{
  const auto [v1, v2] = construct_points();

  constexpr double abs_error = 1e-6;
  EXPECT_NEAR(processing::cosine_pe(v1, v2), 1.0, abs_error);
}

TEST(TestRPE, TestCosinePeWithElement)
{
  const auto [v1, v2] = construct_points();

  constexpr double abs_error = 1e-6;
  EXPECT_NEAR(processing::cosine_pe(v1, v2.x, v2.y), 1.0, abs_error);
}

TEST(TestRPE, testSinePeWithPoints)
{
  const auto [v1, v2] = construct_points();

  constexpr double abs_error = 1e-6;
  EXPECT_NEAR(processing::sine_pe(v1, v2), 0.0, abs_error);
}

TEST(TestRPE, testSinePeWithElement)
{
  const auto [v1, v2] = construct_points();

  constexpr double abs_error = 1e-6;
  EXPECT_NEAR(processing::sine_pe(v1, v2.x, v2.y), 0.0, abs_error);
}
}  // namespace autoware::simpl
