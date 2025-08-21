// Copyright 2022 Autoware Foundation
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

#include "autoware/deviation_estimator/utils.hpp"

#include <gtest/gtest.h>

TEST(DeviationEstimatorUtils, WhetherToUseData1)
{
  const bool is_straight = false;
  const bool is_moving = false;
  const bool is_constant_velocity = false;
  const bool only_use_straight = false;
  const bool only_use_moving = false;
  const bool only_use_constant_velocity = false;
  EXPECT_TRUE(autoware::deviation_estimator::whether_to_use_data(
    is_straight, is_moving, is_constant_velocity, only_use_straight, only_use_moving,
    only_use_constant_velocity));
}

TEST(DeviationEstimatorUtils, WhetherToUseData2)
{
  const bool is_straight = true;
  const bool is_moving = false;
  const bool is_constant_velocity = false;
  const bool only_use_straight = false;
  const bool only_use_moving = false;
  const bool only_use_constant_velocity = false;
  EXPECT_TRUE(autoware::deviation_estimator::whether_to_use_data(
    is_straight, is_moving, is_constant_velocity, only_use_straight, only_use_moving,
    only_use_constant_velocity));
}

TEST(DeviationEstimatorUtils, WhetherToUseData3)
{
  const bool is_straight = false;
  const bool is_moving = false;
  const bool is_constant_velocity = false;
  const bool only_use_straight = true;
  const bool only_use_moving = false;
  const bool only_use_constant_velocity = false;
  EXPECT_FALSE(autoware::deviation_estimator::whether_to_use_data(
    is_straight, is_moving, is_constant_velocity, only_use_straight, only_use_moving,
    only_use_constant_velocity));
}

TEST(DeviationEstimatorUtils, WhetherToUseData4)
{
  const bool is_straight = true;
  const bool is_moving = false;
  const bool is_constant_velocity = false;
  const bool only_use_straight = true;
  const bool only_use_moving = false;
  const bool only_use_constant_velocity = false;
  EXPECT_TRUE(autoware::deviation_estimator::whether_to_use_data(
    is_straight, is_moving, is_constant_velocity, only_use_straight, only_use_moving,
    only_use_constant_velocity));
}

TEST(DeviationEstimatorUtils, WhetherToUseData5)
{
  const bool is_straight = true;
  const bool is_moving = true;
  const bool is_constant_velocity = false;
  const bool only_use_straight = true;
  const bool only_use_moving = false;
  const bool only_use_constant_velocity = false;
  EXPECT_TRUE(autoware::deviation_estimator::whether_to_use_data(
    is_straight, is_moving, is_constant_velocity, only_use_straight, only_use_moving,
    only_use_constant_velocity));
}

TEST(DeviationEstimatorUtils, WhetherToUseData6)
{
  const bool is_straight = true;
  const bool is_moving = false;
  const bool is_constant_velocity = false;
  const bool only_use_straight = true;
  const bool only_use_moving = true;
  const bool only_use_constant_velocity = false;
  EXPECT_FALSE(autoware::deviation_estimator::whether_to_use_data(
    is_straight, is_moving, is_constant_velocity, only_use_straight, only_use_moving,
    only_use_constant_velocity));
}

TEST(DeviationEstimatorUtils, WhetherToUseData7)
{
  const bool is_straight = true;
  const bool is_moving = true;
  const bool is_constant_velocity = false;
  const bool only_use_straight = true;
  const bool only_use_moving = true;
  const bool only_use_constant_velocity = false;
  EXPECT_TRUE(autoware::deviation_estimator::whether_to_use_data(
    is_straight, is_moving, is_constant_velocity, only_use_straight, only_use_moving,
    only_use_constant_velocity));
}
