// Copyright 2026 TIER IV, inc.
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

#include "autoware/map_based_prediction/path_cut/path_cut_utils.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

namespace autoware::map_based_prediction::path_cut
{
namespace
{
using autoware_perception_msgs::msg::ObjectClassification;

PredictedPath make_path_of_length(const size_t num_poses)
{
  PredictedPath path;
  path.confidence = 0.7F;
  path.time_step.sec = 1;
  path.time_step.nanosec = 500000000;
  for (size_t i = 0; i < num_poses; ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = static_cast<double>(i);
    path.path.push_back(pose);
  }
  return path;
}

TEST(PathCutUtils, StoppingDistanceIsZeroWhenNotMoving)
{
  EXPECT_DOUBLE_EQ(distance_to_stop_with_max_deceleration(0.0, 5.0), 0.0);
  EXPECT_DOUBLE_EQ(distance_to_stop_with_max_deceleration(-1.0, 5.0), 0.0);
}

TEST(PathCutUtils, StoppingDistanceUsesConstantDeceleration)
{
  // v^2 / (2 a) = 100 / (2 * 5) = 10
  EXPECT_DOUBLE_EQ(distance_to_stop_with_max_deceleration(10.0, 5.0), 10.0);
}

TEST(PathCutUtils, StoppingDistanceIsInfiniteForNonPositiveDeceleration)
{
  EXPECT_TRUE(std::isinf(distance_to_stop_with_max_deceleration(10.0, 0.0)));
  EXPECT_TRUE(std::isinf(distance_to_stop_with_max_deceleration(10.0, -1.0)));
}

TEST(PathCutUtils, CanStopBeforeTheLineIsInclusiveAtTheBoundary)
{
  // distance_to_stop_with_max_deceleration(10, 5) == 10, so distance == 10 counts as stoppable.
  EXPECT_TRUE(can_stop_before_the_line(10.0, 10.0, 5.0));
  EXPECT_TRUE(can_stop_before_the_line(10.0001, 10.0, 5.0));
  EXPECT_FALSE(can_stop_before_the_line(9.9999, 10.0, 5.0));
}

TEST(PathCutUtils, CanStopBeforeTheLineIsFalseForNonPositiveDeceleration)
{
  EXPECT_FALSE(can_stop_before_the_line(1000.0, 1.0, 0.0));
  EXPECT_FALSE(can_stop_before_the_line(1000.0, 0.0, -1.0));
}

TEST(PathCutUtils, MaxDecelerationForLabelMapsEachClass)
{
  MaxDecelerationParams params;
  params.vehicle = 5.0;
  params.pedestrian = 1.5;
  params.bicycle = 2.0;
  params.motorcycle = 3.0;

  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::CAR), 5.0);
  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::BUS), 5.0);
  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::TRUCK), 5.0);
  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::TRAILER), 5.0);
  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::MOTORCYCLE), 3.0);
  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::PEDESTRIAN), 1.5);
  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::BICYCLE), 2.0);
  // Unknown falls back to the vehicle value.
  EXPECT_DOUBLE_EQ(max_deceleration_for_label(params, ObjectClassification::UNKNOWN), 5.0);
}

TEST(PathCutUtils, ForceCutKeepsInclusiveRangeAndMetadata)
{
  const auto path = make_path_of_length(5);
  const auto cut = force_cut_at_index(path, 2);

  ASSERT_EQ(cut.path.size(), 3U);  // keep indices [0, 1, 2]
  EXPECT_DOUBLE_EQ(cut.path.back().position.x, 2.0);
  EXPECT_FLOAT_EQ(cut.confidence, 0.7F);
  EXPECT_EQ(cut.time_step.sec, 1);
  EXPECT_EQ(cut.time_step.nanosec, 500000000U);
}

TEST(PathCutUtils, ForceCutClampsToPathSize)
{
  const auto path = make_path_of_length(3);
  const auto cut = force_cut_at_index(path, 100);
  EXPECT_EQ(cut.path.size(), 3U);
}

TEST(PathCutUtils, ForceCutKeepsSinglePoseAtIndexZero)
{
  const auto path = make_path_of_length(4);
  const auto cut = force_cut_at_index(path, 0);
  ASSERT_EQ(cut.path.size(), 1U);
  EXPECT_DOUBLE_EQ(cut.path.front().position.x, 0.0);
}

}  // namespace
}  // namespace autoware::map_based_prediction::path_cut
