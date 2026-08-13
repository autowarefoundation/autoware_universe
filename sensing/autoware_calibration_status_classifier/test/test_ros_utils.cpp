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

#include "autoware/calibration_status_classifier/ros_utils.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

namespace autoware::calibration_status_classifier
{

TEST(ComposeInOutTopicsTest, BroadcastsSingleMiscalibrationThreshold)
{
  const auto topics = compose_in_out_topics(
    {"/sensing/lidar/concatenated/pointcloud"}, {"/sensing/camera/camera0/image_raw"}, {0.1},
    {0.25}, {false});

  ASSERT_EQ(topics.size(), 1u);
  EXPECT_DOUBLE_EQ(topics.at(0).miscalibration_confidence_threshold, 0.25);
}

TEST(ComposeInOutTopicsTest, AssignsPerPairMiscalibrationThresholds)
{
  const auto topics = compose_in_out_topics(
    {"/sensing/lidar/concatenated/pointcloud"},
    {"/sensing/camera/camera0/image_raw", "/sensing/camera/camera1/image_raw"}, {0.1}, {0.1, 0.2},
    {false, true});

  ASSERT_EQ(topics.size(), 2u);
  EXPECT_DOUBLE_EQ(topics.at(0).miscalibration_confidence_threshold, 0.1);
  EXPECT_DOUBLE_EQ(topics.at(1).miscalibration_confidence_threshold, 0.2);
}

TEST(ComposeInOutTopicsTest, ThrowsOnInvalidThresholdCount)
{
  EXPECT_THROW(
    compose_in_out_topics(
      {"/sensing/lidar/concatenated/pointcloud"},
      {"/sensing/camera/camera0/image_raw", "/sensing/camera/camera1/image_raw"}, {0.1},
      {0.0, 0.1, 0.2}, {false, false}),
    std::invalid_argument);
}

}  // namespace autoware::calibration_status_classifier
