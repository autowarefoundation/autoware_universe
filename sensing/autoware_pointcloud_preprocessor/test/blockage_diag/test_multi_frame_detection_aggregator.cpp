// Copyright 2026 TIER IV
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

#include "autoware/pointcloud_preprocessor/blockage_diag/multi_frame_detection_aggregator.hpp"

#include <opencv2/core.hpp>

#include <gtest/gtest.h>

namespace autoware::pointcloud_preprocessor
{

MultiFrameDetectionAggregatorConfig make_default_config()
{
  MultiFrameDetectionAggregatorConfig config;
  config.buffering_frames = 5;
  config.buffering_interval = 1;
  return config;
}

TEST(MultiFrameDetectionAggregatorTest, ConstructorDoesNotThrow)
{
  // Arrange
  auto config = make_default_config();

  // Act & Assert
  EXPECT_NO_THROW({ MultiFrameDetectionAggregator aggregator(config); });
}

TEST(MultiFrameDetectionAggregatorTest, ZeroBufferingIntervalReturnsClonedInput)
{
  // Arrange
  auto config = make_default_config();
  config.buffering_interval = 0;
  MultiFrameDetectionAggregator aggregator(config);
  cv::Mat detected_mask(10, 20, CV_8UC1, cv::Scalar(255));

  // Act
  auto result = aggregator.update(detected_mask);

  // Assert
  EXPECT_EQ(result.rows, detected_mask.rows);
  EXPECT_EQ(result.cols, detected_mask.cols);
  EXPECT_EQ(result.type(), CV_8UC1);
  EXPECT_EQ(cv::countNonZero(result), cv::countNonZero(detected_mask));
}

TEST(MultiFrameDetectionAggregatorTest, EmptyMaskReturnsEmptyResult)
{
  // Arrange
  auto config = make_default_config();
  config.buffering_interval = 0;
  MultiFrameDetectionAggregator aggregator(config);
  cv::Mat empty_mask(10, 20, CV_8UC1, cv::Scalar(0));

  // Act
  auto result = aggregator.update(empty_mask);

  // Assert
  EXPECT_EQ(cv::countNonZero(result), 0);
}

TEST(MultiFrameDetectionAggregatorTest, SingleFrameBeforeBufferFills)
{
  // Arrange
  auto config = make_default_config();
  config.buffering_frames = 5;
  config.buffering_interval = 0;
  MultiFrameDetectionAggregator aggregator(config);
  cv::Mat detected_mask(10, 20, CV_8UC1, cv::Scalar(255));

  // Act
  auto result = aggregator.update(detected_mask);

  // Assert
  EXPECT_EQ(result.rows, detected_mask.rows);
  EXPECT_EQ(result.cols, detected_mask.cols);
  EXPECT_EQ(result.type(), CV_8UC1);
}

TEST(MultiFrameDetectionAggregatorTest, BufferFillsAndAggregates)
{
  // Arrange
  auto config = make_default_config();
  config.buffering_frames = 3;
  config.buffering_interval = 0;
  MultiFrameDetectionAggregator aggregator(config);
  cv::Mat detected_mask(10, 20, CV_8UC1, cv::Scalar(255));

  // Act - fill buffer with consistent detections
  aggregator.update(detected_mask);
  aggregator.update(detected_mask);
  auto result = aggregator.update(detected_mask);

  // Assert - with 3 frames all having the same detection, inRange(2, 3) should match
  EXPECT_GT(cv::countNonZero(result), 0);
}

TEST(MultiFrameDetectionAggregatorTest, IntermittentDetectionFiltered)
{
  // Arrange
  auto config = make_default_config();
  config.buffering_frames = 3;
  config.buffering_interval = 0;
  MultiFrameDetectionAggregator aggregator(config);
  cv::Mat detected_mask(10, 20, CV_8UC1, cv::Scalar(255));
  cv::Mat empty_mask(10, 20, CV_8UC1, cv::Scalar(0));

  // Act - alternating detections
  aggregator.update(detected_mask);
  aggregator.update(empty_mask);
  auto result = aggregator.update(detected_mask);

  // Assert - with 2/3 frames having detection, inRange(2, 3) should match
  EXPECT_GT(cv::countNonZero(result), 0);
}

TEST(MultiFrameDetectionAggregatorTest, BufferingIntervalSkipsFrames)
{
  // Arrange
  auto config = make_default_config();
  config.buffering_frames = 3;
  config.buffering_interval = 2;
  MultiFrameDetectionAggregator aggregator(config);
  cv::Mat detected_mask(10, 20, CV_8UC1, cv::Scalar(255));

  // Act - frame 0: not stored (frame_count=1), frame 1: not stored (frame_count=2),
  //       frame 2: stored (frame_count=0)
  aggregator.update(detected_mask);
  aggregator.update(detected_mask);
  auto result = aggregator.update(detected_mask);

  // Assert
  EXPECT_EQ(result.rows, detected_mask.rows);
  EXPECT_EQ(result.cols, detected_mask.cols);
}

TEST(MultiFrameDetectionAggregatorTest, CircularBufferReplacesOldFrames)
{
  // Arrange
  auto config = make_default_config();
  config.buffering_frames = 2;
  config.buffering_interval = 0;
  MultiFrameDetectionAggregator aggregator(config);
  cv::Mat detected_mask(10, 20, CV_8UC1, cv::Scalar(255));
  cv::Mat empty_mask(10, 20, CV_8UC1, cv::Scalar(0));

  // Act - fill with detections, then replace with empty
  aggregator.update(detected_mask);
  aggregator.update(detected_mask);
  aggregator.update(empty_mask);
  auto result = aggregator.update(empty_mask);

  // Assert - buffer now has 2 empty masks, so no detection
  EXPECT_EQ(cv::countNonZero(result), 0);
}

}  // namespace autoware::pointcloud_preprocessor

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
