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

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_detection.hpp"

#include <opencv2/core.hpp>

#include <gtest/gtest.h>

namespace autoware::pointcloud_preprocessor
{

BlockageDetectionConfig make_default_config()
{
  BlockageDetectionConfig config;
  config.blockage_ratio_threshold = 0.1f;
  config.blockage_kernel = 1;
  config.blockage_count_threshold = 3;
  config.horizontal_ring_id = 5;
  config.horizontal_resolution = 1.0;
  config.angle_range_min_deg = 0.0;
  config.angle_range_max_deg = 360.0;
  return config;
}

TEST(BlockageDetectorTest, ConstructorDoesNotThrow)
{
  // Arrange
  auto config = make_default_config();

  // Act & Assert
  EXPECT_NO_THROW({ BlockageDetector detector(config); });
}

TEST(BlockageDetectorTest, NoBlockageWhenAllPixelsHaveDepth)
{
  // Arrange
  BlockageDetector detector(make_default_config());
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(1000));

  // Act
  auto result = detector.compute_blockage_diagnostics(depth_image);

  // Assert
  EXPECT_FLOAT_EQ(result.ground.blockage_ratio, 0.0f);
  EXPECT_FLOAT_EQ(result.sky.blockage_ratio, 0.0f);
  EXPECT_EQ(result.ground.blockage_count, 0);
  EXPECT_EQ(result.sky.blockage_count, 0);
}

TEST(BlockageDetectorTest, FullBlockageWhenAllPixelsNoReturn)
{
  // Arrange
  auto config = make_default_config();
  config.blockage_kernel = 0;
  BlockageDetector detector(config);
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(0));

  // Act
  auto result = detector.compute_blockage_diagnostics(depth_image);

  // Assert
  EXPECT_FLOAT_EQ(result.ground.blockage_ratio, 1.0f);
  EXPECT_FLOAT_EQ(result.sky.blockage_ratio, 1.0f);
}

TEST(BlockageDetectorTest, PartialBlockageDetection)
{
  // Arrange
  auto config = make_default_config();
  config.horizontal_ring_id = 5;
  config.blockage_kernel = 0;
  BlockageDetector detector(config);
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(1000));
  depth_image(cv::Rect(0, 0, 20, 5)).setTo(cv::Scalar(0));  // Sky region blocked

  // Act
  auto result = detector.compute_blockage_diagnostics(depth_image);

  // Assert
  EXPECT_FLOAT_EQ(result.ground.blockage_ratio, 0.0f);
  EXPECT_FLOAT_EQ(result.sky.blockage_ratio, 1.0f);
}

TEST(BlockageDetectorTest, BlockageCountIncrementWhenAboveThreshold)
{
  // Arrange
  auto config = make_default_config();
  config.blockage_kernel = 0;
  BlockageDetector detector(config);
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(0));

  // Act
  auto result1 = detector.compute_blockage_diagnostics(depth_image);
  auto result2 = detector.compute_blockage_diagnostics(depth_image);
  auto result3 = detector.compute_blockage_diagnostics(depth_image);

  // Assert
  EXPECT_EQ(result1.ground.blockage_count, 1);
  EXPECT_EQ(result2.ground.blockage_count, 2);
  EXPECT_EQ(result3.ground.blockage_count, 3);
}

TEST(BlockageDetectorTest, BlockageCountResetWhenBelowThreshold)
{
  // Arrange
  auto config = make_default_config();
  config.blockage_ratio_threshold = 0.5f;
  config.blockage_kernel = 0;
  BlockageDetector detector(config);
  cv::Mat blockage_image(10, 20, CV_16UC1, cv::Scalar(0));
  cv::Mat clear_image(10, 20, CV_16UC1, cv::Scalar(1000));

  // Act
  detector.compute_blockage_diagnostics(blockage_image);
  auto result_accumulated = detector.compute_blockage_diagnostics(blockage_image);
  auto result_reset = detector.compute_blockage_diagnostics(clear_image);

  // Assert
  EXPECT_EQ(result_accumulated.ground.blockage_count, 2);
  EXPECT_EQ(result_reset.ground.blockage_count, 0);
}

TEST(BlockageDetectorTest, BlockageAngleRangeCalculation)
{
  // Arrange
  auto config = make_default_config();
  config.horizontal_resolution = 0.5;
  config.angle_range_min_deg = -180.0;
  config.blockage_kernel = 0;
  config.horizontal_ring_id = 5;
  BlockageDetector detector(config);
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(1000));
  depth_image(cv::Rect(4, 0, 5, 10)).setTo(cv::Scalar(0));  // Columns 4-8 blocked

  // Act
  auto result = detector.compute_blockage_diagnostics(depth_image);

  // Assert
  EXPECT_NEAR(result.ground.blockage_start_deg, -178.0f, 0.1f);
  EXPECT_NEAR(result.ground.blockage_end_deg, -175.5f, 0.1f);
}

TEST(BlockageDetectorTest, BlockageMaskIsReturned)
{
  // Arrange
  BlockageDetector detector(make_default_config());
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(0));

  // Act
  auto result = detector.compute_blockage_diagnostics(depth_image);

  // Assert
  EXPECT_EQ(result.blockage_mask.rows, 10);
  EXPECT_EQ(result.blockage_mask.cols, 20);
  EXPECT_EQ(result.blockage_mask.type(), CV_8UC1);
}

TEST(BlockageDetectorTest, DiagnosticOutputStaleWhenNotComputed)
{
  // Arrange
  BlockageDetector detector(make_default_config());

  // Act
  auto output = detector.get_blockage_diagnostics_output();

  // Assert
  EXPECT_EQ(output.level, DiagnosticLevel::STALE);
}

TEST(BlockageDetectorTest, DiagnosticOutputOKWhenNoBlockage)
{
  // Arrange
  BlockageDetector detector(make_default_config());
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(1000));

  // Act
  detector.compute_blockage_diagnostics(depth_image);
  auto output = detector.get_blockage_diagnostics_output();

  // Assert
  EXPECT_EQ(output.level, DiagnosticLevel::OK);
}

TEST(BlockageDetectorTest, DiagnosticOutputWarnWhenBlockageDetected)
{
  // Arrange
  auto config = make_default_config();
  config.blockage_ratio_threshold = 0.5f;
  config.blockage_count_threshold = 10;
  config.blockage_kernel = 0;
  config.horizontal_ring_id = 5;
  BlockageDetector detector(config);
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(1000));
  depth_image(cv::Rect(0, 5, 5, 5)).setTo(cv::Scalar(0));  // Small ground blockage

  // Act
  detector.compute_blockage_diagnostics(depth_image);
  auto output = detector.get_blockage_diagnostics_output();

  // Assert
  EXPECT_EQ(output.level, DiagnosticLevel::WARN);
}

TEST(BlockageDetectorTest, DiagnosticOutputErrorWhenBlockageExceedsThresholds)
{
  // Arrange
  auto config = make_default_config();
  config.blockage_count_threshold = 2;
  config.blockage_kernel = 0;
  BlockageDetector detector(config);
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(0));

  // Act
  detector.compute_blockage_diagnostics(depth_image);
  detector.compute_blockage_diagnostics(depth_image);
  detector.compute_blockage_diagnostics(depth_image);
  auto output = detector.get_blockage_diagnostics_output();

  // Assert
  EXPECT_EQ(output.level, DiagnosticLevel::ERROR);
}

}  // namespace autoware::pointcloud_preprocessor

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
