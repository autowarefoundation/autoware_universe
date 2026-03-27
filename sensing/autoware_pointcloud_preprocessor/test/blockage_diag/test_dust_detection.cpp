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

#include "autoware/pointcloud_preprocessor/blockage_diag/dust_detection.hpp"

#include <opencv2/core.hpp>

#include <gtest/gtest.h>

namespace autoware::pointcloud_preprocessor
{

DustDetectionConfig make_default_config()
{
  DustDetectionConfig config;
  config.dust_ratio_threshold = 0.1f;
  config.dust_kernel_size = 1;
  config.dust_count_threshold = 3;
  config.horizontal_ring_id = 5;
  return config;
}

TEST(DustDetectorTest, ConstructorDoesNotThrow)
{
  // Arrange
  auto config = make_default_config();

  // Act & Assert
  EXPECT_NO_THROW({ DustDetector detector(config); });
}

TEST(DustDetectorTest, NoDustWhenAllPixelsHaveDepth)
{
  // Arrange
  DustDetector detector(make_default_config());
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(1000));

  // Act
  auto result = detector.compute_dust_diagnostics(depth_image);

  // Assert
  EXPECT_FLOAT_EQ(result.ground_dust_ratio, 0.0f);
  EXPECT_EQ(result.dust_frame_count, 0);
}

TEST(DustDetectorTest, FullDustWhenGroundHasNoReturn)
{
  // Arrange
  auto config = make_default_config();
  config.dust_kernel_size = 0;
  config.horizontal_ring_id = 5;
  DustDetector detector(config);
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(0));

  // Act
  auto result = detector.compute_dust_diagnostics(depth_image);

  // Assert
  EXPECT_FLOAT_EQ(result.ground_dust_ratio, 1.0f);
}

TEST(DustDetectorTest, PartialDustDetection)
{
  // Arrange
  auto config = make_default_config();
  config.dust_kernel_size = 0;
  config.horizontal_ring_id = 5;
  DustDetector detector(config);
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(1000));
  depth_image(cv::Rect(0, 5, 10, 5)).setTo(cv::Scalar(0));  // Half of ground region no-return

  // Act
  auto result = detector.compute_dust_diagnostics(depth_image);

  // Assert
  EXPECT_FLOAT_EQ(result.ground_dust_ratio, 0.5f);
}

TEST(DustDetectorTest, DustCountIncrementWhenAboveThreshold)
{
  // Arrange
  auto config = make_default_config();
  config.dust_kernel_size = 0;
  DustDetector detector(config);
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(0));

  // Act
  auto result1 = detector.compute_dust_diagnostics(depth_image);
  auto result2 = detector.compute_dust_diagnostics(depth_image);

  // Assert
  EXPECT_EQ(result1.dust_frame_count, 1);
  EXPECT_EQ(result2.dust_frame_count, 2);
}

TEST(DustDetectorTest, DustCountResetWhenBelowThreshold)
{
  // Arrange
  auto config = make_default_config();
  config.dust_ratio_threshold = 0.5f;
  config.dust_kernel_size = 0;
  DustDetector detector(config);
  cv::Mat dust_image(10, 20, CV_16UC1, cv::Scalar(0));
  cv::Mat clear_image(10, 20, CV_16UC1, cv::Scalar(1000));

  // Act
  detector.compute_dust_diagnostics(dust_image);
  auto result_accumulated = detector.compute_dust_diagnostics(dust_image);
  auto result_reset = detector.compute_dust_diagnostics(clear_image);

  // Assert
  EXPECT_EQ(result_accumulated.dust_frame_count, 2);
  EXPECT_EQ(result_reset.dust_frame_count, 0);
}

TEST(DustDetectorTest, DustMaskIsReturned)
{
  // Arrange
  DustDetector detector(make_default_config());
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(0));

  // Act
  auto result = detector.compute_dust_diagnostics(depth_image);

  // Assert
  EXPECT_EQ(result.dust_mask.rows, 10);
  EXPECT_EQ(result.dust_mask.cols, 20);
  EXPECT_EQ(result.dust_mask.type(), CV_8UC1);
}

TEST(DustDetectorTest, DiagnosticOutputStaleWhenNotComputed)
{
  // Arrange
  DustDetector detector(make_default_config());

  // Act
  auto output = detector.get_dust_diagnostics_output();

  // Assert
  EXPECT_EQ(output.level, DiagnosticLevel::STALE);
}

TEST(DustDetectorTest, DiagnosticOutputOKWhenNoDust)
{
  // Arrange
  DustDetector detector(make_default_config());
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(1000));

  // Act
  detector.compute_dust_diagnostics(depth_image);
  auto output = detector.get_dust_diagnostics_output();

  // Assert
  EXPECT_EQ(output.level, DiagnosticLevel::OK);
}

TEST(DustDetectorTest, DiagnosticOutputWarnWhenDustDetected)
{
  // Arrange
  auto config = make_default_config();
  config.dust_ratio_threshold = 0.5f;
  config.dust_count_threshold = 10;
  config.dust_kernel_size = 0;
  config.horizontal_ring_id = 5;
  DustDetector detector(config);
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(1000));
  depth_image(cv::Rect(0, 5, 5, 5)).setTo(cv::Scalar(0));  // Small ground dust

  // Act
  detector.compute_dust_diagnostics(depth_image);
  auto output = detector.get_dust_diagnostics_output();

  // Assert
  EXPECT_EQ(output.level, DiagnosticLevel::WARN);
}

TEST(DustDetectorTest, DiagnosticOutputErrorWhenDustExceedsThresholds)
{
  // Arrange
  auto config = make_default_config();
  config.dust_count_threshold = 2;
  config.dust_kernel_size = 0;
  DustDetector detector(config);
  cv::Mat depth_image(10, 20, CV_16UC1, cv::Scalar(0));

  // Act
  detector.compute_dust_diagnostics(depth_image);
  detector.compute_dust_diagnostics(depth_image);
  detector.compute_dust_diagnostics(depth_image);
  auto output = detector.get_dust_diagnostics_output();

  // Assert
  EXPECT_EQ(output.level, DiagnosticLevel::ERROR);
}

}  // namespace autoware::pointcloud_preprocessor

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
