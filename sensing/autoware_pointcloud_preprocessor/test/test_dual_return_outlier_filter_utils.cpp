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

#include "autoware/pointcloud_preprocessor/outlier_filter/dual_return_outlier_filter_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

using autoware::pointcloud_preprocessor::dual_return_outlier_filter::AzimuthBins;
using autoware::pointcloud_preprocessor::dual_return_outlier_filter::find_segment_points;

namespace
{
constexpr float deg_to_rad = static_cast<float>(M_PI / 180.0);
constexpr float half_degree = 0.5F * deg_to_rad;

struct Point
{
  float azimuth;
  float distance;
};

std::vector<Point> make_arc(float start_deg, float step_deg, int num_points, float distance = 5.0F)
{
  std::vector<Point> ring;
  for (int i = 0; i < num_points; ++i) {
    ring.push_back({(start_deg + step_deg * static_cast<float>(i)) * deg_to_rad, distance});
  }
  return ring;
}
}  // namespace

TEST(AzimuthBinsTest, FullCircleHasNonZeroResolution)
{
  // Arrange
  // 36 bins over [0, 2pi) are 10 degrees wide. The old integer resolution was 0 here.
  const AzimuthBins bins(0.0F, static_cast<float>(2.0 * M_PI), 36);

  // Act & Assert
  EXPECT_EQ(bins.bin(5.0F * deg_to_rad), 0U);
  EXPECT_EQ(bins.bin(15.0F * deg_to_rad), 1U);
  EXPECT_EQ(bins.bin(355.0F * deg_to_rad), 35U);
}

TEST(AzimuthBinsTest, NegativeAzimuthIsWrappedIntoZeroToTwoPi)
{
  // Arrange
  // atan2(y, x) gives [-pi, pi). -5 degrees is the same direction as 355 degrees.
  const AzimuthBins bins(0.0F, static_cast<float>(2.0 * M_PI), 36);

  // Act & Assert
  EXPECT_EQ(bins.bin(-5.0F * deg_to_rad), 35U);
  EXPECT_EQ(bins.bin(-175.0F * deg_to_rad), 18U);
}

TEST(AzimuthBinsTest, AzimuthJustBelowMaxStaysInLastBin)
{
  // Arrange
  const float max_azimuth = static_cast<float>(2.0 * M_PI);
  const AzimuthBins bins(0.0F, max_azimuth, 36);

  // Act & Assert
  EXPECT_EQ(bins.bin(std::nextafter(max_azimuth, 0.0F)), 35U);
}

TEST(AzimuthBinsTest, AzimuthOutsideRoiHasNoBin)
{
  // Arrange
  // Fixed_azimuth_ROI with the default 135 to 225 degrees: 36 bins of 2.5 degrees.
  const AzimuthBins bins(135.0F * deg_to_rad, 225.0F * deg_to_rad, 36);

  // Act & Assert
  EXPECT_FALSE(bins.bin(134.0F * deg_to_rad).has_value());
  EXPECT_FALSE(bins.bin(226.0F * deg_to_rad).has_value());
  EXPECT_EQ(bins.bin(136.0F * deg_to_rad), 0U);
  EXPECT_EQ(bins.bin(224.0F * deg_to_rad), 35U);
  // -170 degrees is 190 degrees, inside the ROI
  EXPECT_EQ(bins.bin(-170.0F * deg_to_rad), 22U);
}

TEST(AzimuthBinsTest, EmptyRangeHasNoBin)
{
  // Arrange
  const AzimuthBins bins(1.0F, 1.0F, 36);

  // Act & Assert
  EXPECT_FALSE(bins.bin(1.0F).has_value());
}

TEST(FindSegmentPointsTest, FirstAndLastPointsAreClassified)
{
  // Arrange
  // A dense arc: every point, including the first and the last one, is in the segment.
  const auto ring = make_arc(30.0F, 0.1F, 5);

  // Act
  const auto in_segment = find_segment_points(ring, 1.03, half_degree);

  // Assert
  EXPECT_EQ(in_segment, std::vector<bool>(5, true));
}

TEST(FindSegmentPointsTest, DecreasingAzimuthIsHandledLikeIncreasing)
{
  // Arrange
  const auto increasing = make_arc(30.0F, 0.1F, 5);
  const auto decreasing = make_arc(30.4F, -0.1F, 5);

  // Act & Assert
  EXPECT_EQ(
    find_segment_points(increasing, 1.03, half_degree),
    find_segment_points(decreasing, 1.03, half_degree));
}

TEST(FindSegmentPointsTest, WrapAroundIsContinuous)
{
  // Arrange
  // 359.8, 359.9, 0.0, 0.1 degrees in [0, 2pi)
  std::vector<Point> ring = {
    {359.8F * deg_to_rad, 5.0F},
    {359.9F * deg_to_rad, 5.0F},
    {0.0F, 5.0F},
    {0.1F * deg_to_rad, 5.0F}};

  // Act
  const auto in_segment = find_segment_points(ring, 1.03, half_degree);

  // Assert
  EXPECT_EQ(in_segment, std::vector<bool>(4, true));
}

TEST(FindSegmentPointsTest, IsolatedPointIsNotInSegment)
{
  // Arrange
  // The middle point is a drop (rain or fog), far from both neighbours in distance.
  std::vector<Point> ring = {
    {10.0F * deg_to_rad, 5.0F},
    {10.1F * deg_to_rad, 5.0F},
    {10.2F * deg_to_rad, 1.0F},
    {10.3F * deg_to_rad, 5.0F},
    {10.4F * deg_to_rad, 5.0F}};

  // Act
  const auto in_segment = find_segment_points(ring, 1.03, half_degree);

  // Assert
  EXPECT_EQ(in_segment, (std::vector<bool>{true, true, false, true, true}));
}

TEST(FindSegmentPointsTest, AzimuthStepAboveThresholdBreaksSegment)
{
  // Arrange
  // Steps of 1 degree are above the 0.5 degree threshold, so no point has a neighbour.
  const auto ring = make_arc(10.0F, 1.0F, 4);

  // Act
  const auto in_segment = find_segment_points(ring, 1.03, half_degree);

  // Assert
  EXPECT_EQ(in_segment, std::vector<bool>(4, false));
}

TEST(FindSegmentPointsTest, SinglePointAndEmptyRing)
{
  // Arrange
  const auto single = make_arc(10.0F, 0.1F, 1);
  const std::vector<Point> empty;

  // Act & Assert
  EXPECT_EQ(find_segment_points(single, 1.03, half_degree), std::vector<bool>{false});
  EXPECT_TRUE(find_segment_points(empty, 1.03, half_degree).empty());
}
