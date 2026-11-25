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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

using autoware::pointcloud_preprocessor::dual_return_outlier_filter::AzimuthBins;
using autoware::pointcloud_preprocessor::dual_return_outlier_filter::classify_weak_first_ring;
using autoware::pointcloud_preprocessor::dual_return_outlier_filter::find_segment_points;
using autoware::pointcloud_preprocessor::dual_return_outlier_filter::WeakFirstParams;
using autoware::pointcloud_preprocessor::dual_return_outlier_filter::XyzRoi;

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

std::vector<Point> concat(std::vector<std::vector<Point>> parts)
{
  std::vector<Point> ring;
  for (const auto & part : parts) {
    ring.insert(ring.end(), part.begin(), part.end());
  }
  return ring;
}

// A segment and two isolated points in bin 0 (0 to 10 deg), and a segment in bin 2 (20 to 30 deg)
std::vector<Point> make_ring_with_one_noisy_bin()
{
  return concat({make_arc(1.0F, 0.1F, 6), make_arc(4.0F, 3.0F, 2), make_arc(21.0F, 0.1F, 6)});
}

const AzimuthBins full_circle_bins(0.0F, static_cast<float>(2.0 * M_PI), 36);
// Default values of the node: weak_first_distance_ratio, max_azimuth_diff, and
// weak_first_local_noise_threshold
const WeakFirstParams default_params{1.004, half_degree, 2};

struct Point3
{
  float x;
  float y;
  float z;
};
}  // namespace

TEST(AzimuthBinsTest, FullCircleHasNonZeroResolution)
{
  // Arrange
  // 36 bins over [0, 2pi) are 10 degrees wide.
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
  // Fixed_azimuth_ROI with the default 135 to 225 degrees
  const AzimuthBins bins(135.0F * deg_to_rad, 225.0F * deg_to_rad, 36);

  // Act & Assert
  EXPECT_FALSE(bins.bin(134.0F * deg_to_rad).has_value());
  EXPECT_FALSE(bins.bin(226.0F * deg_to_rad).has_value());
}

TEST(AzimuthBinsTest, AzimuthInsideRoiIsBinnedFromMinAzimuth)
{
  // Arrange
  // 36 bins over 135 to 225 degrees are 2.5 degrees wide
  const AzimuthBins bins(135.0F * deg_to_rad, 225.0F * deg_to_rad, 36);

  // Act & Assert
  EXPECT_EQ(bins.bin(136.0F * deg_to_rad), 0U);
  EXPECT_EQ(bins.bin(224.0F * deg_to_rad), 35U);
}

TEST(AzimuthBinsTest, NegativeAzimuthInsideRoiIsWrapped)
{
  // Arrange
  const AzimuthBins bins(135.0F * deg_to_rad, 225.0F * deg_to_rad, 36);

  // Act & Assert
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

TEST(XyzRoiTest, ContainsOnlyPointsStrictlyInsideEveryRange)
{
  // Arrange
  const XyzRoi roi{{-12.0F, 18.0F}, {-2.0F, 2.0F}, {0.0F, 10.0F}};

  // Act & Assert
  EXPECT_TRUE(roi.contains(Point3{5.0F, 1.0F, 1.0F}));
  // Each coordinate outside its range, including on the bound, puts the point outside.
  EXPECT_FALSE(roi.contains(Point3{30.0F, 1.0F, 1.0F}));
  EXPECT_FALSE(roi.contains(Point3{5.0F, 2.5F, 1.0F}));
  EXPECT_FALSE(roi.contains(Point3{5.0F, -2.5F, 1.0F}));
  EXPECT_FALSE(roi.contains(Point3{5.0F, 1.0F, 0.0F}));
  EXPECT_FALSE(roi.contains(Point3{5.0F, 1.0F, 10.0F}));
}

TEST(ClassifyWeakFirstRingTest, SegmentInNoisyBinIsNoiseAndSegmentInQuietBinIsNot)
{
  // Arrange
  const auto ring = make_ring_with_one_noisy_bin();
  const auto all_in_roi = [](const Point &) { return true; };

  // Act
  const auto result = classify_weak_first_ring(ring, full_circle_bins, default_params, all_in_roi);

  // Assert
  // The 6 segment points and the 2 isolated points of bin 0 are noise. The segment of bin 2 is
  // not. Bin 0 counts all 8 noise points.
  const std::vector<bool> expected_is_noise = {true, true,  true,  true,  true,  true,  true,
                                               true, false, false, false, false, false, false};
  EXPECT_EQ(result.is_noise, expected_is_noise);
  ASSERT_EQ(result.noise_frequency.size(), 36U);
  EXPECT_EQ(result.noise_frequency[0], 8);
  EXPECT_EQ(result.noise_frequency[2], 0);
}

TEST(ClassifyWeakFirstRingTest, PointsOutsideRoiAreNotCounted)
{
  // Arrange
  // With no point in the ROI, no bin reaches the threshold.
  const auto ring = make_ring_with_one_noisy_bin();
  const auto none_in_roi = [](const Point &) { return false; };

  // Act
  const auto result = classify_weak_first_ring(ring, full_circle_bins, default_params, none_in_roi);

  // Assert
  // Only the 2 isolated points are noise, and no bin counts anything.
  const std::vector<bool> expected_is_noise = {false, false, false, false, false, false, true,
                                               true,  false, false, false, false, false, false};
  EXPECT_EQ(result.is_noise, expected_is_noise);
  for (const int count : result.noise_frequency) {
    EXPECT_EQ(count, 0);
  }
}

TEST(ClassifyWeakFirstRingTest, ResultDoesNotDependOnScanDirection)
{
  // Arrange
  auto reversed_ring = make_ring_with_one_noisy_bin();
  std::reverse(reversed_ring.begin(), reversed_ring.end());
  const auto all_in_roi = [](const Point &) { return true; };

  // Act
  const auto forward = classify_weak_first_ring(
    make_ring_with_one_noisy_bin(), full_circle_bins, default_params, all_in_roi);
  const auto backward =
    classify_weak_first_ring(reversed_ring, full_circle_bins, default_params, all_in_roi);

  // Assert
  // The same points are noise, in reverse order, and the bin counts are the same.
  std::vector<bool> backward_is_noise_in_forward_order = backward.is_noise;
  std::reverse(
    backward_is_noise_in_forward_order.begin(), backward_is_noise_in_forward_order.end());
  EXPECT_EQ(backward_is_noise_in_forward_order, forward.is_noise);
  EXPECT_EQ(backward.noise_frequency, forward.noise_frequency);
}
