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

#include "autoware/speed_scale_corrector/utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::speed_scale_corrector
{

// Test for utility functions
// - smooth_gaussian: Gaussian smoothing with various input conditions
// - intersect_intervals: Time interval intersection calculation
// - arange: Array generation with specified range and step

class UtilsTest : public ::testing::Test
{
};

// Test basic Gaussian smoothing functionality
TEST_F(UtilsTest, SmoothGaussianBasic)
{
  std::vector<double> times;
  std::vector<double> values;

  for (size_t i = 0; i < 500; ++i) {
    times.push_back(static_cast<double>(i) / 100.0);
    if (i % 2 == 0) {
      values.push_back(-1.0);
    } else {
      values.push_back(1.0);
    }
  }

  auto result = smooth_gaussian(times, values, 1.0);

  EXPECT_EQ(result.size(), values.size());

  // Check that smoothed values are reasonable
  for (const auto & value : result) {
    EXPECT_GT(value, -1.0);
    EXPECT_LT(value, 1.0);
  }
}

// Test Gaussian smoothing with empty input arrays
TEST_F(UtilsTest, SmoothGaussianEmptyInput)
{
  std::vector<double> empty_times;
  std::vector<double> empty_values;

  auto result = smooth_gaussian(empty_times, empty_values, 1.0);

  EXPECT_TRUE(result.empty());
}

// Test Gaussian smoothing with mismatched array sizes
TEST_F(UtilsTest, SmoothGaussianMismatchedSize)
{
  std::vector<double> times = {0.0, 1.0, 2.0};
  std::vector<double> values = {1.0, 2.0};

  auto result = smooth_gaussian(times, values, 1.0);

  EXPECT_TRUE(result.empty());
}

// Test Gaussian smoothing with invalid sigma value
TEST_F(UtilsTest, SmoothGaussianInvalidSigma)
{
  std::vector<double> times = {0.0, 1.0, 2.0};
  std::vector<double> values = {1.0, 2.0, 3.0};

  auto result = smooth_gaussian(times, values, 0.0);

  EXPECT_EQ(result, values);
}

// Test Gaussian smoothing with non-monotonic time values
TEST_F(UtilsTest, SmoothGaussianNonMonotonicTimes)
{
  std::vector<double> times = {0.0, 2.0, 1.0, 3.0};
  std::vector<double> values = {1.0, 2.0, 3.0, 4.0};

  auto result = smooth_gaussian(times, values, 1.0);

  EXPECT_TRUE(result.empty());
}

// Test basic interval intersection functionality
TEST_F(UtilsTest, IntersectIntervalsBasic)
{
  std::vector<std::pair<double, double>> intervals = {{1.0, 5.0}, {2.0, 4.0}, {1.5, 6.0}};

  auto result = intersect_intervals(intervals);

  EXPECT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->first, 2.0);
  EXPECT_DOUBLE_EQ(result->second, 4.0);
}

// Test interval intersection with no overlapping intervals
TEST_F(UtilsTest, IntersectIntervalsNoOverlap)
{
  std::vector<std::pair<double, double>> intervals = {{1.0, 2.0}, {3.0, 4.0}};

  auto result = intersect_intervals(intervals);

  EXPECT_FALSE(result.has_value());
}

// Test interval intersection with empty input
TEST_F(UtilsTest, IntersectIntervalsEmpty)
{
  std::vector<std::pair<double, double>> intervals;

  auto result = intersect_intervals(intervals);

  EXPECT_FALSE(result.has_value());
}

// Test interval intersection with single interval
TEST_F(UtilsTest, IntersectIntervalsSingleInterval)
{
  std::vector<std::pair<double, double>> intervals = {{1.0, 3.0}};

  auto result = intersect_intervals(intervals);

  EXPECT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->first, 1.0);
  EXPECT_DOUBLE_EQ(result->second, 3.0);
}

// Test basic arange functionality
TEST_F(UtilsTest, ArangeBasic)
{
  auto result = arange(0.0, 1.0, 0.2);

  EXPECT_EQ(result.size(), 5);
  EXPECT_DOUBLE_EQ(result[0], 0.0);
  EXPECT_DOUBLE_EQ(result[1], 0.2);
  EXPECT_DOUBLE_EQ(result[2], 0.4);
  EXPECT_DOUBLE_EQ(result[3], 0.6);
  EXPECT_DOUBLE_EQ(result[4], 0.8);
}

// Test arange with invalid step interval
TEST_F(UtilsTest, ArangeInvalidInterval)
{
  auto result = arange(0.0, 1.0, 0.0);

  EXPECT_TRUE(result.empty());
}

// Test arange with start value greater than end
TEST_F(UtilsTest, ArangeStartGreaterThanEnd)
{
  auto result = arange(2.0, 1.0, 0.1);

  EXPECT_TRUE(result.empty());
}

// Test arange with equal start and end values
TEST_F(UtilsTest, ArangeStartEqualEnd)
{
  auto result = arange(1.0, 1.0, 0.1);

  EXPECT_TRUE(result.empty());
}

// Test arange with step interval larger than range
TEST_F(UtilsTest, ArangeLargeInterval)
{
  auto result = arange(0.0, 1.0, 2.0);

  EXPECT_EQ(result.size(), 1);
  EXPECT_DOUBLE_EQ(result[0], 0.0);
}

// Test Gaussian smoothing with constant input values
TEST_F(UtilsTest, SmoothGaussianConstantValues)
{
  std::vector<double> times = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> values = {5.0, 5.0, 5.0, 5.0, 5.0};

  auto result = smooth_gaussian(times, values, 1.0);

  EXPECT_EQ(result.size(), values.size());
  for (const auto & val : result) {
    EXPECT_NEAR(val, 5.0, 1e-10);
  }
}

// Test Gaussian smoothing with single data point
TEST_F(UtilsTest, SmoothGaussianSinglePoint)
{
  std::vector<double> times = {1.0};
  std::vector<double> values = {3.0};

  auto result = smooth_gaussian(times, values, 1.0);

  EXPECT_EQ(result.size(), 1);
  EXPECT_DOUBLE_EQ(result[0], 3.0);
}

}  // namespace autoware::speed_scale_corrector
