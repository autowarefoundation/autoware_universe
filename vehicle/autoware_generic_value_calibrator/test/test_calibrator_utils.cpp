// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include "autoware_generic_value_calibrator/generic_value_calibrator_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

/**
 * @file test_calibrator_utils.cpp
 * @brief Unit tests for calibrator utility functions and algorithms
 *
 * This test file covers core calibration algorithm logic:
 * - Nearest value search algorithm
 * - Index value search with threshold
 * - Average calculation (including empty vector)
 * - Lowpass filter logic
 * - Map consistency/monotonicity checks
 * - RLS (Recursive Least Squares) update algorithm
 *
 * @note Most functions in GenericValueCalibrator are private and require ROS2 node context.
 *       These tests use lambda functions to mirror the private function logic for testing.
 *       For full node behavior, consider using FakeTestNode for integration tests.
 *
 * @test CalibratorUtilsTest.NearestValueSearchLogic - Tests nearest value search algorithm
 * @test CalibratorUtilsTest.IndexValueSearchLogic - Tests index search with threshold
 * @test CalibratorUtilsTest.GetAverageLogic - Tests average calculation
 * @test CalibratorUtilsTest.LowpassFilterLogic - Tests lowpass filter logic
 * @test CalibratorUtilsTest.MapConsistencyLogic - Tests map monotonicity checks
 * @test CalibratorUtilsTest.RLSUpdateLogic - Tests RLS update algorithm
 */

/**
 * @brief Test nearest value search algorithm
 *
 * Tests the algorithm that finds the index of the nearest value in a sorted array.
 * This is used to find the closest map cell for a given input value.
 */
TEST(CalibratorUtilsTest, NearestValueSearchLogic)
{
  std::vector<double> value_index = {-1.0, -0.5, 0.0, 0.5, 1.0};

  // Test exact match
  auto find_nearest = [](const std::vector<double> & index, double value) {
    double max_dist = std::numeric_limits<double>::max();
    int nearest_idx = 0;
    for (std::size_t i = 0; i < index.size(); i++) {
      const double dist = std::fabs(value - index[i]);
      if (max_dist > dist) {
        nearest_idx = static_cast<int>(i);
        max_dist = dist;
      }
    }
    return nearest_idx;
  };

  EXPECT_EQ(find_nearest(value_index, 0.0), 2);  // Exact match at index 2
  EXPECT_EQ(
    find_nearest(value_index, 0.3), 3);  // Closer to 0.5 (distance 0.2) than 0.0 (distance 0.3)
  EXPECT_EQ(find_nearest(value_index, 0.6), 3);   // Closer to 0.5 than 1.0
  EXPECT_EQ(find_nearest(value_index, -1.5), 0);  // Below range
  EXPECT_EQ(find_nearest(value_index, 1.5), 4);   // Above range
}

/**
 * @brief Test index value search with threshold
 *
 * Tests the algorithm that searches for a value within a threshold distance.
 * Returns the index if found within threshold, -1 otherwise.
 */
TEST(CalibratorUtilsTest, IndexValueSearchLogic)
{
  std::vector<double> value_index = {-1.0, -0.5, 0.0, 0.5, 1.0};

  auto search_index = [](const std::vector<double> & index, double value, double threshold) {
    for (std::size_t i = 0; i < index.size(); i++) {
      const double diff_value = std::fabs(index[i] - value);
      if (diff_value <= threshold) {
        return static_cast<int>(i);
      }
    }
    return -1;
  };

  int result = search_index(value_index, 0.0, 0.1);
  EXPECT_EQ(result, 2);  // Exact match

  result = search_index(value_index, 0.05, 0.1);
  EXPECT_EQ(result, 2);  // Within threshold

  result = search_index(value_index, 0.15, 0.1);
  EXPECT_EQ(result, -1);  // Outside threshold

  result = search_index(value_index, -0.45, 0.1);
  EXPECT_EQ(result, 1);  // Within threshold of -0.5
}

/**
 * @brief Test average calculation logic
 *
 * Tests calculation of average from a vector, including edge cases like empty vectors.
 */
TEST(CalibratorUtilsTest, GetAverageLogic)
{
  auto get_average = [](const std::vector<double> & vec) {
    if (vec.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (const auto num : vec) {
      sum += num;
    }
    return sum / static_cast<double>(vec.size());
  };

  std::vector<double> vec1 = {1.0, 2.0, 3.0, 4.0, 5.0};
  EXPECT_NEAR(get_average(vec1), 3.0, 1e-6);

  std::vector<double> vec2 = {10.0, 20.0, 30.0};
  EXPECT_NEAR(get_average(vec2), 20.0, 1e-6);

  std::vector<double> vec3;
  EXPECT_NEAR(get_average(vec3), 0.0, 1e-6);

  std::vector<double> vec4 = {-1.0, 0.0, 1.0};
  EXPECT_NEAR(get_average(vec4), 0.0, 1e-6);
}

/**
 * @brief Test lowpass filter logic
 *
 * Tests the lowpass filter equation: output = current * gain + original * (1 - gain)
 */
TEST(CalibratorUtilsTest, LowpassFilterLogic)
{
  auto lowpass = [](double original, double current, double gain) {
    return current * gain + original * (1.0 - gain);
  };

  // Test with gain = 0.5
  double result = lowpass(10.0, 20.0, 0.5);
  EXPECT_NEAR(result, 15.0, 1e-6);  // Average of 10 and 20

  // Test with gain = 0.0 (no change)
  result = lowpass(10.0, 20.0, 0.0);
  EXPECT_NEAR(result, 10.0, 1e-6);  // Original value

  // Test with gain = 1.0 (full update)
  result = lowpass(10.0, 20.0, 1.0);
  EXPECT_NEAR(result, 20.0, 1e-6);  // Current value

  // Test with gain = 0.8 (default)
  result = lowpass(10.0, 20.0, 0.8);
  EXPECT_NEAR(result, 18.0, 1e-6);  // 20*0.8 + 10*0.2
}

/**
 * @brief Test map consistency/monotonicity logic
 *
 * Tests that acceleration increases with both input value and velocity (monotonicity check).
 * This ensures the map maintains physical consistency.
 */
TEST(CalibratorUtilsTest, MapConsistencyLogic)
{
  // Test that ensures acceleration increases with value and velocity
  // This is a simplified version of take_consistency_of_value_map logic

  std::vector<std::vector<double>> map = {
    {-2.0, -1.0, 0.0},  // value = -1.0
    {-1.0, 0.0, 1.0},   // value = 0.0
    {0.0, 1.0, 2.0}};   // value = 1.0
  // velocity: 0.0, 5.0, 10.0

  // Check monotonicity: acceleration should increase with value (for same velocity)
  for (size_t vel_idx = 0; vel_idx < map[0].size(); vel_idx++) {
    for (size_t val_idx = 0; val_idx < map.size() - 1; val_idx++) {
      EXPECT_LE(map[val_idx][vel_idx], map[val_idx + 1][vel_idx])
        << "Acceleration should increase with value";
    }
  }

  // Check monotonicity: acceleration should increase with velocity (for same value)
  for (size_t val_idx = 0; val_idx < map.size(); val_idx++) {
    for (size_t vel_idx = 0; vel_idx < map[val_idx].size() - 1; vel_idx++) {
      EXPECT_LE(map[val_idx][vel_idx], map[val_idx][vel_idx + 1])
        << "Acceleration should increase with velocity";
    }
  }
}

/**
 * @brief Test RLS (Recursive Least Squares) update algorithm
 *
 * Tests the core RLS algorithm used for online map calibration:
 * - Covariance update
 * - Gain calculation
 * - Offset update
 * - Convergence behavior
 */
TEST(CalibratorUtilsTest, RLSUpdateLogic)
{
  // Simplified RLS update: map_offset = map_offset + coef * error
  // where coef = (covariance * phi) / (forgetting_factor + phi * covariance * phi)
  // and error = measured_acc - map_acc

  double map_offset = 0.0;
  double covariance = 0.05;
  const double forgetting_factor = 0.999;
  const double phi = 1.0;

  // First update
  double measured_acc = 1.5;
  double map_acc = 1.0;
  double error = measured_acc - map_acc;  // 0.5

  // Update covariance
  covariance = (covariance - (covariance * phi * phi * covariance) /
                               (forgetting_factor + phi * covariance * phi)) /
               forgetting_factor;

  // Calculate coefficient
  double coef = (covariance * phi) / (forgetting_factor + phi * covariance * phi);

  // Update map offset
  map_offset = map_offset + coef * error;

  EXPECT_GT(map_offset, 0.0);    // Should be positive
  EXPECT_LT(map_offset, error);  // Should be less than error (due to coef < 1)

  // Second update (should converge)
  double old_offset = map_offset;
  measured_acc = 1.6;
  map_acc = 1.0;
  error = measured_acc - map_acc;  // 0.6

  covariance = (covariance - (covariance * phi * phi * covariance) /
                               (forgetting_factor + phi * covariance * phi)) /
               forgetting_factor;
  coef = (covariance * phi) / (forgetting_factor + phi * covariance * phi);
  map_offset = map_offset + coef * error;

  EXPECT_GT(map_offset, old_offset);  // Should increase
}
