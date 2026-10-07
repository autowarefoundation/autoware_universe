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

#include "autoware_generic_value_converter/value_map.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>

/**
 * @file test_value_map.cpp
 * @brief Unit tests for ValueMap class - bilinear interpolation and map lookups
 *
 * This test file covers value-acceleration mapping functionality:
 * - CSV map loading
 * - Exact value lookups (no interpolation needed)
 * - Bilinear interpolation for intermediate values
 * - Boundary handling (below min, above max acceleration)
 * - Velocity interpolation across velocity dimension
 * - Empty map handling
 * - Acceleration-to-value conversion (inverse lookup)
 * - Value-to-acceleration conversion (forward lookup)
 *
 * Test Coverage:
 * @test ValueMapTest.LoadMap - Tests CSV map loading
 * @test ValueMapTest.GetValueExact - Tests exact value lookups
 * @test ValueMapTest.GetValueInterpolation - Tests bilinear interpolation
 * @test ValueMapTest.GetValueBoundary - Tests boundary condition handling
 * @test ValueMapTest.GetValueVelocityInterpolation - Tests velocity dimension interpolation
 * @test ValueMapTest.GetValueEmptyMap - Tests empty map handling
 * @test ValueMapTest.GetAcceleration - Tests value-to-acceleration conversion
 *
 * Running tests:
 *   ros2 run autoware_generic_value_converter test_value_map
 *   ros2 run autoware_generic_value_converter test_value_map --gtest_output=xml:test_results.xml
 *
 * @note Tests create temporary CSV files in /tmp and clean them up automatically
 */

class ValueMapTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    // Create test CSV file with known values for interpolation testing
    test_csv_path_ = "/tmp/test_value_map_interp.csv";
    createTestCSV();
  }

  void TearDown() override
  {
    std::remove(test_csv_path_.c_str());
    rclcpp::shutdown();
  }

  void createTestCSV()
  {
    // Create a CSV with a simple linear relationship: accel = 2.0 * value
    // This makes it easy to verify interpolation correctness
    std::ofstream file(test_csv_path_);
    file << "default,0.0,5.0,10.0,15.0,20.0\n";
    file << "-1.0,-2.0,-2.0,-2.0,-2.0,-2.0\n";
    file << "-0.5,-1.0,-1.0,-1.0,-1.0,-1.0\n";
    file << "0.0,0.0,0.0,0.0,0.0,0.0\n";
    file << "0.5,1.0,1.0,1.0,1.0,1.0\n";
    file << "1.0,2.0,2.0,2.0,2.0,2.0\n";
    file.close();
  }

  std::string test_csv_path_;
};

TEST_F(ValueMapTest, ReadValueMapFromCSVSuccess)
{
  autoware::generic_value_converter::ValueMap value_map;

  EXPECT_TRUE(value_map.readValueMapFromCSV(test_csv_path_, true));

  auto vel_idx = value_map.getVelIdx();
  auto value_idx = value_map.getValueIdx();

  EXPECT_EQ(vel_idx.size(), 5);
  EXPECT_EQ(value_idx.size(), 5);
  EXPECT_DOUBLE_EQ(vel_idx[0], 0.0);
  EXPECT_DOUBLE_EQ(value_idx[0], -1.0);
}

TEST_F(ValueMapTest, ReadValueMapFromCSVFailure)
{
  autoware::generic_value_converter::ValueMap value_map;

  EXPECT_FALSE(value_map.readValueMapFromCSV("/nonexistent/file.csv", true));
}

TEST_F(ValueMapTest, GetValueExactMatch)
{
  autoware::generic_value_converter::ValueMap value_map;
  value_map.readValueMapFromCSV(test_csv_path_, true);

  double value = 0.0;
  // Test exact match: acc=0.0, vel=0.0 should return value=0.0
  EXPECT_TRUE(value_map.getValue(0.0, 0.0, value));
  EXPECT_NEAR(value, 0.0, 1e-6);
}

TEST_F(ValueMapTest, GetValueInterpolation)
{
  autoware::generic_value_converter::ValueMap value_map;
  value_map.readValueMapFromCSV(test_csv_path_, true);

  double value = 0.0;
  // Test interpolation: acc=1.0, vel=0.0
  // At value=0.0, acc=0.0; at value=0.5, acc=1.0
  // So acc=1.0 should return value=0.5
  EXPECT_TRUE(value_map.getValue(1.0, 0.0, value));
  EXPECT_NEAR(value, 0.5, 0.1);  // Allow some tolerance for interpolation
}

TEST_F(ValueMapTest, GetValueBelowMinAcceleration)
{
  autoware::generic_value_converter::ValueMap value_map;
  value_map.readValueMapFromCSV(test_csv_path_, true);

  double value = 0.0;
  // Test below minimum: acc=-5.0 (below min accel of -2.0)
  EXPECT_TRUE(value_map.getValue(-5.0, 0.0, value));
  EXPECT_DOUBLE_EQ(value, -1.0);  // Should return min value index
}

TEST_F(ValueMapTest, GetValueAboveMaxAcceleration)
{
  autoware::generic_value_converter::ValueMap value_map;
  value_map.readValueMapFromCSV(test_csv_path_, true);

  double value = 0.0;
  // Test above maximum: acc=5.0 (above max accel of 2.0)
  EXPECT_TRUE(value_map.getValue(5.0, 0.0, value));
  EXPECT_DOUBLE_EQ(value, 1.0);  // Should return max value index
}

TEST_F(ValueMapTest, GetValueVelocityInterpolation)
{
  autoware::generic_value_converter::ValueMap value_map;
  value_map.readValueMapFromCSV(test_csv_path_, true);

  double value = 0.0;
  // Test velocity interpolation: acc=1.0, vel=2.5 (between 0.0 and 5.0)
  EXPECT_TRUE(value_map.getValue(1.0, 2.5, value));
  // Should still return approximately 0.5 (since map is constant across velocities)
  EXPECT_NEAR(value, 0.5, 0.1);
}

TEST_F(ValueMapTest, GetValueEmptyMap)
{
  autoware::generic_value_converter::ValueMap value_map;
  // Don't load map, so it's empty

  double value = 0.0;
  EXPECT_FALSE(value_map.getValue(1.0, 0.0, value));
}

TEST_F(ValueMapTest, GetAccelerationExactMatch)
{
  autoware::generic_value_converter::ValueMap value_map;
  value_map.readValueMapFromCSV(test_csv_path_, true);

  double acc = 0.0;
  // Test exact match: value=0.0, vel=0.0 should return acc=0.0
  EXPECT_TRUE(value_map.getAcceleration(0.0, 0.0, acc));
  EXPECT_NEAR(acc, 0.0, 1e-6);
}

TEST_F(ValueMapTest, GetAccelerationInterpolation)
{
  autoware::generic_value_converter::ValueMap value_map;
  value_map.readValueMapFromCSV(test_csv_path_, true);

  double acc = 0.0;
  // Test interpolation: value=0.25, vel=0.0
  // At value=0.0, acc=0.0; at value=0.5, acc=1.0
  // So value=0.25 should return acc=0.5
  EXPECT_TRUE(value_map.getAcceleration(0.25, 0.0, acc));
  EXPECT_NEAR(acc, 0.5, 0.1);
}

TEST_F(ValueMapTest, GetAccelerationVelocityInterpolation)
{
  autoware::generic_value_converter::ValueMap value_map;
  value_map.readValueMapFromCSV(test_csv_path_, true);

  double acc = 0.0;
  // Test velocity interpolation: value=0.5, vel=2.5 (between 0.0 and 5.0)
  EXPECT_TRUE(value_map.getAcceleration(0.5, 2.5, acc));
  // Should return approximately 1.0 (since map is constant across velocities)
  EXPECT_NEAR(acc, 1.0, 0.1);
}

TEST_F(ValueMapTest, GetAccelerationEmptyMap)
{
  autoware::generic_value_converter::ValueMap value_map;
  // Don't load map, so it's empty

  double acc = 0.0;
  EXPECT_FALSE(value_map.getAcceleration(0.5, 0.0, acc));
}

TEST_F(ValueMapTest, GetAccelerationClampedValue)
{
  autoware::generic_value_converter::ValueMap value_map;
  value_map.readValueMapFromCSV(test_csv_path_, true);

  double acc = 0.0;
  // Test with value outside range: value=2.0 (above max of 1.0)
  EXPECT_TRUE(value_map.getAcceleration(2.0, 0.0, acc));
  EXPECT_NEAR(acc, 2.0, 0.1);  // Should clamp to max value's acceleration
}

TEST_F(ValueMapTest, GetAccelerationClampedVelocity)
{
  autoware::generic_value_converter::ValueMap value_map;
  value_map.readValueMapFromCSV(test_csv_path_, true);

  double acc = 0.0;
  // Test with velocity outside range: value=0.5, vel=25.0 (above max of 20.0)
  EXPECT_TRUE(value_map.getAcceleration(0.5, 25.0, acc));
  EXPECT_NEAR(acc, 1.0, 0.1);  // Should clamp to max velocity
}
