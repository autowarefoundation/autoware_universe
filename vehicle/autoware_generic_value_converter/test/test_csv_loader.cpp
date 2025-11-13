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

#include "autoware_generic_value_converter/csv_loader.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

/**
 * @file test_csv_loader.cpp
 * @brief Unit tests for CSVLoader class
 *
 * This test file covers CSV loading and parsing functionality:
 * - CSV file reading (success and failure cases)
 * - Column index extraction (velocity indices from first row)
 * - Row index extraction (value indices from first column)
 * - Map data extraction
 * - Map validation (empty, non-rectangular, NaN, Inf)
 * - Value clamping (within range, below min, above max, empty range)
 *
 * Test Coverage:
 * @test CSVLoaderTest.LoadValidCSV - Tests successful CSV loading
 * @test CSVLoaderTest.LoadInvalidCSV - Tests error handling for invalid files
 * @test CSVLoaderTest.GetColumnIndex - Tests velocity index extraction
 * @test CSVLoaderTest.GetRowIndex - Tests value index extraction
 * @test CSVLoaderTest.GetMap - Tests map data extraction
 * @test CSVLoaderTest.ValidateMap - Tests map validation logic
 * @test CSVLoaderTest.ClampValue - Tests value clamping functionality
 *
 * Running tests:
 *   ros2 run autoware_generic_value_converter test_csv_loader
 *   ros2 run autoware_generic_value_converter test_csv_loader --gtest_output=xml:test_results.xml
 */

class CSVLoaderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    // Create temporary test CSV file
    test_csv_path_ = "/tmp/test_value_map.csv";
    createTestCSV();
  }

  void TearDown() override
  {
    // Clean up test file
    std::remove(test_csv_path_.c_str());
    rclcpp::shutdown();
  }

  void createTestCSV()
  {
    std::ofstream file(test_csv_path_);
    file << "default,0.0,2.0,4.0,6.0,8.0,10.0\n";
    file << "-1.0,-2.0,-2.0,-2.0,-2.0,-2.0,-2.0\n";
    file << "-0.5,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0\n";
    file << "0.0,0.0,0.0,0.0,0.0,0.0,0.0\n";
    file << "0.5,1.0,1.0,1.0,1.0,1.0,1.0\n";
    file << "1.0,2.0,2.0,2.0,2.0,2.0,2.0\n";
    file.close();
  }

  std::string test_csv_path_;
};

TEST_F(CSVLoaderTest, ReadCSVSuccess)
{
  autoware::generic_value_converter::CSVLoader loader(test_csv_path_);
  std::vector<std::vector<std::string>> result;

  EXPECT_TRUE(loader.readCSV(result));
  EXPECT_EQ(result.size(), 6);     // 6 rows (header + 5 data rows)
  EXPECT_EQ(result[0].size(), 7);  // 7 columns (default + 6 velocity values)
}

TEST_F(CSVLoaderTest, ReadCSVFailure)
{
  autoware::generic_value_converter::CSVLoader loader("/nonexistent/file.csv");
  std::vector<std::vector<std::string>> result;

  EXPECT_FALSE(loader.readCSV(result));
}

TEST_F(CSVLoaderTest, GetColumnIndex)
{
  autoware::generic_value_converter::CSVLoader loader(test_csv_path_);
  std::vector<std::vector<std::string>> table;
  loader.readCSV(table);

  std::vector<double> column_index =
    autoware::generic_value_converter::CSVLoader::getColumnIndex(table);

  EXPECT_EQ(column_index.size(), 6);
  EXPECT_DOUBLE_EQ(column_index[0], 0.0);
  EXPECT_DOUBLE_EQ(column_index[1], 2.0);
  EXPECT_DOUBLE_EQ(column_index[2], 4.0);
  EXPECT_DOUBLE_EQ(column_index[5], 10.0);
}

TEST_F(CSVLoaderTest, GetRowIndex)
{
  autoware::generic_value_converter::CSVLoader loader(test_csv_path_);
  std::vector<std::vector<std::string>> table;
  loader.readCSV(table);

  std::vector<double> row_index = autoware::generic_value_converter::CSVLoader::getRowIndex(table);

  EXPECT_EQ(row_index.size(), 5);
  EXPECT_DOUBLE_EQ(row_index[0], -1.0);
  EXPECT_DOUBLE_EQ(row_index[1], -0.5);
  EXPECT_DOUBLE_EQ(row_index[2], 0.0);
  EXPECT_DOUBLE_EQ(row_index[4], 1.0);
}

TEST_F(CSVLoaderTest, GetMap)
{
  autoware::generic_value_converter::CSVLoader loader(test_csv_path_);
  std::vector<std::vector<std::string>> table;
  loader.readCSV(table);

  std::vector<std::vector<double>> map =
    autoware::generic_value_converter::CSVLoader::getMap(table);

  EXPECT_EQ(map.size(), 5);     // 5 value rows
  EXPECT_EQ(map[0].size(), 6);  // 6 velocity columns
  EXPECT_DOUBLE_EQ(map[0][0], -2.0);
  EXPECT_DOUBLE_EQ(map[2][0], 0.0);
  EXPECT_DOUBLE_EQ(map[4][5], 2.0);
}

TEST_F(CSVLoaderTest, ValidateMapSuccess)
{
  std::vector<std::vector<double>> valid_map = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};

  EXPECT_TRUE(autoware::generic_value_converter::CSVLoader::validateMap(valid_map, false));
}

TEST_F(CSVLoaderTest, ValidateMapEmpty)
{
  std::vector<std::vector<double>> empty_map;

  EXPECT_FALSE(autoware::generic_value_converter::CSVLoader::validateMap(empty_map, false));
}

TEST_F(CSVLoaderTest, ValidateMapNonRectangular)
{
  std::vector<std::vector<double>> non_rect_map = {{1.0, 2.0}, {3.0, 4.0, 5.0}};

  EXPECT_FALSE(autoware::generic_value_converter::CSVLoader::validateMap(non_rect_map, false));
}

TEST_F(CSVLoaderTest, ValidateMapWithNaN)
{
  std::vector<std::vector<double>> nan_map = {{1.0, 2.0}, {std::nan(""), 4.0}};

  EXPECT_FALSE(autoware::generic_value_converter::CSVLoader::validateMap(nan_map, false));
}

TEST_F(CSVLoaderTest, ValidateMapWithInf)
{
  std::vector<std::vector<double>> inf_map = {
    {1.0, 2.0}, {std::numeric_limits<double>::infinity(), 4.0}};

  EXPECT_FALSE(autoware::generic_value_converter::CSVLoader::validateMap(inf_map, false));
}

TEST_F(CSVLoaderTest, ClampValueWithinRange)
{
  std::vector<double> ranges = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
  double value = 5.0;

  double clamped = autoware::generic_value_converter::CSVLoader::clampValue(value, ranges, "test");

  EXPECT_DOUBLE_EQ(clamped, 5.0);
}

TEST_F(CSVLoaderTest, ClampValueBelowMin)
{
  std::vector<double> ranges = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
  double value = -1.0;

  double clamped = autoware::generic_value_converter::CSVLoader::clampValue(value, ranges, "test");

  EXPECT_DOUBLE_EQ(clamped, 0.0);  // Clamped to min
}

TEST_F(CSVLoaderTest, ClampValueAboveMax)
{
  std::vector<double> ranges = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
  double value = 15.0;

  double clamped = autoware::generic_value_converter::CSVLoader::clampValue(value, ranges, "test");

  EXPECT_DOUBLE_EQ(clamped, 10.0);  // Clamped to max
}

TEST_F(CSVLoaderTest, ClampValueEmptyRange)
{
  std::vector<double> empty_ranges;
  double value = 5.0;

  double clamped =
    autoware::generic_value_converter::CSVLoader::clampValue(value, empty_ranges, "test");

  EXPECT_DOUBLE_EQ(clamped, 5.0);  // Returns original value when range is empty
}
