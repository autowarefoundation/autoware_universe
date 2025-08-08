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

#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::test
{

class PreprocessingUtilsEdgeCaseTest : public ::testing::Test
{
protected:
  void SetUp() override {}
};

// Test edge case: Empty input data map
TEST_F(PreprocessingUtilsEdgeCaseTest, NormalizeEmptyInputData)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  // Should handle empty data without crashing
  EXPECT_NO_THROW(preprocess::normalize_input_data(input_data_map, normalization_map));
}

// Test edge case: Mismatched dimensions
TEST_F(PreprocessingUtilsEdgeCaseTest, NormalizeMismatchedDimensions)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  // Input has 3 columns worth of data (2 rows x 3 cols = 6 values)
  input_data_map["f"] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
  // But normalization has 2 columns
  normalization_map["f"] = {{1.0f, 2.0f}, {1.0f, 1.0f}};

  // This should handle the mismatch gracefully
  EXPECT_NO_THROW(preprocess::normalize_input_data(input_data_map, normalization_map));
}

// Test edge case: Extreme values (very large and very small)
TEST_F(PreprocessingUtilsEdgeCaseTest, NormalizeExtremeValues)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  input_data_map["f"] = {
    std::numeric_limits<float>::max(), std::numeric_limits<float>::min(),
    -std::numeric_limits<float>::max(), std::numeric_limits<float>::epsilon()};
  normalization_map["f"] = {{0.0f}, {1.0f}};

  preprocess::normalize_input_data(input_data_map, normalization_map);

  // Check that extreme values don't cause overflow
  EXPECT_TRUE(std::isfinite(input_data_map["f"][0]));
  EXPECT_TRUE(std::isfinite(input_data_map["f"][1]));
  EXPECT_TRUE(std::isfinite(input_data_map["f"][2]));
  EXPECT_TRUE(std::isfinite(input_data_map["f"][3]));
}

// Test edge case: NaN and Inf in input data
TEST_F(PreprocessingUtilsEdgeCaseTest, NormalizeNaNInfInput)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  input_data_map["f"] = {
    std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
    -std::numeric_limits<float>::infinity(), 1.0f};
  normalization_map["f"] = {{0.0f}, {1.0f}};

  preprocess::normalize_input_data(input_data_map, normalization_map);

  // NaN and Inf should propagate through normalization
  EXPECT_TRUE(std::isnan(input_data_map["f"][0]));
  EXPECT_TRUE(std::isinf(input_data_map["f"][1]));
  EXPECT_TRUE(std::isinf(input_data_map["f"][2]));
  EXPECT_FLOAT_EQ(input_data_map["f"][3], 1.0f);  // (1-0)/1 = 1
}

// Test edge case: NaN and Inf in normalization parameters
TEST_F(PreprocessingUtilsEdgeCaseTest, NormalizeNaNInfParameters)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  input_data_map["f"] = {1.0f, 2.0f, 3.0f};
  normalization_map["f"] = {
    {std::numeric_limits<float>::quiet_NaN(), 0.0f, std::numeric_limits<float>::infinity()},
    {1.0f, std::numeric_limits<float>::infinity(), 0.0f}};

  EXPECT_THROW(
    preprocess::normalize_input_data(input_data_map, normalization_map), std::runtime_error);
}

// Test edge case: Very small standard deviation (near zero but not zero)
TEST_F(PreprocessingUtilsEdgeCaseTest, NormalizeVerySmallStdDev)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  input_data_map["f"] = {1.0f, 1.0001f};
  normalization_map["f"] = {
    {1.0f, 1.0f}, {std::numeric_limits<float>::epsilon(), std::numeric_limits<float>::epsilon()}};

  preprocess::normalize_input_data(input_data_map, normalization_map);

  // Should produce very large values but not infinity
  EXPECT_TRUE(std::isfinite(input_data_map["f"][0]));
  EXPECT_TRUE(std::isfinite(input_data_map["f"][1]));
  EXPECT_GT(std::abs(input_data_map["f"][1]), 100.0f);  // Large but finite
}

// Test edge case: Negative standard deviation (invalid but should be handled)
TEST_F(PreprocessingUtilsEdgeCaseTest, NormalizeNegativeStdDev)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  input_data_map["f"] = {5.0f, 10.0f};
  normalization_map["f"] = {{0.0f, 0.0f}, {-1.0f, -2.0f}};  // Negative std dev

  preprocess::normalize_input_data(input_data_map, normalization_map);

  // Result should be negative (5-0)/(-1) = -5
  EXPECT_FLOAT_EQ(input_data_map["f"][0], -5.0f);
  EXPECT_FLOAT_EQ(input_data_map["f"][1], -5.0f);
}

// Test edge case: Single value in data
TEST_F(PreprocessingUtilsEdgeCaseTest, NormalizeSingleValue)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  input_data_map["f"] = {42.0f};
  normalization_map["f"] = {{10.0f}, {5.0f}};

  preprocess::normalize_input_data(input_data_map, normalization_map);

  EXPECT_FLOAT_EQ(input_data_map["f"][0], 6.4f);  // (42-10)/5 = 6.4
}

// Test edge case: Very long feature name
TEST_F(PreprocessingUtilsEdgeCaseTest, NormalizeVeryLongFeatureName)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  std::string long_name(1000, 'a');
  input_data_map[long_name] = {1.0f, 2.0f};
  normalization_map[long_name] = {{0.0f, 0.0f}, {1.0f, 1.0f}};

  EXPECT_NO_THROW(preprocess::normalize_input_data(input_data_map, normalization_map));

  EXPECT_FLOAT_EQ(input_data_map[long_name][0], 1.0f);
  EXPECT_FLOAT_EQ(input_data_map[long_name][1], 2.0f);
}

// Test edge case: Unicode in feature names
TEST_F(PreprocessingUtilsEdgeCaseTest, NormalizeUnicodeFeatureName)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  std::string unicode_name = "特徴_feature_🚗";
  input_data_map[unicode_name] = {3.0f};
  normalization_map[unicode_name] = {{1.0f}, {2.0f}};

  EXPECT_NO_THROW(preprocess::normalize_input_data(input_data_map, normalization_map));

  EXPECT_FLOAT_EQ(input_data_map[unicode_name][0], 1.0f);  // (3-1)/2 = 1
}

// Test edge case: Multiple features with interdependencies
TEST_F(PreprocessingUtilsEdgeCaseTest, NormalizeMultipleFeaturesOrder)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  // Create multiple features
  input_data_map["a"] = {1.0f, 2.0f};
  input_data_map["b"] = {3.0f, 4.0f};
  input_data_map["c"] = {5.0f, 6.0f};

  normalization_map["a"] = {{0.0f, 0.0f}, {1.0f, 1.0f}};
  normalization_map["b"] = {{1.0f, 1.0f}, {2.0f, 2.0f}};
  normalization_map["c"] = {{2.0f, 2.0f}, {3.0f, 3.0f}};

  preprocess::normalize_input_data(input_data_map, normalization_map);

  // Verify each feature is normalized independently
  EXPECT_FLOAT_EQ(input_data_map["a"][0], 1.0f);         // (1-0)/1 = 1
  EXPECT_FLOAT_EQ(input_data_map["a"][1], 2.0f);         // (2-0)/1 = 2
  EXPECT_FLOAT_EQ(input_data_map["b"][0], 1.0f);         // (3-1)/2 = 1
  EXPECT_FLOAT_EQ(input_data_map["b"][1], 1.5f);         // (4-1)/2 = 1.5
  EXPECT_FLOAT_EQ(input_data_map["c"][0], 1.0f);         // (5-2)/3 = 1
  EXPECT_FLOAT_EQ(input_data_map["c"][1], 4.0f / 3.0f);  // (6-2)/3 = 4/3
}

// Test edge case: Row-wise operations with partial zero rows
TEST_F(PreprocessingUtilsEdgeCaseTest, NormalizePartialZeroRows)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  // 3 rows, 2 cols - middle row has one zero
  input_data_map["f"] = {1.0f, 2.0f,   // row 0: non-zero
                         0.0f, 3.0f,   // row 1: partial zero
                         0.0f, 0.0f};  // row 2: all zero
  normalization_map["f"] = {{1.0f, 1.0f}, {1.0f, 1.0f}};

  preprocess::normalize_input_data(input_data_map, normalization_map);

  // First row: normalized normally
  EXPECT_FLOAT_EQ(input_data_map["f"][0], 0.0f);  // (1-1)/1 = 0
  EXPECT_FLOAT_EQ(input_data_map["f"][1], 1.0f);  // (2-1)/1 = 1

  // Second row: normalized normally (not all zeros)
  EXPECT_FLOAT_EQ(input_data_map["f"][2], -1.0f);  // (0-1)/1 = -1
  EXPECT_FLOAT_EQ(input_data_map["f"][3], 2.0f);   // (3-1)/1 = 2

  // Third row: should remain all zeros
  EXPECT_FLOAT_EQ(input_data_map["f"][4], 0.0f);
  EXPECT_FLOAT_EQ(input_data_map["f"][5], 0.0f);
}

// Test edge case: Create float data utility
TEST_F(PreprocessingUtilsEdgeCaseTest, CreateFloatDataEdgeCases)
{
  // Test with empty shape - actually creates vector of size 1
  std::vector<int64_t> empty_shape{};
  auto empty_data = utils::create_float_data(empty_shape, 1.0f);
  EXPECT_EQ(empty_data.size(), 1);  // Empty shape results in scalar

  // Test with zero dimension
  std::vector<int64_t> zero_shape{0, 5, 10};
  auto zero_data = utils::create_float_data(zero_shape, 2.0f);
  EXPECT_TRUE(zero_data.empty());

  // Test with very large shape (but manageable)
  std::vector<int64_t> large_shape{100, 100};
  auto large_data = utils::create_float_data(large_shape, 3.0f);
  EXPECT_EQ(large_data.size(), 10000);
  EXPECT_FLOAT_EQ(large_data[0], 3.0f);
  EXPECT_FLOAT_EQ(large_data[9999], 3.0f);

  // Test with negative dimension (should be handled as unsigned)
  std::vector<int64_t> negative_shape{-5, 10};
  EXPECT_ANY_THROW(utils::create_float_data(negative_shape, 4.0f));
}

}  // namespace autoware::diffusion_planner::test
