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

#include "autoware/ptv3/ptv3_config.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

namespace autoware::ptv3
{
namespace test
{

PTv3Config makeDetectionConfig(
  const std::vector<float> & point_cloud_range = {0.0F, 0.0F, 0.0F, 16.0F, 16.0F, 4.0F},
  const std::vector<float> & bbox_voxel_size = {8.0F, 8.0F, 4.0F},
  const std::vector<float> & distance_bin_upper_limits = {10.0F, 20.0F},
  const std::vector<float> & detection_score_thresholds = {0.1F, 0.2F, 0.3F, 0.4F},
  const std::vector<float> & yaw_norm_thresholds = {0.1F, 0.2F})
{
  return PTv3Config(
    false, true, "", 8, {1, 4, 8}, point_cloud_range, {1.0F, 1.0F, 1.0F}, {}, {"z", "z-trans"},
    {2, 2, 2, 2}, {8, 16, 32, 64, 128}, {}, {}, "", false, "", {}, {"CAR", "PEDESTRIAN"},
    bbox_voxel_size, distance_bin_upper_limits, detection_score_thresholds, yaw_norm_thresholds,
    true, 8, {-2.0F, -2.0F, -2.0F, 4.0F, 4.0F, 4.0F});
}

TEST(PTv3ConfigTest, AcceptsCompatibleDetectionGrid)
{
  const auto config = makeDetectionConfig();
  EXPECT_EQ(config.det_grid_x_size_, 2U);
  EXPECT_EQ(config.det_grid_y_size_, 2U);
}

TEST(PTv3ConfigTest, RejectsDetectionGridThatDoesNotMatchFeatureDepth)
{
  EXPECT_THROW(
    makeDetectionConfig({0.0F, 0.0F, 0.0F, 16.0F, 16.0F, 4.0F}, {4.0F, 8.0F, 4.0F}),
    std::runtime_error);
}

TEST(PTv3ConfigTest, RejectsDetectionGridThatDoesNotCoverVoxelGridExactly)
{
  EXPECT_THROW(makeDetectionConfig({0.0F, 0.0F, 0.0F, 18.0F, 16.0F, 4.0F}), std::runtime_error);
}

TEST(PTv3ConfigTest, RejectsInvalidDetectionThresholdTables)
{
  EXPECT_THROW(
    makeDetectionConfig({0.0F, 0.0F, 0.0F, 16.0F, 16.0F, 4.0F}, {8.0F, 8.0F, 4.0F}, {20.0F, 10.0F}),
    std::runtime_error);
  EXPECT_THROW(
    makeDetectionConfig(
      {0.0F, 0.0F, 0.0F, 16.0F, 16.0F, 4.0F}, {8.0F, 8.0F, 4.0F}, {10.0F}, {0.1F}),
    std::runtime_error);
  EXPECT_THROW(
    makeDetectionConfig(
      {0.0F, 0.0F, 0.0F, 16.0F, 16.0F, 4.0F}, {8.0F, 8.0F, 4.0F}, {10.0F, 20.0F},
      {0.1F, 0.2F, 0.3F, 0.4F}, {0.1F}),
    std::runtime_error);
}

}  // namespace test
}  // namespace autoware::ptv3
