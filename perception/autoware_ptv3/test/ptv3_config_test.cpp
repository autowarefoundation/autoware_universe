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
  const std::vector<float> & yaw_norm_thresholds = {0.1F, 0.2F},
  const std::vector<float> & voxel_size = {1.0F, 1.0F, 1.0F},
  const std::vector<std::int64_t> & voxels_num = {1, 4, 8})
{
  return PTv3Config(
    false, true, "", 8, voxels_num, point_cloud_range, voxel_size, {}, {"z", "z-trans"},
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

// [0.5, 16.5) with unit voxels emits coordinates 0..16; a depth sized for 16 cells would drop the
// boundary coordinate's top Morton bit and merge its voxels with coordinate 0's.
TEST(PTv3ConfigTest, SerializationDepthCoversUnalignedRangeBoundary)
{
  const auto aligned = makeDetectionConfig();
  EXPECT_EQ(aligned.serialization_depth_, 4);

  const auto unaligned = makeDetectionConfig({0.5F, 0.5F, 0.5F, 16.5F, 16.5F, 4.5F});
  EXPECT_EQ(unaligned.serialization_depth_, 5);
}

// The same boundary coordinates count toward the per-stage voxel bound: 17 x 17 x 5 cells at
// stage 0, ceil'd per stride-2 stage. A 16 x 16 x 4 bound would under-size the encoder stage
// buffers and TensorRT profiles.
TEST(PTv3ConfigTest, StageVoxelCapacityCoversUnalignedRangeBoundary)
{
  const auto config = makeDetectionConfig(
    {0.5F, 0.5F, 0.5F, 16.5F, 16.5F, 4.5F}, {8.0F, 8.0F, 4.0F}, {10.0F, 20.0F},
    {0.1F, 0.2F, 0.3F, 0.4F}, {0.1F, 0.2F}, {1.0F, 1.0F, 1.0F}, {1, 1024, 4096});
  EXPECT_EQ(config.stage_voxel_capacity(0), 17 * 17 * 5);
  EXPECT_EQ(config.stage_voxel_capacity(1), 9 * 9 * 3);
  EXPECT_EQ(config.stage_voxel_capacity(4), 2 * 2 * 1);
}

// Borders that are voxel-aligned in decimal but not exactly representable in binary (neither 102.4
// nor 0.1 is a binary float) must not gain a spurious extra coordinate from rounding.
TEST(PTv3ConfigTest, SerializationDepthStaysExactForBase10AlignedRanges)
{
  const auto config = makeDetectionConfig(
    {-102.4F, -102.4F, -0.4F, 102.4F, 102.4F, 0.4F}, {0.8F, 0.8F, 4.0F}, {10.0F, 20.0F},
    {0.1F, 0.2F, 0.3F, 0.4F}, {0.1F, 0.2F}, {0.1F, 0.1F, 0.1F});
  EXPECT_EQ(config.serialization_depth_, 11);
}

}  // namespace test
}  // namespace autoware::ptv3
