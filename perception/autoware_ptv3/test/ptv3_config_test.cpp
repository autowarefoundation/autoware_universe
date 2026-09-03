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

#include <autoware/point_types/types.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
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
    false, true, "", 8, voxels_num, point_cloud_range, voxel_size, {}, {}, {"z", "z-trans"},
    {2, 2, 2, 2}, {8, 16, 32, 64, 128}, {}, {}, "", false, "", {}, {"CAR", "PEDESTRIAN"},
    bbox_voxel_size, distance_bin_upper_limits, detection_score_thresholds, yaw_norm_thresholds,
    true, 8, {-2.0F, -2.0F, -2.0F, 4.0F, 4.0F, 4.0F});
}

// Segmentation-only config exercising segmentation3d.class_mapping resolution.
PTv3Config makeSegmentationConfig(
  const std::vector<std::string> & segmentation_class_names,
  const std::unordered_map<std::string, std::string> & segmentation_class_mapping)
{
  std::vector<std::int64_t> palette(segmentation_class_names.size() * 3, 0);
  return PTv3Config(
    true, false, "", 8, {1, 4, 8}, {-1.0F, -1.0F, -1.0F, 3.0F, 3.0F, 3.0F}, {1.0F, 1.0F, 1.0F},
    segmentation_class_names, segmentation_class_mapping, {"z", "z-trans"}, {2, 2}, {8, 16, 32},
    palette, {}, "xyzi", false, "partial", {0, 0});
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

TEST(PTv3ConfigTest, BuildsClassificationLutInClassNameOrder)
{
  using autoware::point_types::PointCloudClassification;

  // Keep the mapping order different from class_names to verify that the LUT follows class_names.
  const auto config = makeSegmentationConfig(
    {"car", "traffic_cone", "drivable_flat"},
    {{"drivable_flat", "FLAT_SURFACE"}, {"car", "CAR"}, {"traffic_cone", "HAZARD"}});

  ASSERT_EQ(config.class_id_to_classification_.size(), 3U);
  EXPECT_EQ(
    config.class_id_to_classification_[0],
    static_cast<std::uint8_t>(PointCloudClassification::CAR));
  EXPECT_EQ(
    config.class_id_to_classification_[1],
    static_cast<std::uint8_t>(PointCloudClassification::HAZARD));
  EXPECT_EQ(
    config.class_id_to_classification_[2],
    static_cast<std::uint8_t>(PointCloudClassification::FLAT_SURFACE));
}

TEST(PTv3ConfigTest, HonorsClassificationMappingOverride)
{
  using autoware::point_types::PointCloudClassification;

  // traffic_cone used to be hard-coded to HAZARD; the parameter must be able to override it.
  const auto config = makeSegmentationConfig({"traffic_cone"}, {{"traffic_cone", "STRUCTURE"}});

  ASSERT_EQ(config.class_id_to_classification_.size(), 1U);
  EXPECT_EQ(
    config.class_id_to_classification_[0],
    static_cast<std::uint8_t>(PointCloudClassification::STRUCTURE));
}

TEST(PTv3ConfigTest, AcceptsClassificationNamesRegardlessOfCase)
{
  using autoware::point_types::PointCloudClassification;

  const auto config = makeSegmentationConfig(
    {"drivable_flat", "noise"}, {{"drivable_flat", "flat_surface"}, {"noise", "Noise"}});

  ASSERT_EQ(config.class_id_to_classification_.size(), 2U);
  EXPECT_EQ(
    config.class_id_to_classification_[0],
    static_cast<std::uint8_t>(PointCloudClassification::FLAT_SURFACE));
  EXPECT_EQ(
    config.class_id_to_classification_[1],
    static_cast<std::uint8_t>(PointCloudClassification::NOISE));
}

TEST(PTv3ConfigTest, RejectsUnknownClassificationName)
{
  EXPECT_THROW(makeSegmentationConfig({"car"}, {{"car", "UNKNOWN"}}), std::invalid_argument);
}

TEST(PTv3ConfigTest, RejectsClassNameMissingFromMapping)
{
  EXPECT_THROW(makeSegmentationConfig({"car", "truck"}, {{"car", "CAR"}}), std::runtime_error);
}

TEST(PTv3ConfigTest, IgnoresMappingEntriesForClassesTheModelDoesNotOutput)
{
  using autoware::point_types::PointCloudClassification;

  // The mapping may cover more classes than class_names, e.g. one param file shared across model
  // variants that output different class subsets.
  const auto config = makeSegmentationConfig(
    {"car"}, {{"car", "CAR"}, {"truck", "TRUCK"}, {"vegetation", "VEGETATION"}});

  ASSERT_EQ(config.class_id_to_classification_.size(), 1U);
  EXPECT_EQ(
    config.class_id_to_classification_[0],
    static_cast<std::uint8_t>(PointCloudClassification::CAR));
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
