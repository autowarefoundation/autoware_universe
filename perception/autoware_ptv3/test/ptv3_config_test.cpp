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
  const std::vector<float> & yaw_norm_thresholds = {0.1F, 0.2F})
{
  return PTv3Config(
    false, true, "", 8, {1, 4, 8}, point_cloud_range, {1.0F, 1.0F, 1.0F}, {}, {}, {"z", "z-trans"},
    {2, 2, 2, 2}, {8, 16, 32, 64, 128}, {}, {}, "", false, "", {}, {"CAR", "PEDESTRIAN"},
    bbox_voxel_size, distance_bin_upper_limits, detection_score_thresholds, yaw_norm_thresholds,
    true, 8, {-2.0F, -2.0F, -2.0F, 4.0F, 4.0F, 4.0F});
}

// Segmentation-only config exercising segmentation3d.class_remap resolution.
PTv3Config makeSegmentationConfig(
  const std::vector<std::string> & segmentation_class_names,
  const std::unordered_map<std::string, std::string> & segmentation_class_remaps)
{
  std::vector<std::int64_t> palette(segmentation_class_names.size() * 3, 0);
  return PTv3Config(
    true, false, "", 8, {1, 4, 8}, {-1.0F, -1.0F, -1.0F, 3.0F, 3.0F, 3.0F}, {1.0F, 1.0F, 1.0F},
    segmentation_class_names, segmentation_class_remaps, {"z", "z-trans"}, {2, 2}, {8, 16, 32},
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

  const auto config = makeSegmentationConfig(
    {"car", "traffic_cone", "drivable_flat"},
    {{"car", "CAR"}, {"traffic_cone", "HAZARD"}, {"drivable_flat", "FLAT_SURFACE"}});

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

TEST(PTv3ConfigTest, HonorsClassificationRemapOverride)
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

TEST(PTv3ConfigTest, RejectsClassNameMissingFromRemap)
{
  EXPECT_THROW(makeSegmentationConfig({"car", "truck"}, {{"car", "CAR"}}), std::runtime_error);
}

TEST(PTv3ConfigTest, IgnoresRemapEntriesForClassesTheModelDoesNotOutput)
{
  using autoware::point_types::PointCloudClassification;

  // The remap may cover more classes than class_names, e.g. one param file shared across model
  // variants that output different class subsets.
  const auto config = makeSegmentationConfig(
    {"car"}, {{"car", "CAR"}, {"truck", "TRUCK"}, {"vegetation", "VEGETATION"}});

  ASSERT_EQ(config.class_id_to_classification_.size(), 1U);
  EXPECT_EQ(
    config.class_id_to_classification_[0],
    static_cast<std::uint8_t>(PointCloudClassification::CAR));
}

}  // namespace test
}  // namespace autoware::ptv3
