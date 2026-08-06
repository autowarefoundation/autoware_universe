// Copyright 2026 The Autoware Contributors
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
#include "autoware/obstacle_grid_extractor/obstacle_grid_extractor.hpp"
#include "make_point_cloud.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace
{
using autoware::obstacle_grid_extractor::ExtractorParams;
using autoware::obstacle_grid_extractor::ObstacleGridExtractor;
using autoware::obstacle_grid_extractor::test::makePointCloud;
using autoware::obstacle_grid_extractor::test::makePointXyzircCloud;

// ROI: length_x 60 m centred at offset_x 20 -> x in [-10, 50]; length_y 40 m centred at 0 ->
// y in [-20, 20]. At 0.2 m resolution the cell boundaries land on multiples of 0.2, so the cell
// holding the query position (5.1, 0.1) spans [5.0, 5.2] x [0.0, 0.2].
constexpr double kResolution = 0.2;
constexpr float kCropZMin = -1.0f;
constexpr float kCropZMax = 3.0f;
constexpr float kOverheadSplit = 2.5f;
constexpr char kBaseLink[] = "base_link";

ExtractorParams testParams()
{
  ExtractorParams params;
  params.roi_length_x = 60.0;
  params.roi_length_y = 40.0;
  params.roi_offset_x = 20.0;
  params.resolution = kResolution;
  params.crop_z_min = kCropZMin;
  params.crop_z_max = kCropZMax;
  params.overhead_split = kOverheadSplit;
  return params;
}

grid_map::GridMap toGridMap(const grid_map_msgs::msg::GridMap & msg)
{
  grid_map::GridMap grid;
  grid_map::GridMapRosConverter::fromMessage(msg, grid);
  return grid;
}

grid_map::GridMap extractGrid(const sensor_msgs::msg::PointCloud2 & cloud)
{
  return toGridMap(ObstacleGridExtractor(testParams()).extract(cloud));
}
}  // namespace

TEST(ObstacleGridExtractor, TwoPointsInSameCellAccumulate)
{
  // Arrange: two points inside the single cell spanning [5.0, 5.2] x [0.0, 0.2]
  // (x = 5.0 / y = 0.0 would sit on cell boundaries, hence the 0.05 offsets).
  const auto cloud = makePointCloud(kBaseLink, {{5.05f, 0.05f, 0.5f}, {5.15f, 0.15f, 1.2f}});

  // Act
  const auto grid = extractGrid(cloud);

  // Assert: the cell holds both points, with the height envelope spanning them.
  grid_map::Index occupied_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(5.1, 0.1), occupied_cell));
  EXPECT_FLOAT_EQ(grid.at("point_count", occupied_cell), 2.0f);
  EXPECT_FLOAT_EQ(grid.at("max_height", occupied_cell), 1.2f);
  EXPECT_FLOAT_EQ(grid.at("min_height", occupied_cell), 0.5f);
}

TEST(ObstacleGridExtractor, EmptyCellsAreNaN)
{
  // Arrange: a single point near the vehicle, far from the cell queried below.
  const auto cloud = makePointCloud(kBaseLink, {{5.0f, 0.0f, 0.5f}});

  // Act
  const auto grid = extractGrid(cloud);

  // Assert: a cell no point fell into is left unobserved rather than reported as clear.
  grid_map::Index unobserved_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(30.0, 15.0), unobserved_cell));
  EXPECT_TRUE(std::isnan(grid.at("point_count", unobserved_cell)));
}

TEST(ObstacleGridExtractor, NarrowObstacleOccupiesAtLeastOneCell)
{
  // Arrange: a 0.15 m-wide vertical pole. The integer-stepped z avoids the float-accumulation
  // drift of a `z += 0.1f` loop, so the top return stays exactly 1.0 m.
  std::vector<autoware::obstacle_grid_extractor::test::TestPoint> pole;
  for (int i = 0; i <= 10; ++i) {
    pole.push_back({8.0f, 0.0f, 0.1f * static_cast<float>(i)});  // z = 0.0 .. 1.0
  }
  const auto cloud = makePointCloud(kBaseLink, pole);

  // Act
  const auto grid = extractGrid(cloud);

  // Assert: the pole is not thinner than the grid can represent — it occupies its cell.
  grid_map::Index pole_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(8.0, 0.0), pole_cell));
  EXPECT_FALSE(std::isnan(grid.at("point_count", pole_cell)));
  EXPECT_FLOAT_EQ(grid.at("max_height", pole_cell), 1.0f);
  EXPECT_FLOAT_EQ(grid.at("min_height", pole_cell), 0.0f);
}

TEST(ObstacleGridExtractor, PointsOutsideCropZBandAreDropped)
{
  // Arrange: one point well below crop_z_min and one well above crop_z_max, both in the same cell.
  const auto cloud = makePointCloud(kBaseLink, {{5.0f, 0.0f, -2.0f}, {5.0f, 0.0f, 5.0f}});

  // Act
  const auto grid = extractGrid(cloud);

  // Assert: with both points cropped the cell stays empty.
  grid_map::Index cropped_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(5.0, 0.0), cropped_cell));
  EXPECT_TRUE(std::isnan(grid.at("point_count", cropped_cell)));
}

TEST(ObstacleGridExtractor, PointsExactlyOnCropBoundsAreKept)
{
  // Arrange: points sitting exactly on crop_z_min / crop_z_max — the crop uses strict < / >.
  const auto cloud =
    makePointCloud(kBaseLink, {{5.05f, 0.05f, kCropZMin}, {5.15f, 0.15f, kCropZMax}});

  // Act
  const auto grid = extractGrid(cloud);

  // Assert
  grid_map::Index occupied_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(5.1, 0.1), occupied_cell));
  EXPECT_FLOAT_EQ(grid.at("point_count", occupied_cell), 2.0f);
  EXPECT_FLOAT_EQ(grid.at("max_height", occupied_cell), kCropZMax);
  EXPECT_FLOAT_EQ(grid.at("min_height", occupied_cell), kCropZMin);
}

TEST(ObstacleGridExtractor, PointsJustOutsideCropBoundsAreDropped)
{
  // Arrange: the same two points nudged just past each bound.
  const auto cloud = makePointCloud(
    kBaseLink, {{8.05f, 0.05f, kCropZMin - 1e-4f}, {8.15f, 0.15f, kCropZMax + 1e-4f}});

  // Act
  const auto grid = extractGrid(cloud);

  // Assert
  grid_map::Index cropped_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(8.1, 0.1), cropped_cell));
  EXPECT_TRUE(std::isnan(grid.at("point_count", cropped_cell)));
}

TEST(ObstacleGridExtractor, LowMaxHeightIgnoresOverheadReturnInMixedCell)
{
  // Arrange: a gantry-like cell — a ground return plus an overhead return above overhead_split.
  const auto cloud = makePointCloud(kBaseLink, {{5.05f, 0.05f, 0.05f}, {5.15f, 0.15f, 2.8f}});

  // Act
  const auto grid = extractGrid(cloud);

  // Assert: max_height still reports the true top, but low_max_height records only the tallest
  // in-band return — the discriminator consumers gate on to reject overhead-only structures.
  grid_map::Index gantry_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(5.1, 0.1), gantry_cell));
  EXPECT_FLOAT_EQ(grid.at("point_count", gantry_cell), 2.0f);
  EXPECT_FLOAT_EQ(grid.at("max_height", gantry_cell), 2.8f);
  EXPECT_FLOAT_EQ(grid.at("low_max_height", gantry_cell), 0.05f);
}

TEST(ObstacleGridExtractor, LowMaxHeightIsNaNWhenOnlyOverheadReturnsExist)
{
  // Arrange: a cell whose only return is above overhead_split.
  const auto cloud = makePointCloud(kBaseLink, {{8.05f, 0.05f, 2.8f}});

  // Act
  const auto grid = extractGrid(cloud);

  // Assert: the cell is observed, but nothing qualifies it as an obstacle to brake for.
  grid_map::Index overhead_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(8.1, 0.1), overhead_cell));
  EXPECT_FLOAT_EQ(grid.at("point_count", overhead_cell), 1.0f);
  EXPECT_FLOAT_EQ(grid.at("max_height", overhead_cell), 2.8f);
  EXPECT_TRUE(std::isnan(grid.at("low_max_height", overhead_cell)));
}

TEST(ObstacleGridExtractor, LowMaxHeightRecordsTallInBandReturn)
{
  // Arrange: a wall-like cell whose tall return is still at or below overhead_split.
  const auto cloud =
    makePointCloud(kBaseLink, {{11.05f, 0.05f, 0.05f}, {11.15f, 0.15f, kOverheadSplit}});

  // Act
  const auto grid = extractGrid(cloud);

  // Assert: a tall in-band obstacle is not lost to the overhead filter.
  grid_map::Index wall_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(11.1, 0.1), wall_cell));
  EXPECT_FLOAT_EQ(grid.at("low_max_height", wall_cell), kOverheadSplit);
}

TEST(ObstacleGridExtractor, NonFinitePointsAreDropped)
{
  // Arrange: a NaN-z point with finite x/y arrives FIRST in the cell. Without the finiteness guard
  // it would pass the z crop (NaN comparisons are false) and permanently poison max/min_height
  // (std::max/std::min return their first, NaN, argument).
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto cloud = makePointCloud(kBaseLink, {{5.05f, 0.05f, nan}, {5.15f, 0.15f, 0.5f}});

  // Act
  const auto grid = extractGrid(cloud);

  // Assert: the real point fully defines the cell.
  grid_map::Index occupied_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(5.1, 0.1), occupied_cell));
  EXPECT_FLOAT_EQ(grid.at("point_count", occupied_cell), 1.0f);
  EXPECT_FLOAT_EQ(grid.at("max_height", occupied_cell), 0.5f);
  EXPECT_FLOAT_EQ(grid.at("min_height", occupied_cell), 0.5f);
  EXPECT_FLOAT_EQ(grid.at("low_max_height", occupied_cell), 0.5f);
}

TEST(ObstacleGridExtractor, PointsOutsideRoiAreDropped)
{
  // Arrange: a finite, in-z-band point far outside the [-10, 50] x [-20, 20] ROI, alongside an
  // in-ROI point. The out-of-ROI point must be rejected by the getIndex() bounds check without
  // touching any cell or crashing.
  const auto cloud = makePointCloud(kBaseLink, {{1000.0f, 1000.0f, 0.5f}, {8.05f, 0.05f, 0.5f}});

  // Act
  const auto grid = extractGrid(cloud);

  // Assert: only the in-ROI point was recorded, and the far position has no cell at all.
  grid_map::Index occupied_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(8.05, 0.05), occupied_cell));
  EXPECT_FLOAT_EQ(grid.at("point_count", occupied_cell), 1.0f);
  EXPECT_FLOAT_EQ(grid.at("max_height", occupied_cell), 0.5f);
  grid_map::Index out_of_roi_cell;
  EXPECT_FALSE(grid.getIndex(grid_map::Position(1000.0, 1000.0), out_of_roi_cell));
}

TEST(ObstacleGridExtractor, EmptyCloudIsFreshHeartbeat)
{
  // Arrange: a cloud carrying no points but a fresh stamp.
  auto cloud = makePointCloud(kBaseLink, {});
  cloud.header.stamp.sec = 42;

  // Act
  const auto grid_msg = ObstacleGridExtractor(testParams()).extract(cloud);

  // Assert: an all-NaN grid at the input stamp — "alive, nothing detected", not "clear".
  EXPECT_EQ(grid_msg.header.stamp.sec, 42);
  const auto grid = toGridMap(grid_msg);
  grid_map::Index unobserved_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(5.0, 0.0), unobserved_cell));
  EXPECT_TRUE(std::isnan(grid.at("point_count", unobserved_cell)));
}

TEST(ObstacleGridExtractor, CloudWithoutPointFieldsIsHeartbeat)
{
  // Arrange: a default-constructed cloud — no points AND no field descriptors, which is what a
  // publisher emitting a bare "nothing detected" message produces. Constructing a
  // PointCloud2ConstIterator on it would throw, so the extractor must not reach for one.
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = kBaseLink;
  cloud.header.stamp.sec = 7;

  // Act
  const auto grid_msg = ObstacleGridExtractor(testParams()).extract(cloud);

  // Assert: same all-NaN heartbeat as an empty-but-typed cloud.
  EXPECT_EQ(grid_msg.header.stamp.sec, 7);
  const auto grid = toGridMap(grid_msg);
  grid_map::Index unobserved_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(5.0, 0.0), unobserved_cell));
  EXPECT_TRUE(std::isnan(grid.at("point_count", unobserved_cell)));
}

TEST(ObstacleGridExtractor, WiderPointStrideIsHonoured)
{
  // Arrange: the same two points in Autoware's production PointXYZIRC layout, whose 16-byte stride
  // differs from the packed 12-byte one. Reading x/y/z at a fixed pitch would land on wrong bytes.
  const auto cloud = makePointXyzircCloud(kBaseLink, {{5.05f, 0.05f, 0.5f}, {5.15f, 0.15f, 1.2f}});

  // Act
  const auto grid = extractGrid(cloud);

  // Assert: identical to the packed-layout result.
  grid_map::Index occupied_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(5.1, 0.1), occupied_cell));
  EXPECT_FLOAT_EQ(grid.at("point_count", occupied_cell), 2.0f);
  EXPECT_FLOAT_EQ(grid.at("max_height", occupied_cell), 1.2f);
  EXPECT_FLOAT_EQ(grid.at("min_height", occupied_cell), 0.5f);
}

TEST(ObstacleGridExtractor, GridInheritsInputCloudFrame)
{
  // Arrange: a cloud in a frame other than base_link. The ROI is rasterized in whatever frame the
  // cloud is in, so the grid must be labelled with that frame rather than a hardcoded one.
  const auto cloud = makePointCloud("some_other_frame", {{5.05f, 0.05f, 0.5f}});

  // Act
  const auto grid_msg = ObstacleGridExtractor(testParams()).extract(cloud);

  // Assert
  EXPECT_EQ(grid_msg.header.frame_id, "some_other_frame");
  EXPECT_EQ(toGridMap(grid_msg).getFrameId(), "some_other_frame");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
