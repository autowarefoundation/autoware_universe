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
//
// Size/coverage validation: on a representative dense cloud, the grid has the configured
// geometry, drops no in-ROI/in-z-band point silently, and records an informational latency.
// This is the CI-resident gate, so the producer ships only when geometry and coverage hold.
#include "autoware/obstacle_grid_extractor/obstacle_grid_extractor.hpp"
#include "make_point_cloud.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::obstacle_grid_extractor::ExtractorParams;
using autoware::obstacle_grid_extractor::ObstacleGridExtractor;
using autoware::obstacle_grid_extractor::test::makePointCloud;
using autoware::obstacle_grid_extractor::test::TestPoint;

constexpr int kFrameCount = 20;
constexpr int kPointsPerFrame = 12000;
constexpr double kResolution = 0.2;
constexpr float kCropZMin = -2.5f;
constexpr float kCropZMax = 3.5f;
constexpr char kBaseLink[] = "base_link";

ExtractorParams productionParams()
{
  ExtractorParams params;
  params.roi_length_x = 180.0;  // x in [-100, +80]
  params.roi_length_y = 100.0;  // y in [-50, +50]
  params.roi_offset_x = -10.0;
  params.resolution = kResolution;
  params.crop_z_min = kCropZMin;
  params.crop_z_max = kCropZMax;
  params.overhead_split = 2.5f;
  return params;
}
}  // namespace

TEST(ObstacleGridCoverage, GeometryAndNoSilentDrop)
{
  // Arrange: representative clouds spread over the WHOLE ROI and z-band, so the no-silent-drop
  // assertion below covers every corner of the grid rather than a forward patch of it.
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> dist_x(-99.0f, 79.0f);
  std::uniform_real_distribution<float> dist_y(-49.0f, 49.0f);
  std::uniform_real_distribution<float> dist_z(-2.0f, 3.0f);

  std::vector<sensor_msgs::msg::PointCloud2> clouds;
  clouds.reserve(kFrameCount);
  for (int frame = 0; frame < kFrameCount; ++frame) {
    std::vector<TestPoint> points;
    points.reserve(kPointsPerFrame);
    for (int i = 0; i < kPointsPerFrame; ++i) {
      points.push_back({dist_x(rng), dist_y(rng), dist_z(rng)});
    }
    clouds.push_back(makePointCloud(kBaseLink, points));
  }

  // Act: rasterize every frame with one extractor instance, timing each call.
  const ObstacleGridExtractor extractor(productionParams());
  std::vector<grid_map_msgs::msg::GridMap> grid_msgs;
  std::vector<double> elapsed_ms;
  grid_msgs.reserve(kFrameCount);
  elapsed_ms.reserve(kFrameCount);
  for (const auto & cloud : clouds) {
    const auto started_at = std::chrono::steady_clock::now();
    auto grid_msg = extractor.extract(cloud);
    elapsed_ms.push_back(
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started_at)
        .count());
    grid_msgs.push_back(std::move(grid_msg));
  }

  // Assert (a): every frame carries the configured geometry.
  for (const auto & grid_msg : grid_msgs) {
    grid_map::GridMap grid;
    grid_map::GridMapRosConverter::fromMessage(grid_msg, grid);
    ASSERT_EQ(grid.getLayers().size(), 4u);
    EXPECT_NEAR(grid.getResolution(), kResolution, 1e-9);
    EXPECT_EQ(grid.getFrameId(), kBaseLink);
  }

  // Assert (b): coverage — every point inside the ROI and the z-band lands in an occupied cell.
  for (int frame = 0; frame < kFrameCount; ++frame) {
    grid_map::GridMap grid;
    grid_map::GridMapRosConverter::fromMessage(grid_msgs[frame], grid);
    sensor_msgs::PointCloud2ConstIterator<float> it_x(clouds[frame], "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(clouds[frame], "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(clouds[frame], "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      if (*it_z < kCropZMin || *it_z > kCropZMax) continue;
      grid_map::Index cell_index;
      if (!grid.getIndex(grid_map::Position(*it_x, *it_y), cell_index)) continue;  // outside ROI
      ASSERT_FALSE(std::isnan(grid.at("point_count", cell_index)));
    }
  }

  // Assert (c): informational latency only. A hard wall-clock bound would flake on loaded CI and
  // sanitizer builds; the 15 ms budget is checked in the eval report instead.
  std::sort(elapsed_ms.begin(), elapsed_ms.end());
  RecordProperty(
    "p99_ms", std::to_string(elapsed_ms[static_cast<size_t>(0.99 * (elapsed_ms.size() - 1))]));
  RecordProperty("max_ms", std::to_string(elapsed_ms.back()));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
