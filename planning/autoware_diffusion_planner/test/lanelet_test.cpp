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

#include "lanelet_test.hpp"

#include <vector>

namespace autoware::diffusion_planner::test
{

TEST_F(LaneletTest, ConvertToLaneSegments)
{
  LaneletConverter converter(lanelet_map_);

  auto lane_segments = converter.convert_to_lane_segments(10);

  EXPECT_EQ(lane_segments.size(), 1);               // Expect one lanelet to be converted
  EXPECT_EQ(lane_segments[0].polyline.size(), 10);  // Expect 10 points in the polyline
}

TEST_F(LaneletTest, FromLineString)
{
  LaneletConverter converter(lanelet_map_);

  auto points = converter.from_linestring(centerline_);

  EXPECT_EQ(points.size(), 3);  // Expect 3 points in the centerline
  EXPECT_FLOAT_EQ(points[0].x(), 0.0);
  EXPECT_FLOAT_EQ(points[1].x(), 10.0);
  EXPECT_FLOAT_EQ(points[2].x(), 20.0);
}

TEST_F(LaneletTest, InterpolateLane)
{
  std::vector<LanePoint> waypoints = {
    LanePoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    LanePoint(10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    LanePoint(20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
  };

  auto interpolated = interpolate_points(waypoints, 5);

  EXPECT_EQ(interpolated.size(), 5);  // Expect 5 interpolated points
  EXPECT_FLOAT_EQ(interpolated[0].x(), 0.0);
  EXPECT_FLOAT_EQ(interpolated[4].x(), 20.0);
}

}  // namespace autoware::diffusion_planner::test
