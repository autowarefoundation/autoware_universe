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
  auto lane_segments = convert_to_lane_segments(lanelet_map_, 10);

  EXPECT_EQ(lane_segments.size(), 1);                 // Expect one lanelet to be converted
  EXPECT_EQ(lane_segments[0].centerline.size(), 10);  // Expect 10 points in the polyline
}

}  // namespace autoware::diffusion_planner::test
