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

#include "lane_segments_test.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/preprocessing/lane_segments.hpp"

#include <Eigen/Dense>

#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

namespace autoware::diffusion_planner::test
{

TEST_F(LaneSegmentsTest, ProcessSegmentToMatrixThrowsOnInvalidInput)
{
  // Empty polyline
  LaneSegment invalid_segment = lane_segments_.front();
  invalid_segment.polyline = Polyline();
  EXPECT_EQ(preprocess::process_segment_to_matrix(invalid_segment).size(), 0);

  // Empty left boundary
  invalid_segment = lane_segments_.front();
  invalid_segment.left_boundaries.clear();
  EXPECT_EQ(preprocess::process_segment_to_matrix(invalid_segment).size(), 0);

  // Empty right boundary
  invalid_segment = lane_segments_.front();
  invalid_segment.right_boundaries.clear();
  EXPECT_EQ(preprocess::process_segment_to_matrix(invalid_segment).size(), 0);

  // Wrong number of points
  invalid_segment = lane_segments_.front();
  auto wrong_polyline = invalid_segment.polyline;
  // Remove a point to make it invalid
  auto points = wrong_polyline.waypoints();
  points.pop_back();
  Polyline short_polyline(MapType::Lane, points);
  invalid_segment.polyline = short_polyline;
  EXPECT_THROW(preprocess::process_segment_to_matrix(invalid_segment), std::runtime_error);
}

}  // namespace autoware::diffusion_planner::test
