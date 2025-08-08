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

#ifndef LANE_SEGMENTS_TEST_HPP_
#define LANE_SEGMENTS_TEST_HPP_

#include "autoware/diffusion_planner/conversion/lanelet.hpp"
#include "autoware/diffusion_planner/polyline.hpp"
#include "autoware/diffusion_planner/preprocessing/lane_segments.hpp"

#include <Eigen/Dense>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace autoware::diffusion_planner::test
{

class LaneSegmentsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create mock lane segments
    const auto polyline = []() -> Polyline {
      std::vector<LanePoint> waypoints{
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
        {10.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0},
        {20.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0},
      };
      return {MapType::Lane, interpolate_points(waypoints, POINTS_PER_SEGMENT)};
    }();

    const auto right_boundaries = []() -> std::vector<BoundarySegment> {
      std::vector<BoundarySegment> segments;
      std::vector<LanePoint> waypoints{
        {0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0},
        {10.0, -1.0, 0.0, 1.0, 0.0, 0.0, 1.0},
        {20.0, -1.0, 0.0, 1.0, 1.0, 1.0, 1.0},
      };
      segments.emplace_back(MapType::Lane, interpolate_points(waypoints, POINTS_PER_SEGMENT));
      return segments;
    }();
    const auto left_boundaries = []() -> std::vector<BoundarySegment> {
      std::vector<BoundarySegment> segments;
      std::vector<LanePoint> waypoints{
        {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0},
        {10.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0},
        {20.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0},
      };
      segments.emplace_back(MapType::Lane, interpolate_points(waypoints, POINTS_PER_SEGMENT));
      return segments;
    }();
    constexpr auto speed_limit = 30.0f;
    lane_segments_.emplace_back(0, polyline, false, left_boundaries, right_boundaries, speed_limit);
  }

  std::vector<LaneSegment> lane_segments_;
};

}  // namespace autoware::diffusion_planner::test

#endif  // LANE_SEGMENTS_TEST_HPP_
