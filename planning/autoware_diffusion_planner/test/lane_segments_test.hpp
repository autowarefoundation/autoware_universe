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
#include "autoware/diffusion_planner/preprocessing/traffic_signals.hpp"

#include <Eigen/Dense>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <vector>

namespace autoware::diffusion_planner::test
{

class LaneSegmentsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a simple lanelet map with a single lanelet
    lanelet_map_ = std::make_shared<lanelet::LaneletMap>();

    // Create points for a simple straight lane
    auto p1 = lanelet::Point3d(1, 0.0, 0.0, 0.0);
    auto p2 = lanelet::Point3d(2, 10.0, 0.0, 0.0);
    auto p3 = lanelet::Point3d(3, 20.0, 0.0, 0.0);
    auto p4 = lanelet::Point3d(4, 0.0, -2.0, 0.0);
    auto p5 = lanelet::Point3d(5, 10.0, -2.0, 0.0);
    auto p6 = lanelet::Point3d(6, 20.0, -2.0, 0.0);
    auto p7 = lanelet::Point3d(7, 0.0, 2.0, 0.0);
    auto p8 = lanelet::Point3d(8, 10.0, 2.0, 0.0);
    auto p9 = lanelet::Point3d(9, 20.0, 2.0, 0.0);

    // Create linestrings for center, left, and right boundaries
    lanelet::LineString3d centerline(10, {p1, p2, p3});
    lanelet::LineString3d right_bound(11, {p4, p5, p6});
    lanelet::LineString3d left_bound(12, {p7, p8, p9});

    // Create lanelet
    lanelet::Lanelet test_lanelet(100, left_bound, right_bound);
    test_lanelet.setCenterline(centerline);

    // Add attributes to make it recognized as a valid lane
    test_lanelet.setAttribute(lanelet::AttributeName::Subtype, lanelet::AttributeValueString::Road);
    test_lanelet.setAttribute(
      lanelet::AttributeName::Location, lanelet::AttributeValueString::Urban);
    test_lanelet.setAttribute("speed_limit", "30");  // Add speed limit attribute

    // Add to map
    lanelet_map_->add(test_lanelet);

    // Store the lanelet for tests
    test_lanelet_ = test_lanelet;
  }

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_;
  lanelet::Lanelet test_lanelet_;
};

}  // namespace autoware::diffusion_planner::test

#endif  // LANE_SEGMENTS_TEST_HPP_
