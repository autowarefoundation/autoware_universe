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

#include "autoware/diffusion_planner/conversion/lanelet.hpp"
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

TEST_F(LaneSegmentsTest, LaneSegmentContextFunctionality)
{
  // Create LaneSegmentContext
  preprocess::LaneSegmentContext context(lanelet_map_);

  // Test that context was created successfully
  EXPECT_TRUE(true) << "LaneSegmentContext created successfully";

  // Create identity transformation matrix (no transformation)
  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

  // Create empty traffic light map (no traffic lights)
  std::map<lanelet::Id, preprocess::TrafficSignalStamped> traffic_light_id_map;

  // Create current lanes list with our test lanelet
  lanelet::ConstLanelets current_lanes = {test_lanelet_};

  // Test get_route_segments functionality
  auto result = context.get_route_segments(transform_matrix, traffic_light_id_map, current_lanes);

  // Check that we get valid results
  EXPECT_FALSE(result.first.empty()) << "Route segments should not be empty";
  EXPECT_FALSE(result.second.empty()) << "Speed limits should not be empty";

  // Check that route segment values are reasonable (not NaN or infinite)
  for (size_t i = 0; i < result.first.size(); ++i) {
    EXPECT_FALSE(std::isnan(result.first[i]))
      << "Route segment value should not be NaN at index " << i;
    EXPECT_FALSE(std::isinf(result.first[i]))
      << "Route segment value should not be infinite at index " << i;
  }

  // Check that speed limit values are reasonable (allow NaN but check for inf)
  for (size_t i = 0; i < result.second.size(); ++i) {
    EXPECT_FALSE(std::isinf(result.second[i]))
      << "Speed limit value should not be infinite at index " << i;
  }

  // Test transformation effects
  Eigen::Matrix4f transform_matrix_translated = Eigen::Matrix4f::Identity();
  transform_matrix_translated(0, 3) = 5.0f;  // x translation
  transform_matrix_translated(1, 3) = 5.0f;  // y translation

  auto result_transformed =
    context.get_route_segments(transform_matrix_translated, traffic_light_id_map, current_lanes);

  // Both results should have the same size
  EXPECT_EQ(result_transformed.first.size(), result.first.size())
    << "Transformed and identity results should have the same size";

  // Results should be different (due to transformation)
  bool results_different = false;
  for (size_t i = 0; i < std::min(result_transformed.first.size(), result.first.size()); ++i) {
    if (std::abs(result_transformed.first[i] - result.first[i]) > 0.001f) {
      results_different = true;
      break;
    }
  }
  EXPECT_TRUE(results_different) << "Transformation should change the route segment values";
}

}  // namespace autoware::diffusion_planner::test
