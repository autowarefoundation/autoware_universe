// Copyright 2024 TIER IV, Inc.
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

#include "autoware/diffusion_planner/conversion/lanelet.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/preprocessing/lane_segments.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

using autoware::diffusion_planner::convert_to_internal_lanelet_map;
using autoware::diffusion_planner::LaneSegment;
using autoware_map_msgs::msg::LaneletMapBin;

class LaneletIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Get the path to the lanelet2_map.osm file in the tests folder
    const std::string package_name = "autoware_diffusion_planner";
    const std::string map_filename = "lanelet2_map.osm";
    const std::string test_map_path =
      autoware::test_utils::get_absolute_path_to_lanelet_map(package_name, map_filename);

    // Create HADMapBin message from the OSM file
    map_bin_msg_ = autoware::test_utils::make_map_bin_msg(test_map_path, 1.0);

    // Convert HADMapBin to lanelet map
    lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
      autoware::experimental::lanelet2_utils::from_autoware_map_msgs(map_bin_msg_));

    auto routing_graph_and_traffic_rules =
      autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
        lanelet_map_ptr_);

    routing_graph_ptr_ =
      autoware::experimental::lanelet2_utils::remove_const(routing_graph_and_traffic_rules.first);
    traffic_rules_ptr_ = routing_graph_and_traffic_rules.second;
  }

  void TearDown() override
  {
    lanelet_map_ptr_.reset();
    traffic_rules_ptr_.reset();
    routing_graph_ptr_.reset();
  }

  LaneletMapBin map_bin_msg_;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
};

TEST_F(LaneletIntegrationTest, ConvertToLaneSegmentsBasic)
{
  using autoware::diffusion_planner::POINTS_PER_SEGMENT;
  auto lane_segments = convert_to_internal_lanelet_map(lanelet_map_ptr_);

  // Basic assertions
  EXPECT_FALSE(lane_segments.lane_segments.empty()) << "Lane segments should not be empty";

  // Check that each lane segment has the expected number of points
  for (const auto & segment : lane_segments.lane_segments) {
    EXPECT_EQ(segment.centerline.size(), static_cast<size_t>(POINTS_PER_SEGMENT))
      << "Lane polyline should have " << POINTS_PER_SEGMENT << " points";

    // Each boundary should have the expected number of points
    EXPECT_EQ(segment.left_boundary.size(), static_cast<size_t>(POINTS_PER_SEGMENT))
      << "Left boundary should have " << POINTS_PER_SEGMENT << " points";

    EXPECT_EQ(segment.right_boundary.size(), static_cast<size_t>(POINTS_PER_SEGMENT))
      << "Right boundary should have " << POINTS_PER_SEGMENT << " points";
  }
}

TEST_F(LaneletIntegrationTest, ConvertToLaneSegmentsAttributes)
{
  auto lane_segments = convert_to_internal_lanelet_map(lanelet_map_ptr_);

  EXPECT_FALSE(lane_segments.lane_segments.empty());

  // Check various attributes
  for (const auto & segment : lane_segments.lane_segments) {
    // Check ID is valid
    EXPECT_GT(segment.id, 0) << "Lane segment ID should be positive";
  }
}

TEST_F(LaneletIntegrationTest, ConvertToLaneSegmentsConsistency)
{
  auto lane_segments_1 = convert_to_internal_lanelet_map(lanelet_map_ptr_);
  auto lane_segments_2 = convert_to_internal_lanelet_map(lanelet_map_ptr_);

  EXPECT_EQ(lane_segments_1.lane_segments.size(), lane_segments_2.lane_segments.size())
    << "Multiple calls should produce the same number of segments";

  // Check that segments are in the same order with same IDs
  for (size_t i = 0; i < lane_segments_1.lane_segments.size(); ++i) {
    EXPECT_EQ(lane_segments_1.lane_segments[i].id, lane_segments_2.lane_segments[i].id)
      << "Segment IDs should be consistent across calls";

    EXPECT_EQ(
      lane_segments_1.lane_segments[i].centerline.size(),
      lane_segments_2.lane_segments[i].centerline.size())
      << "Polyline sizes should be consistent";
  }
}

TEST_F(LaneletIntegrationTest, CheckForNaNAndInfiniteValues)
{
  auto lane_segments = convert_to_internal_lanelet_map(lanelet_map_ptr_);

  EXPECT_FALSE(lane_segments.lane_segments.empty());

  for (const auto & segment : lane_segments.lane_segments) {
    // Check polyline points
    const auto & waypoints = segment.centerline;
    for (size_t i = 0; i < waypoints.size(); ++i) {
      const auto & point = waypoints[i];

      // Check for NaN
      EXPECT_FALSE(std::isnan(point.x()))
        << "NaN found in x coordinate at point " << i << " of segment " << segment.id;
      EXPECT_FALSE(std::isnan(point.y()))
        << "NaN found in y coordinate at point " << i << " of segment " << segment.id;
      EXPECT_FALSE(std::isnan(point.z()))
        << "NaN found in z coordinate at point " << i << " of segment " << segment.id;

      // Check for infinite values
      EXPECT_FALSE(std::isinf(point.x()))
        << "Infinite value found in x coordinate at point " << i << " of segment " << segment.id;
      EXPECT_FALSE(std::isinf(point.y()))
        << "Infinite value found in y coordinate at point " << i << " of segment " << segment.id;
      EXPECT_FALSE(std::isinf(point.z()))
        << "Infinite value found in z coordinate at point " << i << " of segment " << segment.id;
    }

    // Check boundaries
    EXPECT_FALSE(segment.left_boundary.empty()) << "Left boundary should not be empty";
    for (size_t i = 0; i < segment.left_boundary.size(); ++i) {
      const auto & point = segment.left_boundary[i];
      EXPECT_FALSE(std::isnan(point.x()) || std::isnan(point.y()) || std::isnan(point.z()))
        << "NaN found in left boundary at point " << i << " of segment " << segment.id;
      EXPECT_FALSE(std::isinf(point.x()) || std::isinf(point.y()) || std::isinf(point.z()))
        << "Infinite value found in left boundary at point " << i << " of segment " << segment.id;
    }

    EXPECT_FALSE(segment.right_boundary.empty()) << "Right boundary should not be empty";
    for (size_t i = 0; i < segment.right_boundary.size(); ++i) {
      const auto & point = segment.right_boundary[i];
      EXPECT_FALSE(std::isnan(point.x()) || std::isnan(point.y()) || std::isnan(point.z()))
        << "NaN found in right boundary at point " << i << " of segment " << segment.id;
      EXPECT_FALSE(std::isinf(point.x()) || std::isinf(point.y()) || std::isinf(point.z()))
        << "Infinite value found in right boundary at point " << i << " of segment " << segment.id;
    }
  }
}

TEST_F(LaneletIntegrationTest, CheckReasonableCoordinateRanges)
{
  // First, get the bounds from the original lanelet map
  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();

  // Calculate bounds from the original lanelet map
  for (const auto & lanelet : lanelet_map_ptr_->laneletLayer) {
    // Check centerline points
    for (const auto & point : lanelet.centerline3d()) {
      min_x = std::min(min_x, static_cast<float>(point.x()));
      max_x = std::max(max_x, static_cast<float>(point.x()));
      min_y = std::min(min_y, static_cast<float>(point.y()));
      max_y = std::max(max_y, static_cast<float>(point.y()));
      min_z = std::min(min_z, static_cast<float>(point.z()));
      max_z = std::max(max_z, static_cast<float>(point.z()));
    }

    // Check left boundary points
    for (const auto & point : lanelet.leftBound3d()) {
      min_x = std::min(min_x, static_cast<float>(point.x()));
      max_x = std::max(max_x, static_cast<float>(point.x()));
      min_y = std::min(min_y, static_cast<float>(point.y()));
      max_y = std::max(max_y, static_cast<float>(point.y()));
      min_z = std::min(min_z, static_cast<float>(point.z()));
      max_z = std::max(max_z, static_cast<float>(point.z()));
    }

    // Check right boundary points
    for (const auto & point : lanelet.rightBound3d()) {
      min_x = std::min(min_x, static_cast<float>(point.x()));
      max_x = std::max(max_x, static_cast<float>(point.x()));
      min_y = std::min(min_y, static_cast<float>(point.y()));
      max_y = std::max(max_y, static_cast<float>(point.y()));
      min_z = std::min(min_z, static_cast<float>(point.z()));
      max_z = std::max(max_z, static_cast<float>(point.z()));
    }
  }

  // Add small tolerance for floating point comparisons and potential interpolation overshoot
  const float tolerance = 0.1f;  // 10cm tolerance
  const float min_x_allowed = min_x - tolerance;
  const float max_x_allowed = max_x + tolerance;
  const float min_y_allowed = min_y - tolerance;
  const float max_y_allowed = max_y + tolerance;
  const float min_z_allowed = min_z - tolerance;
  const float max_z_allowed = max_z + tolerance;

  // Now convert and check that interpolated points are within bounds
  auto lane_segments = convert_to_internal_lanelet_map(lanelet_map_ptr_);

  EXPECT_FALSE(lane_segments.lane_segments.empty());

  // Check all interpolated points are within the original map bounds
  for (const auto & segment : lane_segments.lane_segments) {
    const auto & waypoints = segment.centerline;

    for (size_t i = 0; i < waypoints.size(); ++i) {
      const auto & point = waypoints[i];

      // Check coordinate ranges
      EXPECT_GE(point.x(), min_x_allowed)
        << "Interpolated X coordinate below original map bounds at point " << i << " of segment "
        << segment.id << ". Value: " << point.x() << ", Min allowed: " << min_x_allowed;
      EXPECT_LE(point.x(), max_x_allowed)
        << "Interpolated X coordinate above original map bounds at point " << i << " of segment "
        << segment.id << ". Value: " << point.x() << ", Max allowed: " << max_x_allowed;
      EXPECT_GE(point.y(), min_y_allowed)
        << "Interpolated Y coordinate below original map bounds at point " << i << " of segment "
        << segment.id << ". Value: " << point.y() << ", Min allowed: " << min_y_allowed;
      EXPECT_LE(point.y(), max_y_allowed)
        << "Interpolated Y coordinate above original map bounds at point " << i << " of segment "
        << segment.id << ". Value: " << point.y() << ", Max allowed: " << max_y_allowed;
      EXPECT_GE(point.z(), min_z_allowed)
        << "Interpolated Z coordinate below original map bounds at point " << i << " of segment "
        << segment.id << ". Value: " << point.z() << ", Min allowed: " << min_z_allowed;
      EXPECT_LE(point.z(), max_z_allowed)
        << "Interpolated Z coordinate above original map bounds at point " << i << " of segment "
        << segment.id << ". Value: " << point.z() << ", Max allowed: " << max_z_allowed;
    }

    // Also check boundaries
    for (size_t i = 0; i < segment.left_boundary.size(); ++i) {
      const auto & point = segment.left_boundary[i];
      EXPECT_GE(point.x(), min_x_allowed)
        << "Left boundary X out of bounds at point " << i << " of segment " << segment.id;
      EXPECT_LE(point.x(), max_x_allowed)
        << "Left boundary X out of bounds at point " << i << " of segment " << segment.id;
      EXPECT_GE(point.y(), min_y_allowed)
        << "Left boundary Y out of bounds at point " << i << " of segment " << segment.id;
      EXPECT_LE(point.y(), max_y_allowed)
        << "Left boundary Y out of bounds at point " << i << " of segment " << segment.id;
    }

    for (size_t i = 0; i < segment.right_boundary.size(); ++i) {
      const auto & point = segment.right_boundary[i];
      EXPECT_GE(point.x(), min_x_allowed)
        << "Right boundary X out of bounds at point " << i << " of segment " << segment.id;
      EXPECT_LE(point.x(), max_x_allowed)
        << "Right boundary X out of bounds at point " << i << " of segment " << segment.id;
      EXPECT_GE(point.y(), min_y_allowed)
        << "Right boundary Y out of bounds at point " << i << " of segment " << segment.id;
      EXPECT_LE(point.y(), max_y_allowed)
        << "Right boundary Y out of bounds at point " << i << " of segment " << segment.id;
    }
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
