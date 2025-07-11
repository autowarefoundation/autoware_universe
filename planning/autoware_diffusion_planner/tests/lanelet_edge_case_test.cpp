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

#include "autoware/diffusion_planner/conversion/lanelet.hpp"
#include "autoware/diffusion_planner/polyline.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::test
{

class LaneletEdgeCaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a simple lanelet map for testing
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
    traffic_rules_ptr_ = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  }

  lanelet::LineString3d createLineString(const std::vector<std::tuple<double, double, double>>& points)
  {
    lanelet::LineString3d ls;
    for (const auto& [x, y, z] : points) {
      ls.push_back(lanelet::Point3d(lanelet::utils::getId(), x, y, z));
    }
    return ls;
  }

  lanelet::Lanelet createLanelet(
    const lanelet::LineString3d& left,
    const lanelet::LineString3d& right,
    const lanelet::AttributeMap& attributes = {})
  {
    lanelet::Lanelet ll(lanelet::utils::getId(), left, right);
    for (const auto& [key, value] : attributes) {
      ll.setAttribute(key, value);
    }
    return ll;
  }

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;
};

// Test edge case: Empty lanelet map
TEST_F(LaneletEdgeCaseTest, ConvertEmptyLaneletMap)
{
  const size_t max_num_polyline = 100;
  const size_t max_num_point = 20;
  const double point_break_distance = 100.0;
  LaneletConverter converter(lanelet_map_ptr_, max_num_polyline, max_num_point, point_break_distance);

  geometry_msgs::msg::Point position;
  position.x = 0.0;
  position.y = 0.0;
  position.z = 0.0;

  auto result = converter.convert(position, 100.0);

  // Should return empty optional
  EXPECT_FALSE(result.has_value());
}

// Test edge case: Lanelet with invalid speed limit string
TEST_F(LaneletEdgeCaseTest, ConvertLaneletInvalidSpeedLimit)
{
  auto left = createLineString({{0, 0, 0}, {10, 0, 0}});
  auto right = createLineString({{0, 2, 0}, {10, 2, 0}});

  // Add lanelet with invalid speed limit
  lanelet::AttributeMap attrs;
  attrs["speed_limit"] = "not_a_number";
  attrs["subtype"] = "road";

  auto lanelet = createLanelet(left, right, attrs);
  lanelet_map_ptr_->add(lanelet);

  const size_t max_num_polyline = 100;
  const size_t max_num_point = 20;
  const double point_break_distance = 100.0;
  LaneletConverter converter(lanelet_map_ptr_, max_num_polyline, max_num_point, point_break_distance);

  // This should throw due to std::stof failing
  EXPECT_THROW(converter.convert_to_lane_segments(10), std::invalid_argument);
}

// Test edge case: Lanelet with extreme speed limit values
TEST_F(LaneletEdgeCaseTest, ConvertLaneletExtremeSpeedLimit)
{
  auto left = createLineString({{0, 0, 0}, {10, 0, 0}});
  auto right = createLineString({{0, 2, 0}, {10, 2, 0}});

  // Test with very large speed limit
  lanelet::AttributeMap attrs;
  attrs["speed_limit"] = "999999";
  attrs["subtype"] = "road";

  auto lanelet = createLanelet(left, right, attrs);
  lanelet_map_ptr_->add(lanelet);

  const size_t max_num_polyline = 100;
  const size_t max_num_point = 20;
  const double point_break_distance = 100.0;
  LaneletConverter converter(lanelet_map_ptr_, max_num_polyline, max_num_point, point_break_distance);
  auto segments = converter.convert_to_lane_segments(10);

  ASSERT_EQ(segments.size(), 1);
  EXPECT_TRUE(segments[0].speed_limit_mps.has_value());
  // Speed should be converted from km/h to m/s
  EXPECT_GT(segments[0].speed_limit_mps.value(), 100000.0f);  // Very large value
}

// Test edge case: Lanelet with NaN/Inf coordinates
TEST_F(LaneletEdgeCaseTest, ConvertLaneletWithNaNInfCoordinates)
{
  auto left = lanelet::LineString3d();
  left.push_back(lanelet::Point3d(1, 0, 0, 0));
  left.push_back(lanelet::Point3d(2, std::numeric_limits<double>::quiet_NaN(), 0, 0));

  auto right = lanelet::LineString3d();
  right.push_back(lanelet::Point3d(3, 0, 2, 0));
  right.push_back(lanelet::Point3d(4, std::numeric_limits<double>::infinity(), 2, 0));

  lanelet::AttributeMap attrs;
  attrs["subtype"] = "road";

  auto lanelet = createLanelet(left, right, attrs);
  lanelet_map_ptr_->add(lanelet);

  const size_t max_num_polyline = 100;
  const size_t max_num_point = 20;
  const double point_break_distance = 100.0;
  LaneletConverter converter(lanelet_map_ptr_, max_num_polyline, max_num_point, point_break_distance);
  auto segments = converter.convert_to_lane_segments(10);

  // Should handle NaN/Inf gracefully
  ASSERT_EQ(segments.size(), 1);
  const auto& polyline = segments[0].polyline;

  // Check that NaN/Inf propagated through
  bool has_nan = false;
  bool has_inf = false;
  for (const auto& point : polyline.waypoints()) {
    if (std::isnan(point.x()) || std::isnan(point.y())) has_nan = true;
    if (std::isinf(point.x()) || std::isinf(point.y())) has_inf = true;
  }
  EXPECT_TRUE(has_nan || has_inf);
}

// Test edge case: Zero-length lanelet
TEST_F(LaneletEdgeCaseTest, ConvertZeroLengthLanelet)
{
  // All points at the same location
  auto left = createLineString({{5, 5, 0}, {5, 5, 0}});
  auto right = createLineString({{5, 7, 0}, {5, 7, 0}});

  lanelet::AttributeMap attrs;
  attrs["subtype"] = "road";

  auto lanelet = createLanelet(left, right, attrs);
  lanelet_map_ptr_->add(lanelet);

  const size_t max_num_polyline = 100;
  const size_t max_num_point = 20;
  const double point_break_distance = 100.0;
  LaneletConverter converter(lanelet_map_ptr_, max_num_polyline, max_num_point, point_break_distance);
  auto segments = converter.convert_to_lane_segments(10);

  ASSERT_EQ(segments.size(), 1);
  // Should still create a segment, even if degenerate
  EXPECT_GE(segments[0].polyline.size(), 2);
}

// Test edge case: Very large number of interpolation points
TEST_F(LaneletEdgeCaseTest, ConvertLaneletManyInterpolationPoints)
{
  auto left = createLineString({{0, 0, 0}, {1000, 0, 0}});
  auto right = createLineString({{0, 10, 0}, {1000, 10, 0}});

  lanelet::AttributeMap attrs;
  attrs["subtype"] = "road";

  auto lanelet = createLanelet(left, right, attrs);
  lanelet_map_ptr_->add(lanelet);

  const size_t max_num_polyline = 100;
  const size_t max_num_point = 20;
  const double point_break_distance = 100.0;
  LaneletConverter converter(lanelet_map_ptr_, max_num_polyline, max_num_point, point_break_distance);

  // Request extremely high number of interpolation points
  auto segments = converter.convert_to_lane_segments(10000);

  ASSERT_EQ(segments.size(), 1);
  // Should create many interpolated points
  EXPECT_GT(segments[0].polyline.size(), 1000);
}

// Test edge case: Lanelet with intersection attribute edge cases
TEST_F(LaneletEdgeCaseTest, ConvertLaneletIntersectionAttributes)
{
  auto left = createLineString({{0, 0, 0}, {10, 0, 0}});
  auto right = createLineString({{0, 2, 0}, {10, 2, 0}});

  // Test various turn_direction values
  std::vector<std::string> turn_directions = {"", "left", "right", "straight", "invalid_direction"};

  for (const auto& direction : turn_directions) {
    // Recreate map for each test
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();

    lanelet::AttributeMap attrs;
    attrs["subtype"] = "road";
    attrs["turn_direction"] = direction;

    auto lanelet = createLanelet(left, right, attrs);
    lanelet_map_ptr_->add(lanelet);

    const size_t max_num_polyline = 100;
    const size_t max_num_point = 20;
    const double point_break_distance = 100.0;
    LaneletConverter converter(lanelet_map_ptr_, max_num_polyline, max_num_point, point_break_distance);
    auto segments = converter.convert_to_lane_segments(10);

    ASSERT_EQ(segments.size(), 1);
    EXPECT_TRUE(segments[0].is_intersection);
  }
}

// Test edge case: Distance threshold boundary conditions
TEST_F(LaneletEdgeCaseTest, ConvertDistanceThresholdBoundary)
{
  auto left = createLineString({{0, 0, 0}, {10, 0, 0}});
  auto right = createLineString({{0, 2, 0}, {10, 2, 0}});

  lanelet::AttributeMap attrs;
  attrs["subtype"] = "road";

  auto lanelet = createLanelet(left, right, attrs);
  lanelet_map_ptr_->add(lanelet);

  const size_t max_num_polyline = 100;
  const size_t max_num_point = 20;
  const double point_break_distance = 100.0;
  LaneletConverter converter(lanelet_map_ptr_, max_num_polyline, max_num_point, point_break_distance);

  // Test with position exactly at distance threshold
  geometry_msgs::msg::Point position;
  position.x = 15.0;  // 5 units away from end of lanelet
  position.y = 1.0;
  position.z = 0.0;

  // Test with threshold exactly at distance
  auto result1 = converter.convert(position, 5.0);
  EXPECT_TRUE(result1.has_value());

  // Test with threshold just below distance
  auto result2 = converter.convert(position, 4.999);
  EXPECT_FALSE(result2.has_value());
}

// Test edge case: Crosswalk polygon conversion
TEST_F(LaneletEdgeCaseTest, ConvertCrosswalkPolygon)
{
  // Create a crosswalk as a polygon
  auto p1 = lanelet::Point3d(1, 0, 0, 0);
  auto p2 = lanelet::Point3d(2, 10, 0, 0);
  auto p3 = lanelet::Point3d(3, 10, 3, 0);
  auto p4 = lanelet::Point3d(4, 0, 3, 0);

  lanelet::LineString3d left;
  left.push_back(p1);
  left.push_back(p2);

  lanelet::LineString3d right;
  right.push_back(p4);
  right.push_back(p3);

  lanelet::AttributeMap attrs;
  attrs["subtype"] = "crosswalk";

  auto crosswalk = createLanelet(left, right, attrs);
  lanelet_map_ptr_->add(crosswalk);

  const size_t max_num_polyline = 100;
  const size_t max_num_point = 20;
  const double point_break_distance = 100.0;
  LaneletConverter converter(lanelet_map_ptr_, max_num_polyline, max_num_point, point_break_distance);

  geometry_msgs::msg::Point position;
  position.x = 5.0;
  position.y = 1.5;
  position.z = 0.0;

  auto result = converter.convert(position, 100.0);

  // Result should contain crosswalk points
  EXPECT_TRUE(result.has_value());
}

// Test edge case: Convert empty map to lane segments
TEST_F(LaneletEdgeCaseTest, ConvertEmptyMapToLaneSegments)
{
  const size_t max_num_polyline = 100;
  const size_t max_num_point = 20;
  const double point_break_distance = 100.0;
  LaneletConverter converter(lanelet_map_ptr_, max_num_polyline, max_num_point, point_break_distance);

  // Convert empty map
  auto segments = converter.convert_to_lane_segments(10);

  // Should return empty vector
  EXPECT_TRUE(segments.empty());
}

// Test edge case: Negative speed limit conversion
TEST_F(LaneletEdgeCaseTest, ConvertNegativeSpeedLimit)
{
  auto left = createLineString({{0, 0, 0}, {10, 0, 0}});
  auto right = createLineString({{0, 2, 0}, {10, 2, 0}});

  lanelet::AttributeMap attrs;
  attrs["speed_limit"] = "-50";  // Negative speed limit
  attrs["subtype"] = "road";

  auto lanelet = createLanelet(left, right, attrs);
  lanelet_map_ptr_->add(lanelet);

  const size_t max_num_polyline = 100;
  const size_t max_num_point = 20;
  const double point_break_distance = 100.0;
  LaneletConverter converter(lanelet_map_ptr_, max_num_polyline, max_num_point, point_break_distance);
  auto segments = converter.convert_to_lane_segments(10);

  ASSERT_EQ(segments.size(), 1);
  EXPECT_TRUE(segments[0].speed_limit_mps.has_value());
  // Negative speed should be preserved (though semantically invalid)
  EXPECT_LT(segments[0].speed_limit_mps.value(), 0.0f);
}

// Test edge case: Unicode in lanelet attributes
TEST_F(LaneletEdgeCaseTest, ConvertUnicodeAttributes)
{
  auto left = createLineString({{0, 0, 0}, {10, 0, 0}});
  auto right = createLineString({{0, 2, 0}, {10, 2, 0}});

  lanelet::AttributeMap attrs;
  attrs["subtype"] = "road";
  attrs["name"] = "道路_🚗";  // Unicode road name
  attrs["turn_direction"] = "左折";  // Japanese for "left turn"

  auto lanelet = createLanelet(left, right, attrs);
  lanelet_map_ptr_->add(lanelet);

  const size_t max_num_polyline = 100;
  const size_t max_num_point = 20;
  const double point_break_distance = 100.0;
  LaneletConverter converter(lanelet_map_ptr_, max_num_polyline, max_num_point, point_break_distance);

  // Should handle unicode attributes without crashing
  EXPECT_NO_THROW(converter.convert_to_lane_segments(10));
}

}  // namespace autoware::diffusion_planner::test
