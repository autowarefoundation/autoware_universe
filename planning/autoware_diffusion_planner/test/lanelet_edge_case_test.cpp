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

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

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

  lanelet::LineString3d createLineString(
    const std::vector<std::tuple<double, double, double>> & points)
  {
    lanelet::LineString3d ls;
    for (const auto & [x, y, z] : points) {
      ls.push_back(lanelet::Point3d(lanelet::utils::getId(), x, y, z));
    }
    return ls;
  }

  lanelet::Lanelet createLanelet(
    const lanelet::LineString3d & left, const lanelet::LineString3d & right,
    const lanelet::AttributeMap & attributes = {})
  {
    lanelet::Lanelet ll(lanelet::utils::getId(), left, right);
    for (const auto & [key, value] : attributes) {
      ll.setAttribute(key, value);
    }
    return ll;
  }

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;
};

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

  LaneletConverter converter(lanelet_map_ptr_);

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

  LaneletConverter converter(lanelet_map_ptr_);
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

  LaneletConverter converter(lanelet_map_ptr_);
  auto segments = converter.convert_to_lane_segments(10);

  // Should handle NaN/Inf gracefully
  ASSERT_EQ(segments.size(), 1);
  const auto & polyline = segments[0].polyline;

  // Check that NaN/Inf propagated through
  bool has_nan = false;
  bool has_inf = false;
  for (const auto & point : polyline.waypoints()) {
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

  LaneletConverter converter(lanelet_map_ptr_);
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

  LaneletConverter converter(lanelet_map_ptr_);

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

  for (const auto & direction : turn_directions) {
    // Recreate map for each test
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();

    lanelet::AttributeMap attrs;
    attrs["subtype"] = "road";
    attrs["turn_direction"] = direction;

    auto lanelet = createLanelet(left, right, attrs);
    lanelet_map_ptr_->add(lanelet);

    LaneletConverter converter(lanelet_map_ptr_);
    auto segments = converter.convert_to_lane_segments(10);

    ASSERT_EQ(segments.size(), 1);
    EXPECT_TRUE(segments[0].is_intersection);
  }
}

// Test edge case: Convert empty map to lane segments
TEST_F(LaneletEdgeCaseTest, ConvertEmptyMapToLaneSegments)
{
  LaneletConverter converter(lanelet_map_ptr_);

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

  LaneletConverter converter(lanelet_map_ptr_);
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
  attrs["name"] = "道路_🚗";         // Unicode road name
  attrs["turn_direction"] = "左折";  // Japanese for "left turn"

  auto lanelet = createLanelet(left, right, attrs);
  lanelet_map_ptr_->add(lanelet);

  LaneletConverter converter(lanelet_map_ptr_);

  // Should handle unicode attributes without crashing
  EXPECT_NO_THROW(converter.convert_to_lane_segments(10));
}

}  // namespace autoware::diffusion_planner::test
