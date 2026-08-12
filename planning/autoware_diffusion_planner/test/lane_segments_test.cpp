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
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::test
{

TEST_F(LaneSegmentsTest, LaneSegmentContextFunctionality)
{
  /////////////
  // Arrange //
  /////////////
  // Create LaneSegmentContext
  preprocess::LaneSegmentContext context(lanelet_map_);

  // Create identity transformation matrix (no transformation)
  Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();

  // Create empty traffic light map (no traffic lights)
  std::map<lanelet::Id, preprocess::TrafficSignalStamped> traffic_light_id_map;

  // Create a LaneletRoute with our test lanelet
  autoware_planning_msgs::msg::LaneletRoute route;

  // Set center coordinates (middle of the test lanelet)
  const double center_x = 10.0;  // Middle point along the lanelet
  const double center_y = 0.0;   // Center of the lane
  const double center_z = 0.0;   // Ground level

  /////////
  // Act //
  /////////
  const std::vector<int64_t> segment_indices = context.select_route_segment_indices(
    route, center_x, center_y, center_z, NUM_SEGMENTS_IN_ROUTE);
  const std::pair<std::vector<float>, std::vector<float>> result =
    context.create_tensor_data_from_indices(
      transform_matrix, traffic_light_id_map, segment_indices, NUM_SEGMENTS_IN_ROUTE);

  ////////////
  // Assert //
  ////////////
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
}

TEST_F(LaneSegmentsTest, GetFirstTrafficLightOnRoute_NoTrafficLightOnRoute)
{
  preprocess::LaneSegmentContext context(lanelet_map_);

  autoware_planning_msgs::msg::LaneletRoute route;
  route.segments.resize(1);
  route.segments[0].preferred_primitive.id = 100;
  route.segments[0].preferred_primitive.primitive_type = "lane";

  const double center_x = 10.0;
  const double center_y = 0.0;
  const double center_z = 0.0;
  std::map<lanelet::Id, preprocess::TrafficSignalStamped> traffic_light_id_map;

  const auto result = context.get_first_traffic_light_on_route(
    route, center_x, center_y, center_z, traffic_light_id_map);

  EXPECT_EQ(result.traffic_light_group_id, 0)
    << "Should return empty when no traffic light on route";
  EXPECT_TRUE(result.elements.empty()) << "Elements should be empty when no traffic light on route";
}

class GetFirstTrafficLightOnRouteTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::string path = autoware::test_utils::get_absolute_path_to_lanelet_map(
      "autoware_diffusion_planner", "lanelet2_map.osm");
    const auto map_bin = autoware::test_utils::make_map_bin_msg(path, 1.0);

    lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(map_bin, lanelet_map_);

    const auto internal = convert_to_internal_lanelet_map(lanelet_map_);
    for (const auto & seg : internal.lane_segments) {
      if (seg.traffic_light_id != LaneSegment::TRAFFIC_LIGHT_ID_NONE) {
        lanelet_id_with_tl_ = seg.id;
        traffic_light_id_ = seg.traffic_light_id;
        center_x_ = seg.centerline.front().x();
        center_y_ = seg.centerline.front().y();
        center_z_ = seg.centerline.front().z();
        break;
      }
    }
  }

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_;
  int64_t lanelet_id_with_tl_{-1};
  int64_t traffic_light_id_{-1};
  double center_x_{0.0};
  double center_y_{0.0};
  double center_z_{0.0};
};

TEST_F(GetFirstTrafficLightOnRouteTest, TrafficLightNotInMap_ReturnsUnknown)
{
  ASSERT_GT(lanelet_id_with_tl_, 0) << "Test map must have at least one lanelet with traffic light";

  preprocess::LaneSegmentContext context(lanelet_map_);

  autoware_planning_msgs::msg::LaneletRoute route;
  route.segments.resize(1);
  route.segments[0].preferred_primitive.id = lanelet_id_with_tl_;
  route.segments[0].preferred_primitive.primitive_type = "lane";

  std::map<lanelet::Id, preprocess::TrafficSignalStamped> traffic_light_id_map;

  const auto result = context.get_first_traffic_light_on_route(
    route, center_x_, center_y_, center_z_, traffic_light_id_map);

  EXPECT_EQ(result.traffic_light_group_id, traffic_light_id_)
    << "Should return the traffic light ID when not in perception map";
  ASSERT_EQ(result.elements.size(), 1u) << "Should have one UNKNOWN element";
  EXPECT_EQ(result.elements[0].color, autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN)
    << "Should report UNKNOWN when no perception data";
}

TEST_F(GetFirstTrafficLightOnRouteTest, TrafficLightPresentInMap_ReturnsCachedSignal)
{
  ASSERT_GT(lanelet_id_with_tl_, 0) << "Test map must have at least one lanelet with traffic light";

  preprocess::LaneSegmentContext context(lanelet_map_);

  autoware_planning_msgs::msg::LaneletRoute route;
  route.segments.resize(1);
  route.segments[0].preferred_primitive.id = lanelet_id_with_tl_;
  route.segments[0].preferred_primitive.primitive_type = "lane";

  preprocess::TrafficSignalStamped stamped;
  stamped.signal.traffic_light_group_id = traffic_light_id_;
  autoware_perception_msgs::msg::TrafficLightElement elem;
  elem.color = autoware_perception_msgs::msg::TrafficLightElement::GREEN;
  elem.shape = autoware_perception_msgs::msg::TrafficLightElement::CIRCLE;
  elem.status = autoware_perception_msgs::msg::TrafficLightElement::SOLID_ON;
  elem.confidence = 1.0f;
  stamped.signal.elements.push_back(elem);

  std::map<lanelet::Id, preprocess::TrafficSignalStamped> traffic_light_id_map;
  traffic_light_id_map[static_cast<lanelet::Id>(traffic_light_id_)] = stamped;

  const auto result = context.get_first_traffic_light_on_route(
    route, center_x_, center_y_, center_z_, traffic_light_id_map);

  EXPECT_EQ(result.traffic_light_group_id, traffic_light_id_)
    << "Should return cached traffic light group ID";
  ASSERT_EQ(result.elements.size(), 1u) << "Should have cached element";
  EXPECT_EQ(result.elements[0].color, autoware_perception_msgs::msg::TrafficLightElement::GREEN)
    << "Should return cached signal (GREEN)";
}

}  // namespace autoware::diffusion_planner::test
