// Copyright 2026 Tier IV, Inc.
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

#include "traffic_light_map_visualizer/traffic_light_visualizer.hpp"

#include <builtin_interfaces/msg/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace
{
using autoware::traffic_light::Bulb;
using autoware::traffic_light::BulbsByGroupId;
using autoware::traffic_light::TrafficLightVisualizer;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using visualization_msgs::msg::Marker;

constexpr lanelet::Id test_group_id = 100;
constexpr lanelet::Id another_test_group_id = 200;
constexpr lanelet::Id non_existent_group_id = 999;
constexpr lanelet::Id arbitrary_bulb_id = 42;

Bulb make_bulb(lanelet::Id id, double x, double y, double z, uint8_t color)
{
  Bulb bulb;
  bulb.id = id;
  bulb.position.x = x;
  bulb.position.y = y;
  bulb.position.z = z;
  bulb.color = color;
  return bulb;
}

// Overload for tests where the bulb's position is irrelevant.
Bulb make_bulb(lanelet::Id id, uint8_t color)
{
  return make_bulb(id, 0.0, 0.0, 0.0, color);
}

TrafficLightGroup make_group(lanelet::Id group_id, const std::vector<uint8_t> & colors)
{
  TrafficLightGroup group;
  group.traffic_light_group_id = group_id;
  for (auto color : colors) {
    TrafficLightElement element;
    element.color = color;
    group.elements.push_back(element);
  }
  return group;
}

TrafficLightGroupArray make_detection(std::vector<TrafficLightGroup> groups)
{
  TrafficLightGroupArray array;
  array.traffic_light_groups = std::move(groups);
  return array;
}

}  // namespace

TEST(TrafficLightVisualizer, EmptyBulbsProducesEmptyMarkers)
{
  TrafficLightVisualizer visualizer{BulbsByGroupId{}};
  auto detection = make_detection({make_group(test_group_id, {TrafficLightElement::RED})});

  auto markers = visualizer.generate_markers(detection, builtin_interfaces::msg::Time{});

  EXPECT_TRUE(markers.empty());
}

TEST(TrafficLightVisualizer, EmptyDetectionProducesEmptyMarkers)
{
  BulbsByGroupId map_data;
  map_data.emplace(
    test_group_id, std::vector<Bulb>{make_bulb(arbitrary_bulb_id, TrafficLightElement::RED)});
  TrafficLightVisualizer visualizer{std::move(map_data)};
  auto detection = make_detection({});

  auto markers = visualizer.generate_markers(detection, builtin_interfaces::msg::Time{});

  EXPECT_TRUE(markers.empty());
}

TEST(TrafficLightVisualizer, RedBulbDetectedProducesRedMarker)
{
  BulbsByGroupId map_data;
  map_data.emplace(
    test_group_id, std::vector<Bulb>{make_bulb(arbitrary_bulb_id, TrafficLightElement::RED)});
  TrafficLightVisualizer visualizer{std::move(map_data)};
  auto detection = make_detection({make_group(test_group_id, {TrafficLightElement::RED})});

  auto markers = visualizer.generate_markers(detection, builtin_interfaces::msg::Time{});

  ASSERT_EQ(markers.size(), 1u);
  EXPECT_FLOAT_EQ(markers[0].color.r, 1.0f);
  EXPECT_FLOAT_EQ(markers[0].color.g, 0.0f);
  EXPECT_FLOAT_EQ(markers[0].color.b, 0.0f);
}

TEST(TrafficLightVisualizer, GreenBulbDetectedProducesGreenMarker)
{
  BulbsByGroupId map_data;
  map_data.emplace(
    test_group_id, std::vector<Bulb>{make_bulb(arbitrary_bulb_id, TrafficLightElement::GREEN)});
  TrafficLightVisualizer visualizer{std::move(map_data)};
  auto detection = make_detection({make_group(test_group_id, {TrafficLightElement::GREEN})});

  auto markers = visualizer.generate_markers(detection, builtin_interfaces::msg::Time{});

  ASSERT_EQ(markers.size(), 1u);
  EXPECT_FLOAT_EQ(markers[0].color.r, 0.0f);
  EXPECT_FLOAT_EQ(markers[0].color.g, 1.0f);
  EXPECT_FLOAT_EQ(markers[0].color.b, 0.0f);
}

TEST(TrafficLightVisualizer, AmberBulbDetectedProducesYellowMarker)
{
  BulbsByGroupId map_data;
  map_data.emplace(
    test_group_id, std::vector<Bulb>{make_bulb(arbitrary_bulb_id, TrafficLightElement::AMBER)});
  TrafficLightVisualizer visualizer{std::move(map_data)};
  auto detection = make_detection({make_group(test_group_id, {TrafficLightElement::AMBER})});

  auto markers = visualizer.generate_markers(detection, builtin_interfaces::msg::Time{});

  ASSERT_EQ(markers.size(), 1u);
  EXPECT_FLOAT_EQ(markers[0].color.r, 1.0f);
  EXPECT_FLOAT_EQ(markers[0].color.g, 1.0f);
  EXPECT_FLOAT_EQ(markers[0].color.b, 0.0f);
}

TEST(TrafficLightVisualizer, UnknownGroupIdProducesEmptyMarkers)
{
  BulbsByGroupId map_data;
  map_data.emplace(
    test_group_id, std::vector<Bulb>{make_bulb(arbitrary_bulb_id, TrafficLightElement::RED)});
  TrafficLightVisualizer visualizer{std::move(map_data)};
  auto detection = make_detection({make_group(non_existent_group_id, {TrafficLightElement::RED})});

  auto markers = visualizer.generate_markers(detection, builtin_interfaces::msg::Time{});

  EXPECT_TRUE(markers.empty());
}

TEST(TrafficLightVisualizer, KnownAndUnknownGroupMixOnlyKnownProducesMarkers)
{
  BulbsByGroupId map_data;
  map_data.emplace(
    test_group_id, std::vector<Bulb>{make_bulb(arbitrary_bulb_id, TrafficLightElement::RED)});
  TrafficLightVisualizer visualizer{std::move(map_data)};
  auto detection = make_detection({
    make_group(test_group_id, {TrafficLightElement::RED}),
    make_group(non_existent_group_id, {TrafficLightElement::RED}),
  });

  auto markers = visualizer.generate_markers(detection, builtin_interfaces::msg::Time{});

  ASSERT_EQ(markers.size(), 1u);
  EXPECT_EQ(markers[0].id, arbitrary_bulb_id);
}

TEST(TrafficLightVisualizer, OnlyDetectedColorsAreShown)
{
  BulbsByGroupId map_data;
  map_data.emplace(
    test_group_id, std::vector<Bulb>{
                     make_bulb(1, TrafficLightElement::RED),
                     make_bulb(2, TrafficLightElement::GREEN),
                     make_bulb(3, TrafficLightElement::AMBER),
                   });
  TrafficLightVisualizer visualizer{std::move(map_data)};
  auto detection = make_detection({make_group(test_group_id, {TrafficLightElement::RED})});

  auto markers = visualizer.generate_markers(detection, builtin_interfaces::msg::Time{});

  ASSERT_EQ(markers.size(), 1u);
  EXPECT_EQ(markers[0].id, 1);
}

TEST(TrafficLightVisualizer, DetectionColorWithoutMapBulbIsIgnored)
{
  BulbsByGroupId map_data;
  map_data.emplace(
    test_group_id, std::vector<Bulb>{make_bulb(arbitrary_bulb_id, TrafficLightElement::RED)});
  TrafficLightVisualizer visualizer{std::move(map_data)};
  // detection contains colors not represented in the map (GREEN, UNKNOWN);
  // only RED has a corresponding bulb and should produce a marker.
  auto detection = make_detection({make_group(
    test_group_id,
    {TrafficLightElement::RED, TrafficLightElement::GREEN, TrafficLightElement::UNKNOWN})});

  auto markers = visualizer.generate_markers(detection, builtin_interfaces::msg::Time{});

  ASSERT_EQ(markers.size(), 1u);
  EXPECT_EQ(markers[0].id, arbitrary_bulb_id);
}

TEST(TrafficLightVisualizer, TwoGroupsBothMatchedProduceFourMarkers)
{
  // Setup convention: bulb id N is placed at x = N.0. The id-to-x correlation
  // makes the position assertions below readable and guards against position
  // aliasing (e.g. copying bulbs[0].position to all markers).
  BulbsByGroupId map_data;
  map_data.emplace(
    test_group_id, std::vector<Bulb>{
                     make_bulb(1, 1.0, 0.0, 0.0, TrafficLightElement::RED),
                     make_bulb(2, 2.0, 0.0, 0.0, TrafficLightElement::GREEN),
                   });
  map_data.emplace(
    another_test_group_id, std::vector<Bulb>{
                             make_bulb(3, 3.0, 0.0, 0.0, TrafficLightElement::RED),
                             make_bulb(4, 4.0, 0.0, 0.0, TrafficLightElement::GREEN),
                           });
  TrafficLightVisualizer visualizer{std::move(map_data)};
  auto detection = make_detection({
    make_group(test_group_id, {TrafficLightElement::RED, TrafficLightElement::GREEN}),
    make_group(another_test_group_id, {TrafficLightElement::RED, TrafficLightElement::GREEN}),
  });

  auto markers = visualizer.generate_markers(detection, builtin_interfaces::msg::Time{});

  ASSERT_EQ(markers.size(), 4u);

  std::set<int32_t> marker_ids;
  std::map<int32_t, const Marker *> markers_by_bulb_id;
  for (const auto & marker : markers) {
    marker_ids.insert(marker.id);
    markers_by_bulb_id[marker.id] = &marker;
  }
  EXPECT_EQ(marker_ids, (std::set<int32_t>{1, 2, 3, 4}));
  EXPECT_DOUBLE_EQ(markers_by_bulb_id.at(1)->pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(markers_by_bulb_id.at(2)->pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(markers_by_bulb_id.at(3)->pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(markers_by_bulb_id.at(4)->pose.position.x, 4.0);
}
