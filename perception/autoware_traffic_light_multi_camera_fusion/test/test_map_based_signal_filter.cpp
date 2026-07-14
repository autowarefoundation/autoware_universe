// Copyright 2026 TIER IV, Inc.
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

#include "../src/map_based_signal_filter.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>

#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <string>
#include <vector>

namespace
{
using autoware::traffic_light::MapBasedSignalFilter;
using T4Element = tier4_perception_msgs::msg::TrafficLightElement;

constexpr lanelet::Id TRAFFIC_LIGHT_ID = 11;
constexpr lanelet::Id OTHER_TRAFFIC_LIGHT_ID = 12;
constexpr lanelet::Id UNMAPPED_TRAFFIC_LIGHT_ID = 999;
constexpr lanelet::Id REGULATORY_ELEMENT_ID = 100;

// Build a bulb point with "color" (and optional "arrow") attributes at a distinct id/position
// so lanelet2's internal invariants (unique ids) are respected.
lanelet::Point3d make_bulb_point(
  lanelet::Id id, const std::string & color, const std::string & arrow = "")
{
  lanelet::Point3d point(id, 0.0, 0.0, 3.5);
  point.attributes()["color"] = color;
  if (!arrow.empty()) {
    point.attributes()["arrow"] = arrow;
  }
  return point;
}

// Build a light_bulbs LineString3d whose "traffic_light_id" attribute references the passed
// traffic light linestring id. The bulb points define the allowed (color, shape) pairs.
lanelet::LineString3d make_light_bulbs_linestring(
  lanelet::Id linestring_id, lanelet::Id traffic_light_id,
  const std::vector<lanelet::Point3d> & bulbs)
{
  lanelet::LineString3d ls(linestring_id, bulbs);
  ls.attributes()["traffic_light_id"] = std::to_string(traffic_light_id);
  return ls;
}

// Small helper: a lanelet map holding a single traffic light regulatory element with the
// passed light bulb definitions.
lanelet::LaneletMapPtr make_map_with_single_traffic_light(
  const std::vector<lanelet::LineString3d> & light_bulbs_linestrings)
{
  lanelet::Point3d road_left_start(1, 0.0, 2.0, 0.0);
  lanelet::Point3d road_left_end(2, 30.0, 2.0, 0.0);
  lanelet::Point3d road_right_start(3, 0.0, -2.0, 0.0);
  lanelet::Point3d road_right_end(4, 30.0, -2.0, 0.0);
  lanelet::LineString3d road_left(5, {road_left_start, road_left_end});
  lanelet::LineString3d road_right(6, {road_right_start, road_right_end});

  auto road_lanelet = lanelet::Lanelet(1000, road_left, road_right);
  road_lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;

  lanelet::Point3d tl_left_end(7, 19.5, 0.5, 3.5);
  lanelet::Point3d tl_right_end(8, 19.5, -0.5, 3.5);
  lanelet::LineString3d traffic_light_linestring(TRAFFIC_LIGHT_ID, {tl_left_end, tl_right_end});
  traffic_light_linestring.attributes()["subtype"] = "red_yellow_green";
  traffic_light_linestring.attributes()["height"] = "1.0";

  auto traffic_light_regulatory_element = lanelet::autoware::AutowareTrafficLight::make(
    REGULATORY_ELEMENT_ID, lanelet::AttributeMap(), {traffic_light_linestring}, {},
    light_bulbs_linestrings);
  road_lanelet.addRegulatoryElement(traffic_light_regulatory_element);

  auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet_map->add(road_lanelet);
  return lanelet_map;
}

T4Element make_element(uint8_t color, uint8_t shape, float confidence = 0.9f)
{
  T4Element element;
  element.color = color;
  element.shape = shape;
  element.status = T4Element::SOLID_ON;
  element.confidence = confidence;
  return element;
}

}  // namespace

TEST(MapBasedSignalFilter, NullLaneletMapProducesEmptyFilter)
{
  const MapBasedSignalFilter filter(lanelet::LaneletMapConstPtr{});

  EXPECT_FALSE(filter.has_rules_for(TRAFFIC_LIGHT_ID));
  std::vector<T4Element> elements = {make_element(T4Element::RED, T4Element::CIRCLE)};
  EXPECT_FALSE(filter.filter_elements(TRAFFIC_LIGHT_ID, elements));
  ASSERT_EQ(elements.size(), 1u);
}

TEST(MapBasedSignalFilter, BulbsWithoutArrowAttributeMapToCircleShape)
{
  // a red-yellow-green light with no arrow attributes should allow only CIRCLE-shaped predictions
  auto light_bulbs_linestring = make_light_bulbs_linestring(
    200, TRAFFIC_LIGHT_ID,
    {make_bulb_point(20, "red"), make_bulb_point(21, "yellow"), make_bulb_point(22, "green")});
  const auto lanelet_map = make_map_with_single_traffic_light({light_bulbs_linestring});
  const MapBasedSignalFilter filter(lanelet_map);

  ASSERT_TRUE(filter.has_rules_for(TRAFFIC_LIGHT_ID));
  const auto allowed = filter.get_allowed_bulbs(TRAFFIC_LIGHT_ID);
  EXPECT_EQ(allowed.count({T4Element::RED, T4Element::CIRCLE}), 1u);
  EXPECT_EQ(allowed.count({T4Element::AMBER, T4Element::CIRCLE}), 1u);
  EXPECT_EQ(allowed.count({T4Element::GREEN, T4Element::CIRCLE}), 1u);
  EXPECT_EQ(allowed.size(), 3u);
}

TEST(MapBasedSignalFilter, ArrowBulbsPopulateAllowedShapes)
{
  // a light with a red circle plus a green left arrow should allow that specific (color, shape)
  auto light_bulbs_linestring = make_light_bulbs_linestring(
    200, TRAFFIC_LIGHT_ID,
    {make_bulb_point(20, "red"), make_bulb_point(21, "green", "left"),
     make_bulb_point(22, "green", "up_right")});
  const auto lanelet_map = make_map_with_single_traffic_light({light_bulbs_linestring});
  const MapBasedSignalFilter filter(lanelet_map);

  const auto allowed = filter.get_allowed_bulbs(TRAFFIC_LIGHT_ID);
  EXPECT_EQ(allowed.count({T4Element::RED, T4Element::CIRCLE}), 1u);
  EXPECT_EQ(allowed.count({T4Element::GREEN, T4Element::LEFT_ARROW}), 1u);
  EXPECT_EQ(allowed.count({T4Element::GREEN, T4Element::UP_RIGHT_ARROW}), 1u);
  EXPECT_EQ(allowed.size(), 3u);
}

TEST(MapBasedSignalFilter, FilterDropsElementsOutsideAllowedSet)
{
  // map only allows red/yellow/green circles. LEFT_ARROW must be filtered out
  auto light_bulbs_linestring = make_light_bulbs_linestring(
    200, TRAFFIC_LIGHT_ID,
    {make_bulb_point(20, "red"), make_bulb_point(21, "yellow"), make_bulb_point(22, "green")});
  const auto lanelet_map = make_map_with_single_traffic_light({light_bulbs_linestring});
  const MapBasedSignalFilter filter(lanelet_map);

  std::vector<T4Element> elements = {
    make_element(T4Element::RED, T4Element::CIRCLE),
    make_element(T4Element::GREEN, T4Element::LEFT_ARROW),
  };
  EXPECT_TRUE(filter.filter_elements(TRAFFIC_LIGHT_ID, elements));
  ASSERT_EQ(elements.size(), 1u);
  EXPECT_EQ(elements[0].color, T4Element::RED);
  EXPECT_EQ(elements[0].shape, T4Element::CIRCLE);
}

TEST(MapBasedSignalFilter, FilterKeepsAllElementsWhenAllValid)
{
  auto light_bulbs_linestring = make_light_bulbs_linestring(
    200, TRAFFIC_LIGHT_ID,
    {make_bulb_point(20, "red"), make_bulb_point(21, "green"),
     make_bulb_point(22, "green", "left")});
  const auto lanelet_map = make_map_with_single_traffic_light({light_bulbs_linestring});
  const MapBasedSignalFilter filter(lanelet_map);

  std::vector<T4Element> elements = {
    make_element(T4Element::RED, T4Element::CIRCLE),
    make_element(T4Element::GREEN, T4Element::LEFT_ARROW),
  };
  EXPECT_FALSE(filter.filter_elements(TRAFFIC_LIGHT_ID, elements));
  EXPECT_EQ(elements.size(), 2u);
}

TEST(MapBasedSignalFilter, FilterDropsAllInvalidElementsProducingEmpty)
{
  auto light_bulbs_linestring = make_light_bulbs_linestring(
    200, TRAFFIC_LIGHT_ID, {make_bulb_point(20, "red"), make_bulb_point(21, "green")});
  const auto lanelet_map = make_map_with_single_traffic_light({light_bulbs_linestring});
  const MapBasedSignalFilter filter(lanelet_map);

  std::vector<T4Element> elements = {
    make_element(T4Element::GREEN, T4Element::LEFT_ARROW),
    make_element(T4Element::AMBER, T4Element::CIRCLE),
  };
  EXPECT_TRUE(filter.filter_elements(TRAFFIC_LIGHT_ID, elements));
  EXPECT_TRUE(elements.empty());
}

TEST(MapBasedSignalFilter, UnknownElementsAlwaysPassThrough)
{
  auto light_bulbs_linestring =
    make_light_bulbs_linestring(200, TRAFFIC_LIGHT_ID, {make_bulb_point(20, "red")});
  const auto lanelet_map = make_map_with_single_traffic_light({light_bulbs_linestring});
  const MapBasedSignalFilter filter(lanelet_map);

  std::vector<T4Element> elements = {
    make_element(T4Element::UNKNOWN, T4Element::UNKNOWN),
    make_element(T4Element::GREEN, T4Element::CIRCLE),  // not allowed -> should be removed
  };
  EXPECT_TRUE(filter.filter_elements(TRAFFIC_LIGHT_ID, elements));
  ASSERT_EQ(elements.size(), 1u);
  EXPECT_EQ(elements[0].color, T4Element::UNKNOWN);
  EXPECT_EQ(elements[0].shape, T4Element::UNKNOWN);
}

TEST(MapBasedSignalFilter, IdWithoutBulbsIsNotFilteredAgainst)
{
  // no light_bulbs at all in the map -> no rules, filter is a no-op for any id
  const auto lanelet_map = make_map_with_single_traffic_light({});
  const MapBasedSignalFilter filter(lanelet_map);

  EXPECT_FALSE(filter.has_rules_for(TRAFFIC_LIGHT_ID));

  std::vector<T4Element> elements = {make_element(T4Element::GREEN, T4Element::LEFT_ARROW)};
  EXPECT_FALSE(filter.filter_elements(TRAFFIC_LIGHT_ID, elements));
  ASSERT_EQ(elements.size(), 1u);
  EXPECT_EQ(elements[0].color, T4Element::GREEN);
  EXPECT_EQ(elements[0].shape, T4Element::LEFT_ARROW);
}

TEST(MapBasedSignalFilter, UnknownIdInMapIsNotFilteredAgainst)
{
  // the map has rules for TRAFFIC_LIGHT_ID only. a prediction for a different id passes through
  // because we have no rules to apply
  auto light_bulbs_linestring =
    make_light_bulbs_linestring(200, TRAFFIC_LIGHT_ID, {make_bulb_point(20, "red")});
  const auto lanelet_map = make_map_with_single_traffic_light({light_bulbs_linestring});
  const MapBasedSignalFilter filter(lanelet_map);

  EXPECT_FALSE(filter.has_rules_for(UNMAPPED_TRAFFIC_LIGHT_ID));

  std::vector<T4Element> elements = {make_element(T4Element::GREEN, T4Element::LEFT_ARROW)};
  EXPECT_FALSE(filter.filter_elements(UNMAPPED_TRAFFIC_LIGHT_ID, elements));
  ASSERT_EQ(elements.size(), 1u);
  EXPECT_EQ(elements[0].color, T4Element::GREEN);
  EXPECT_EQ(elements[0].shape, T4Element::LEFT_ARROW);
}

TEST(MapBasedSignalFilter, BulbsWithoutTrafficLightIdAttributeAreIgnored)
{
  // a light_bulbs linestring without the "traffic_light_id" attribute must be skipped, otherwise
  // the filter would silently apply the rules to the wrong id
  lanelet::LineString3d bad_ls(200, {make_bulb_point(20, "red")});
  // deliberately no "traffic_light_id" attribute
  const auto lanelet_map = make_map_with_single_traffic_light({bad_ls});
  const MapBasedSignalFilter filter(lanelet_map);

  EXPECT_FALSE(filter.has_rules_for(TRAFFIC_LIGHT_ID));
}

TEST(MapBasedSignalFilter, BulbsForDifferentTrafficLightIdAreKeyedCorrectly)
{
  // a single light_bulbs linestring can reference OTHER_TRAFFIC_LIGHT_ID; filtering for
  // TRAFFIC_LIGHT_ID must be untouched
  auto light_bulbs_linestring =
    make_light_bulbs_linestring(200, OTHER_TRAFFIC_LIGHT_ID, {make_bulb_point(20, "red")});
  const auto lanelet_map = make_map_with_single_traffic_light({light_bulbs_linestring});
  const MapBasedSignalFilter filter(lanelet_map);

  EXPECT_TRUE(filter.has_rules_for(OTHER_TRAFFIC_LIGHT_ID));
  EXPECT_FALSE(filter.has_rules_for(TRAFFIC_LIGHT_ID));

  std::vector<T4Element> elements = {make_element(T4Element::GREEN, T4Element::LEFT_ARROW)};
  EXPECT_FALSE(filter.filter_elements(TRAFFIC_LIGHT_ID, elements));
  EXPECT_TRUE(filter.filter_elements(OTHER_TRAFFIC_LIGHT_ID, elements));
  EXPECT_TRUE(elements.empty());
}
