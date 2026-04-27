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

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_perception_msgs/msg/traffic_light_element.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <string>
#include <vector>

namespace
{
using autoware::traffic_light::extract_bulbs;
using autoware_perception_msgs::msg::TrafficLightElement;
using lanelet::AutowareTrafficLightConstPtr;
using lanelet::LineString3d;
using lanelet::LineStringOrPolygon3d;
using lanelet::Point3d;
using lanelet::utils::getId;

Point3d make_bulb_point(double x, double y, double z, const std::string & color)
{
  Point3d point(getId(), x, y, z);
  point.attributes()["color"] = color;
  return point;
}

Point3d make_bulb_point_without_color(double x, double y, double z)
{
  return Point3d(getId(), x, y, z);
}

// Builds an AutowareTrafficLight regulatory element wrapping the given bulb
// points in a single lightBulbs linestring. The base linestring is a dummy
// required by AutowareTrafficLight::make() but unused by extract_bulbs.
AutowareTrafficLightConstPtr make_traffic_light(
  const std::vector<Point3d> & bulb_points, bool with_traffic_light_id_attribute = true)
{
  LineString3d light_bulbs(getId(), bulb_points);
  if (with_traffic_light_id_attribute) {
    light_bulbs.attributes()["traffic_light_id"] = "1";
  }

  LineString3d base(getId(), {Point3d(getId(), 0, 0, 0), Point3d(getId(), 1, 0, 0)});

  return lanelet::autoware::AutowareTrafficLight::make(
    getId(), lanelet::AttributeMap(), {LineStringOrPolygon3d(base)}, {}, {light_bulbs});
}

}  // namespace

TEST(ExtractBulbs, EmptyInputProducesEmptyResult)
{
  std::vector<AutowareTrafficLightConstPtr> map_traffic_lights;

  auto bulbs = extract_bulbs(map_traffic_lights);

  EXPECT_TRUE(bulbs.empty());
}

TEST(ExtractBulbs, RedGreenYellowResolveToCorrectColors)
{
  auto traffic_light = make_traffic_light({
    make_bulb_point(1, 2, 3, "red"),
    make_bulb_point(4, 5, 6, "green"),
    make_bulb_point(7, 8, 9, "yellow"),
  });

  auto bulbs = extract_bulbs({traffic_light});

  ASSERT_EQ(bulbs.size(), 1u);
  auto it = bulbs.find(traffic_light->id());
  ASSERT_NE(it, bulbs.end());
  ASSERT_EQ(it->second.size(), 3u);
  EXPECT_EQ(it->second[0].color, TrafficLightElement::RED);
  EXPECT_EQ(it->second[1].color, TrafficLightElement::GREEN);
  EXPECT_EQ(it->second[2].color, TrafficLightElement::AMBER);
}

TEST(ExtractBulbs, BulbCarriesPointIdAndPosition)
{
  auto point = make_bulb_point(1.5, 2.5, 3.5, "red");
  auto traffic_light = make_traffic_light({point});

  auto bulbs = extract_bulbs({traffic_light});

  ASSERT_EQ(bulbs.size(), 1u);
  auto it = bulbs.find(traffic_light->id());
  ASSERT_NE(it, bulbs.end());
  ASSERT_EQ(it->second.size(), 1u);
  EXPECT_EQ(it->second[0].id, point.id());
  EXPECT_DOUBLE_EQ(it->second[0].position.x, 1.5);
  EXPECT_DOUBLE_EQ(it->second[0].position.y, 2.5);
  EXPECT_DOUBLE_EQ(it->second[0].position.z, 3.5);
}

TEST(ExtractBulbs, PointWithoutColorAttributeIsSkipped)
{
  auto traffic_light = make_traffic_light({
    make_bulb_point(0, 0, 0, "red"),
    make_bulb_point_without_color(1, 1, 1),
  });

  auto bulbs = extract_bulbs({traffic_light});

  ASSERT_EQ(bulbs.size(), 1u);
  EXPECT_EQ(bulbs.begin()->second.size(), 1u);
}

TEST(ExtractBulbs, UnknownColorIsSkipped)
{
  auto traffic_light = make_traffic_light({make_bulb_point(0, 0, 0, "blue")});

  auto bulbs = extract_bulbs({traffic_light});

  EXPECT_TRUE(bulbs.empty());
}

TEST(ExtractBulbs, LightBulbsWithoutTrafficLightIdAttributeIsSkipped)
{
  auto traffic_light = make_traffic_light(
    {make_bulb_point(0, 0, 0, "red")}, /*with_traffic_light_id_attribute=*/false);

  auto bulbs = extract_bulbs({traffic_light});

  EXPECT_TRUE(bulbs.empty());
}

TEST(ExtractBulbs, MultipleTrafficLightsProduceMultipleEntries)
{
  auto first = make_traffic_light({make_bulb_point(0, 0, 0, "red")});
  auto second = make_traffic_light({make_bulb_point(1, 1, 1, "green")});

  auto bulbs = extract_bulbs({first, second});

  EXPECT_EQ(bulbs.size(), 2u);
  EXPECT_NE(bulbs.find(first->id()), bulbs.end());
  EXPECT_NE(bulbs.find(second->id()), bulbs.end());
}
