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

#include "autoware/simpl_prediction/conversion/lanelet.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>

namespace autoware::simpl_prediction::test
{
using autoware::simpl_prediction::conversion::LaneletConverter;

TEST(TestLaneletConversion, ConvertMinimalRoadLanelet)
{
  // Create simple line string (centerline)
  lanelet::LineString3d centerline(
    lanelet::InvalId, {lanelet::Point3d{lanelet::InvalId, 0.0, 0.0, 0.0},
                       lanelet::Point3d{lanelet::InvalId, 1.0, 0.0, 0.0}});
  centerline.setAttribute("type", "centerline");

  // Create left/right boundaries
  lanelet::LineString3d left_bound(
    lanelet::InvalId, {lanelet::Point3d{lanelet::InvalId, 0.0, 1.0, 0.0},
                       lanelet::Point3d{lanelet::InvalId, 1.0, 1.0, 0.0}});
  left_bound.setAttribute("type", "line_thin");

  lanelet::LineString3d right_bound(
    lanelet::InvalId, {lanelet::Point3d{lanelet::InvalId, 0.0, -1.0, 0.0},
                       lanelet::Point3d{lanelet::InvalId, 1.0, -1.0, 0.0}});
  right_bound.setAttribute("type", "line_thick");

  // Construct a lanelet
  lanelet::Lanelet lanelet(lanelet::InvalId, left_bound, right_bound);
  lanelet.attributes()["subtype"] = "road";

  // Add to map
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lanelet);
  map->add(left_bound);
  map->add(right_bound);
  map->add(centerline);

  // Run converter
  LaneletConverter converter;
  converter.convert(map);

  // Check result
  const auto result = converter.polylines();
  ASSERT_TRUE(result.has_value());
  ASSERT_GE(result->size(), 3u);  // centerline + left + right

  for (const auto & polyline : *result) {
    EXPECT_GE(polyline.size(), 2u);
  }
}

TEST(TestLaneletConversion, ConvertCrosswalkPolygon)
{
  // Create a polygon (crosswalk)
  lanelet::LineString3d polygon(
    lanelet::InvalId, {lanelet::Point3d{lanelet::InvalId, 0.0, 0.0, 0.0},
                       lanelet::Point3d{lanelet::InvalId, 1.0, 0.0, 0.0},
                       lanelet::Point3d{lanelet::InvalId, 1.0, 1.0, 0.0},
                       lanelet::Point3d{lanelet::InvalId, 0.0, 1.0, 0.0},
                       lanelet::Point3d{lanelet::InvalId, 0.0, 0.0, 0.0}});

  lanelet::Lanelet crosswalk(lanelet::InvalId, polygon, polygon);
  crosswalk.attributes()["subtype"] = "crosswalk";

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(crosswalk);
  map->add(polygon);

  LaneletConverter converter;
  converter.convert(map);

  auto result = converter.polylines();
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->empty());
}
}  // namespace autoware::simpl_prediction::test
