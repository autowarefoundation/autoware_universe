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

#include "environment_classifier.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <memory>
#include <optional>
#include <string>

namespace autoware::environment_adaptor
{

static autoware_map_msgs::msg::LaneletMapBin make_map_bin(
  const std::string & subtype,
  const std::optional<double> map_longitudinal_scale_factor = std::nullopt)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Polygon3d poly(
    lanelet::utils::getId(), {
                               lanelet::Point3d(lanelet::utils::getId(), -1.0, -1.0, 0.0),
                               lanelet::Point3d(lanelet::utils::getId(), 1.0, -1.0, 0.0),
                               lanelet::Point3d(lanelet::utils::getId(), 1.0, 1.0, 0.0),
                               lanelet::Point3d(lanelet::utils::getId(), -1.0, 1.0, 0.0),
                             });
  poly.setAttribute(lanelet::AttributeName::Type, "degenerate_area");
  poly.setAttribute(lanelet::AttributeName::Subtype, subtype);
  if (map_longitudinal_scale_factor.has_value()) {
    poly.setAttribute(
      "longitudinal_scale_factor", std::to_string(map_longitudinal_scale_factor.value()));
  }
  map->add(poly);
  return autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
}

static EnvironmentClassifier::Param base_param()
{
  EnvironmentClassifier::Param param;
  param.default_environment_id = 0;
  param.default_longitudinal_scale_factor = 1.0;
  param.area_subtype_to_environment_id["uniform_road"] = 1;
  return param;
}

static geometry_msgs::msg::Point make_point(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

TEST(EnvironmentClassifierTest, test_map_not_ready)
{
  EnvironmentClassifier classifier;
  classifier.set_param(base_param());

  const auto result = classifier.classify(make_point(0.0, 0.0));
  EXPECT_EQ(result.environment_id, 0);
  EXPECT_DOUBLE_EQ(result.longitudinal_scale_factor, 1.0);
}

TEST(EnvironmentClassifierTest, test_point_outside_polygon)
{
  EnvironmentClassifier classifier;
  classifier.set_param(base_param());
  classifier.load_map(make_map_bin("uniform_road"));

  const auto result = classifier.classify(make_point(10.0, 10.0));
  EXPECT_EQ(result.environment_id, 0);
  EXPECT_DOUBLE_EQ(result.longitudinal_scale_factor, 1.0);
}

TEST(EnvironmentClassifierTest, test_point_inside_polygon_default_fallback)
{
  EnvironmentClassifier classifier;
  classifier.set_param(base_param());
  classifier.load_map(make_map_bin("uniform_road"));

  // Polygon has no longitudinal_scale_factor attribute, so default is applied.
  const auto result = classifier.classify(make_point(0.0, 0.0));
  EXPECT_EQ(result.environment_id, 1);
  EXPECT_DOUBLE_EQ(result.longitudinal_scale_factor, 1.0);
}

TEST(EnvironmentClassifierTest, test_point_inside_polygon_map_attribute)
{
  EnvironmentClassifier classifier;
  classifier.set_param(base_param());
  classifier.load_map(make_map_bin("uniform_road", 1.02));

  const auto result = classifier.classify(make_point(0.0, 0.0));
  EXPECT_EQ(result.environment_id, 1);
  EXPECT_DOUBLE_EQ(result.longitudinal_scale_factor, 1.02);
}

TEST(EnvironmentClassifierTest, test_point_inside_polygon_unknown_subtype)
{
  EnvironmentClassifier classifier;
  classifier.set_param(base_param());
  classifier.load_map(make_map_bin("unknown_subtype"));

  const auto result = classifier.classify(make_point(0.0, 0.0));
  EXPECT_EQ(result.environment_id, 0);
  EXPECT_DOUBLE_EQ(result.longitudinal_scale_factor, 1.0);
}

}  // namespace autoware::environment_adaptor

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
