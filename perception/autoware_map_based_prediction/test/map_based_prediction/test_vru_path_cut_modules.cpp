// Copyright 2026 TIER IV, inc.
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

#include "autoware/map_based_prediction/predictor_vru/fence.hpp"
#include "autoware/map_based_prediction/predictor_vru/road_boundary.hpp"
#include "autoware/map_based_prediction/predictor_vru/vegetation.hpp"
#include "autoware/map_based_prediction/utils.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/utility/Utilities.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction
{
namespace
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;

lanelet::Point3d make_point(const double x, const double y)
{
  return lanelet::Point3d(lanelet::utils::getId(), x, y, 0.0);
}

lanelet::LineString3d make_linestring(
  const std::vector<std::pair<double, double>> & points, const std::string & type = "")
{
  lanelet::LineString3d linestring(lanelet::utils::getId());
  for (const auto & [x, y] : points) {
    linestring.push_back(make_point(x, y));
  }
  if (!type.empty()) {
    linestring.attributes()[lanelet::AttributeName::Type] = type;
  }
  return linestring;
}

lanelet::Polygon3d make_vegetation_polygon(const std::vector<std::pair<double, double>> & points)
{
  lanelet::Polygon3d polygon(lanelet::utils::getId());
  for (const auto & [x, y] : points) {
    polygon.push_back(make_point(x, y));
  }
  polygon.attributes()[lanelet::AttributeName::Type] = "area";
  polygon.attributes()[lanelet::AttributeName::Subtype] = "vegetation";
  return polygon;
}

lanelet::Lanelet make_lanelet(
  const lanelet::LineString3d & left, const lanelet::LineString3d & right,
  const std::string & subtype)
{
  lanelet::Lanelet lanelet(lanelet::utils::getId(), left, right);
  lanelet.attributes()[lanelet::AttributeName::Subtype] = subtype;
  return lanelet;
}

// Straight path along the x-axis with 1 m spacing, poses at x = 0 .. n_poses - 1.
PredictedPath make_straight_path(const size_t n_poses)
{
  PredictedPath path;
  for (auto i = 0UL; i < n_poses; ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = static_cast<double>(i);
    pose.orientation.w = 1.0;
    path.path.push_back(pose);
  }
  return path;
}

Shape cylinder_shape(const double diameter)
{
  Shape shape;
  shape.type = Shape::CYLINDER;
  shape.dimensions.x = diameter;
  return shape;
}

Shape bounding_box_shape(const double length, const double width)
{
  Shape shape;
  shape.type = Shape::BOUNDING_BOX;
  shape.dimensions.x = length;
  shape.dimensions.y = width;
  return shape;
}

lanelet::BasicLineString2d full_linestring(const PredictedPath & path)
{
  return utils::to_linestring_2d(path.path, path.path.size() - 1);
}

class VegetationModuleTest : public ::testing::Test
{
protected:
  VegetationModule build_module(const std::vector<lanelet::Polygon3d> & polygons)
  {
    auto map = std::make_shared<lanelet::LaneletMap>();
    for (const auto & polygon : polygons) {
      map->add(polygon);
    }
    VegetationModule module;
    module.buildFromMap(map);
    return module;
  }
};

TEST_F(VegetationModuleTest, EarliestCrossingWins)
{
  // Two polygons over the path; the cut belongs to the nearer one at x in [2.2, 3].
  const auto module = build_module(
    {make_vegetation_polygon({{6.2, -1.0}, {7.0, -1.0}, {7.0, 1.0}, {6.2, 1.0}}),
     make_vegetation_polygon({{2.2, -1.0}, {3.0, -1.0}, {3.0, 1.0}, {2.2, 1.0}})});
  const auto path = make_straight_path(11);
  const auto cut =
    module.cutPathsCrossingVegetation(path, full_linestring(path), cylinder_shape(1.0));
  // Segment (1,0)-(2,0) is the first within 0.5 m of the nearer polygon.
  ASSERT_EQ(cut.path.size(), 2UL);
}

TEST_F(VegetationModuleTest, InitialOverlapIsNotACrossing)
{
  const auto module =
    build_module({make_vegetation_polygon({{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}})});
  const auto path = make_straight_path(11);
  const auto cut =
    module.cutPathsCrossingVegetation(path, full_linestring(path), cylinder_shape(1.0));
  ASSERT_EQ(cut.path.size(), 11UL);
}

TEST_F(VegetationModuleTest, PolygonWithinFootprintRadiusOfCenterlineIsDetected)
{
  // The polygon never touches the centerline (y >= 0.3) but the 0.5 m footprint sweeps into it.
  const auto module =
    build_module({make_vegetation_polygon({{4.0, 0.3}, {5.0, 0.3}, {5.0, 1.0}, {4.0, 1.0}})});
  const auto path = make_straight_path(11);
  const auto cut =
    module.cutPathsCrossingVegetation(path, full_linestring(path), cylinder_shape(1.0));
  ASSERT_EQ(cut.path.size(), 4UL);
}

TEST_F(VegetationModuleTest, BoundingBoxShapeCuts)
{
  const auto module =
    build_module({make_vegetation_polygon({{4.2, -1.0}, {5.0, -1.0}, {5.0, 1.0}, {4.2, 1.0}})});
  const auto path = make_straight_path(11);
  const auto cut =
    module.cutPathsCrossingVegetation(path, full_linestring(path), bounding_box_shape(1.0, 0.6));
  ASSERT_EQ(cut.path.size(), 4UL);
}

TEST_F(VegetationModuleTest, VegetationFreeMapLeavesPathUntouched)
{
  const auto module = build_module({});
  const auto path = make_straight_path(11);
  const auto cut =
    module.cutPathsCrossingVegetation(path, full_linestring(path), cylinder_shape(1.0));
  ASSERT_EQ(cut.path.size(), 11UL);
}

TEST_F(VegetationModuleTest, InvalidPolygonIsSkippedWithoutDisablingValidOnes)
{
  // The degenerate two-point polygon is rejected at load; the valid one still cuts.
  const auto module = build_module(
    {make_vegetation_polygon({{2.2, -1.0}, {3.0, -1.0}}),
     make_vegetation_polygon({{6.2, -1.0}, {7.0, -1.0}, {7.0, 1.0}, {6.2, 1.0}})});
  const auto path = make_straight_path(11);
  const auto cut =
    module.cutPathsCrossingVegetation(path, full_linestring(path), cylinder_shape(1.0));
  ASSERT_EQ(cut.path.size(), 6UL);
}

TEST_F(VegetationModuleTest, CrossingBeyondArrivalIndexIsIgnored)
{
  const auto module =
    build_module({make_vegetation_polygon({{6.2, -1.0}, {7.0, -1.0}, {7.0, 1.0}, {6.2, 1.0}})});
  PredictedPathWithArrivalIndex path;
  static_cast<PredictedPath &>(path) = make_straight_path(11);
  path.arrival_index = 4;
  const auto path_ls = utils::to_linestring_2d(path.path, path.arrival_index);
  EXPECT_FALSE(
    module.doesPathCrossAnyVegetationBeforeCrosswalk(path, path_ls, cylinder_shape(1.0)));
  path.arrival_index = 10;
  const auto full_ls = utils::to_linestring_2d(path.path, path.arrival_index);
  EXPECT_TRUE(module.doesPathCrossAnyVegetationBeforeCrosswalk(path, full_ls, cylinder_shape(1.0)));
}

TEST(FenceModuleTest, EarliestFenceCrossingWins)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(make_linestring({{6.5, -1.0}, {6.5, 1.0}}, "fence"));
  map->add(make_linestring({{2.5, -1.0}, {2.5, 1.0}}, "fence"));
  FenceModule module;
  module.buildFromMap(map);

  const auto path = make_straight_path(11);
  const auto cut = module.cutPathBeforeFences(path, full_linestring(path));
  // Segment (2,0)-(3,0) crosses the nearer fence at x = 2.5.
  ASSERT_EQ(cut.path.size(), 3UL);
}

class RoadBoundaryModuleTest : public ::testing::Test
{
protected:
  // One road lanelet with outer bounds at x = 2.5 and x = 4, and an adjacent one sharing the
  // bound at x = 4 with its own outer bound at x = 6. All bounds run along the y-axis.
  void SetUp() override
  {
    map_ = std::make_shared<lanelet::LaneletMap>();
    const auto bound_a = make_linestring({{2.5, -10.0}, {2.5, 10.0}});
    const auto bound_shared = make_linestring({{4.0, -10.0}, {4.0, 10.0}});
    const auto bound_b = make_linestring({{6.0, -10.0}, {6.0, 10.0}});
    map_->add(make_lanelet(bound_a, bound_shared, "road"));
    map_->add(make_lanelet(bound_shared, bound_b, "road"));
  }

  RoadBoundaryModule build_module()
  {
    RoadBoundaryModule module;
    module.build_from_map(map_);
    utils::ObjectDecelerationParams deceleration_params;
    deceleration_params.per_label.emplace(ObjectClassification::PEDESTRIAN, -0.5);
    module.set_object_deceleration(deceleration_params);
    return module;
  }

  static PredictedObject make_pedestrian(const double speed, PredictedPath path)
  {
    PredictedObject object;
    ObjectClassification classification;
    classification.label = ObjectClassification::PEDESTRIAN;
    classification.probability = 1.0;
    object.classification.push_back(classification);
    object.kinematics.initial_twist_with_covariance.twist.linear.x = speed;
    object.kinematics.predicted_paths.push_back(std::move(path));
    return object;
  }

  static bool never_red(const lanelet::ConstLanelet &) { return false; }
  static bool always_red(const lanelet::ConstLanelet &) { return true; }

  std::shared_ptr<lanelet::LaneletMap> map_;
};

TEST_F(RoadBoundaryModuleTest, SlowObjectIsCutAtOuterBoundary)
{
  const auto module = build_module();
  const auto object = make_pedestrian(0.5, make_straight_path(11));
  const auto cut_paths = module.cut_paths_crossing_road_boundary(object, false, never_red);
  ASSERT_EQ(cut_paths.size(), 1UL);
  // Segment (2,0)-(3,0) crosses the outer bound at x = 2.5.
  ASSERT_EQ(cut_paths.front().path.size(), 3UL);
}

TEST_F(RoadBoundaryModuleTest, InteriorDividerIsNotABoundary)
{
  const auto module = build_module();
  // Path from x = 3 to x = 3.9: inside the first lanelet, reaching toward the shared bound only.
  PredictedPath path;
  for (const auto x : {3.0, 3.5, 3.9}) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.orientation.w = 1.0;
    path.path.push_back(pose);
  }
  // Extend across the shared bound at x = 4 but short of the outer bound at x = 6.
  geometry_msgs::msg::Pose pose;
  pose.position.x = 5.0;
  pose.orientation.w = 1.0;
  path.path.push_back(pose);
  const auto object = make_pedestrian(0.5, path);
  const auto cut_paths = module.cut_paths_crossing_road_boundary(object, false, never_red);
  ASSERT_EQ(cut_paths.front().path.size(), 4UL);
}

TEST_F(RoadBoundaryModuleTest, CrosswalkCrossingIsExemptUnlessRed)
{
  const auto left = make_linestring({{2.0, -1.0}, {3.0, -1.0}});
  const auto right = make_linestring({{2.0, 1.0}, {3.0, 1.0}});
  map_->add(make_lanelet(left, right, "crosswalk"));
  const auto module = build_module();

  // Six poses so the exempt crossing at x = 2.5 is the only boundary the path reaches.
  const auto object = make_pedestrian(0.5, make_straight_path(6));
  const auto kept = module.cut_paths_crossing_road_boundary(object, false, never_red);
  ASSERT_EQ(kept.front().path.size(), 6UL);
  const auto cut = module.cut_paths_crossing_road_boundary(object, false, always_red);
  ASSERT_EQ(cut.front().path.size(), 3UL);
}

TEST_F(RoadBoundaryModuleTest, UnstoppableObjectKeepsItsPath)
{
  const auto module = build_module();
  const auto object = make_pedestrian(5.0, make_straight_path(11));
  const auto cut_paths = module.cut_paths_crossing_road_boundary(object, false, never_red);
  ASSERT_EQ(cut_paths.front().path.size(), 11UL);
}

}  // namespace
}  // namespace autoware::map_based_prediction
