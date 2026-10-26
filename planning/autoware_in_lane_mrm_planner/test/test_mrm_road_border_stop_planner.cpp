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

#include "mrm_road_border_stop_planner.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineString.h>

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::in_lane_mrm_planner
{
namespace
{

// wheel_base 2.74 + front_overhang 1.0 -> max_longitudinal_offset 3.74
// wheel_tread 1.63 + overhangs 0.1 * 2 -> vehicle_width 1.83 (half width 0.915)
VehicleInfo make_vehicle_info()
{
  return autoware::vehicle_info_utils::createVehicleInfo(
    0.39, 0.42, 2.74, 1.63, 1.0, 1.03, 0.1, 0.1, 2.5, 0.7);
}

constexpr double kFrontOffset = 3.74;
constexpr double kHalfWidth = 0.915;
constexpr double kTrajStartX = -5.0;
constexpr double kTrajEndX = 100.0;
constexpr double kTrajInterval = 0.5;
constexpr double kVelocity = 10.0;

Params make_params()
{
  Params params;
  params.road_border_stop.enable = true;
  params.road_border_stop.boundary_types_to_detect = {"road_border"};
  params.road_border_stop.stop_margin = 1.0;
  params.road_border_stop.lateral_margin = 0.0;
  params.road_border_stop.longitudinal_margin = 0.0;
  params.road_border_stop.max_check_length = 100.0;
  params.road_border_stop.use_height_filter = true;
  return params;
}

TrajectoryPoints make_straight_trajectory()
{
  TrajectoryPoints points;
  for (double x = kTrajStartX; x <= kTrajEndX + 1e-6; x += kTrajInterval) {
    TrajectoryPoint p;
    p.pose.position.x = x;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.0;
    p.pose.orientation.w = 1.0;
    p.longitudinal_velocity_mps = static_cast<float>(kVelocity);
    points.push_back(p);
  }
  return points;
}

Odometry make_ego_odometry(const double x = 0.0)
{
  Odometry odom;
  odom.header.frame_id = "map";
  odom.pose.pose.position.x = x;
  odom.pose.pose.orientation.w = 1.0;
  return odom;
}

struct Line
{
  std::vector<std::array<double, 3>> points;
  std::string type;
};

lanelet::LaneletMapPtr make_map(const std::vector<Line> & lines)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Id id = 1;
  for (const auto & line : lines) {
    lanelet::Points3d pts;
    for (const auto & p : line.points) {
      pts.emplace_back(lanelet::Point3d(id++, p[0], p[1], p[2]));
    }
    lanelet::AttributeMap attributes;
    attributes[lanelet::AttributeName::Type] = line.type;
    lanelet::LineString3d ls(id++, pts, attributes);
    map->add(ls);
  }
  return map;
}

Line crossing_border(const double x, const double z = 0.0, const std::string & type = "road_border")
{
  return Line{{{x, -4.0, z}, {x, 4.0, z}}, type};
}

Line parallel_border(
  const double y, const double x_start, const double x_end,
  const std::string & type = "road_border")
{
  return Line{{{x_start, y, 0.0}, {x_end, y, 0.0}}, type};
}

std::optional<size_t> first_zero_velocity_index(const TrajectoryPoints & points)
{
  for (size_t i = 0; i < points.size(); ++i) {
    if (points.at(i).longitudinal_velocity_mps < 1e-3F) return i;
  }
  return std::nullopt;
}

MrmRoadBorderStopPlanner make_planner(const Params & params, const lanelet::LaneletMapPtr & map)
{
  MrmRoadBorderStopPlanner planner;
  planner.initialize(nullptr, make_vehicle_info(), params);
  planner.set_lanelet_map(map);
  return planner;
}

}  // namespace

TEST(MrmRoadBorderStopPlannerTest, StopInsertedBeforeCrossingBorder)
{
  const double border_x = 30.0;
  auto planner = make_planner(make_params(), make_map({crossing_border(border_x)}));
  auto points = make_straight_trajectory();

  const auto contact = planner.apply(points, make_ego_odometry());
  ASSERT_TRUE(contact.has_value());
  ASSERT_TRUE(contact->stop_pose.has_value());

  // footprint front touches the border when base_link is at border_x - front offset
  const double expected_contact_x = border_x - kFrontOffset;
  const double expected_stop_x = expected_contact_x - 1.0;
  const double contact_x = kTrajStartX + contact->contact_arc_length;
  EXPECT_NEAR(contact_x, expected_contact_x, 0.02);
  // the contact pose (used for the debug marker) is the refined base_link pose, not a point index
  EXPECT_NEAR(contact->contact_pose.position.x, expected_contact_x, 0.02);

  const auto stop_idx = first_zero_velocity_index(points);
  ASSERT_TRUE(stop_idx.has_value());
  EXPECT_NEAR(points.at(*stop_idx).pose.position.x, expected_stop_x, 0.05);
  // the stop pose is kept with the contact (the latched trajectory may be resampled later)
  EXPECT_DOUBLE_EQ(contact->stop_pose->position.x, points.at(*stop_idx).pose.position.x);
  // re-publishing while latched is a no-op without a node (publishers are disabled)
  EXPECT_NO_THROW(planner.publish_latched(points, make_ego_odometry()));
  // every point after the stop point is zero velocity
  for (size_t i = *stop_idx; i < points.size(); ++i) {
    EXPECT_FLOAT_EQ(points.at(i).longitudinal_velocity_mps, 0.0F);
  }
  // points before the stop keep their velocity
  EXPECT_FLOAT_EQ(
    points.at(*stop_idx - 1).longitudinal_velocity_mps, static_cast<float>(kVelocity));
}

TEST(MrmRoadBorderStopPlannerTest, NoStopWhenBorderOutsideFootprint)
{
  auto planner = make_planner(
    make_params(),
    make_map({parallel_border(1.75, -10.0, 100.0), parallel_border(-1.75, -10.0, 100.0)}));
  auto points = make_straight_trajectory();
  const auto original = points;

  const auto contact = planner.apply(points, make_ego_odometry());
  EXPECT_FALSE(contact.has_value());
  EXPECT_FALSE(first_zero_velocity_index(points).has_value());
  EXPECT_EQ(points.size(), original.size());
}

TEST(MrmRoadBorderStopPlannerTest, BoundaryConditionOfLateralContact)
{
  // border slightly outside the half width -> no contact
  {
    auto planner =
      make_planner(make_params(), make_map({parallel_border(kHalfWidth + 0.02, 20.0, 60.0)}));
    auto points = make_straight_trajectory();
    EXPECT_FALSE(planner.apply(points, make_ego_odometry()).has_value());
  }
  // border slightly inside the half width -> contact when the footprint front reaches x=20
  {
    auto planner =
      make_planner(make_params(), make_map({parallel_border(kHalfWidth - 0.02, 20.0, 60.0)}));
    auto points = make_straight_trajectory();
    const auto contact = planner.apply(points, make_ego_odometry());
    ASSERT_TRUE(contact.has_value());
    const double contact_x = kTrajStartX + contact->contact_arc_length;
    EXPECT_NEAR(contact_x, 20.0 - kFrontOffset, 0.02);
    const auto stop_idx = first_zero_velocity_index(points);
    ASSERT_TRUE(stop_idx.has_value());
    EXPECT_NEAR(points.at(*stop_idx).pose.position.x, 20.0 - kFrontOffset - 1.0, 0.05);
  }
}

TEST(MrmRoadBorderStopPlannerTest, IgnoresLinestringOfOtherTypeUnlessConfigured)
{
  const auto map = make_map({crossing_border(30.0, 0.0, "curbstone")});
  {
    auto planner = make_planner(make_params(), map);
    EXPECT_TRUE(planner.boundary_index().empty());
    auto points = make_straight_trajectory();
    EXPECT_FALSE(planner.apply(points, make_ego_odometry()).has_value());
  }
  {
    auto params = make_params();
    params.road_border_stop.boundary_types_to_detect = {"road_border", "curbstone"};
    auto planner = make_planner(params, map);
    EXPECT_EQ(planner.boundary_index().size(), 1U);
    auto points = make_straight_trajectory();
    EXPECT_TRUE(planner.apply(points, make_ego_odometry()).has_value());
  }
}

TEST(MrmRoadBorderStopPlannerTest, DisabledDoesNothing)
{
  auto params = make_params();
  params.road_border_stop.enable = false;
  auto planner = make_planner(params, make_map({crossing_border(30.0)}));
  auto points = make_straight_trajectory();
  EXPECT_FALSE(planner.apply(points, make_ego_odometry()).has_value());
  EXPECT_FALSE(first_zero_velocity_index(points).has_value());
}

TEST(MrmRoadBorderStopPlannerTest, ContactAtEgoStopsAtEgo)
{
  // border runs inside the footprint from behind the ego: contact at the ego nearest index
  auto planner = make_planner(make_params(), make_map({parallel_border(0.5, -10.0, 60.0)}));
  auto points = make_straight_trajectory();
  const auto contact = planner.apply(points, make_ego_odometry(0.0));
  ASSERT_TRUE(contact.has_value());
  // contact at (or slightly behind) the ego: no forward search happened
  EXPECT_LE(contact->contact_arc_length, contact->ego_arc_length + 1e-6);
  EXPECT_NEAR(contact->ego_arc_length, 0.0 - kTrajStartX, 1e-6);
  // stop point must not be behind the ego even though stop_margin > 0
  EXPECT_DOUBLE_EQ(contact->stop_arc_length, contact->ego_arc_length);
  const auto stop_idx = first_zero_velocity_index(points);
  ASSERT_TRUE(stop_idx.has_value());
  EXPECT_NEAR(points.at(*stop_idx).pose.position.x, 0.0, 0.01);
}

TEST(MrmRoadBorderStopPlannerTest, BorderBehindEgoFootprintIsIgnored)
{
  // Ego at x=0.25 lies inside the segment [0.0, 0.5]. A border crossing at x=-0.9 is behind the
  // ego rear (0.25 - 1.03 = -0.78) but inside the footprint of the segment start point at x=0.0
  // (rear -1.03). The search starts at the ego pose, so this border must not produce a stop.
  auto planner = make_planner(make_params(), make_map({crossing_border(-0.9)}));
  auto points = make_straight_trajectory();
  const auto contact = planner.apply(points, make_ego_odometry(0.25));
  EXPECT_FALSE(contact.has_value());
  EXPECT_FALSE(first_zero_velocity_index(points).has_value());
}

TEST(MrmRoadBorderStopPlannerTest, ContactAtEgoPoseBetweenTrajectoryPoints)
{
  // Border crossing just in front of the ego front (0.25 + 3.74 = 3.99): the ego footprint does not
  // touch it, the contact is found ahead and refined from the ego pose.
  const double border_x = 4.2;
  auto planner = make_planner(make_params(), make_map({crossing_border(border_x)}));
  auto points = make_straight_trajectory();
  const auto contact = planner.apply(points, make_ego_odometry(0.25));
  ASSERT_TRUE(contact.has_value());
  EXPECT_NEAR(kTrajStartX + contact->contact_arc_length, border_x - kFrontOffset, 0.02);
  EXPECT_GT(contact->contact_arc_length, contact->ego_arc_length);
  // stop point is clamped to the ego (contact is closer than stop_margin)
  EXPECT_DOUBLE_EQ(contact->stop_arc_length, contact->ego_arc_length);
}

TEST(MrmRoadBorderStopPlannerTest, HeightFilterIgnoresElevatedBorder)
{
  const auto map = make_map({crossing_border(30.0, 5.0)});
  {
    auto planner = make_planner(make_params(), map);
    auto points = make_straight_trajectory();
    EXPECT_FALSE(planner.apply(points, make_ego_odometry()).has_value());
  }
  {
    auto params = make_params();
    params.road_border_stop.use_height_filter = false;
    auto planner = make_planner(params, map);
    auto points = make_straight_trajectory();
    EXPECT_TRUE(planner.apply(points, make_ego_odometry()).has_value());
  }
}

TEST(MrmRoadBorderStopPlannerTest, StopMarginIsApplied)
{
  const auto map = make_map({crossing_border(30.0)});
  double stop_x_margin_1 = 0.0;
  double stop_x_margin_3 = 0.0;
  {
    auto planner = make_planner(make_params(), map);
    auto points = make_straight_trajectory();
    ASSERT_TRUE(planner.apply(points, make_ego_odometry()).has_value());
    stop_x_margin_1 = points.at(*first_zero_velocity_index(points)).pose.position.x;
  }
  {
    auto params = make_params();
    params.road_border_stop.stop_margin = 3.0;
    auto planner = make_planner(params, map);
    auto points = make_straight_trajectory();
    ASSERT_TRUE(planner.apply(points, make_ego_odometry()).has_value());
    stop_x_margin_3 = points.at(*first_zero_velocity_index(points)).pose.position.x;
  }
  EXPECT_NEAR(stop_x_margin_1 - stop_x_margin_3, 2.0, 0.05);
}

TEST(MrmRoadBorderStopPlannerTest, BeyondMaxCheckLengthIsIgnored)
{
  auto params = make_params();
  params.road_border_stop.max_check_length = 20.0;  // contact would be at ~26.3 m ahead
  auto planner = make_planner(params, make_map({crossing_border(30.0)}));
  auto points = make_straight_trajectory();
  EXPECT_FALSE(planner.apply(points, make_ego_odometry()).has_value());
}

TEST(MrmRoadBorderStopPlannerTest, LateralMarginExpandsFootprint)
{
  // border 0.3 m outside the half width: no contact without margin, contact with 0.5 m margin
  const auto map = make_map({parallel_border(kHalfWidth + 0.3, 20.0, 60.0)});
  {
    auto planner = make_planner(make_params(), map);
    auto points = make_straight_trajectory();
    EXPECT_FALSE(planner.apply(points, make_ego_odometry()).has_value());
  }
  {
    auto params = make_params();
    params.road_border_stop.lateral_margin = 0.5;
    auto planner = make_planner(params, map);
    auto points = make_straight_trajectory();
    EXPECT_TRUE(planner.apply(points, make_ego_odometry()).has_value());
  }
}

TEST(MrmRoadBorderStopPlannerTest, IndexIsRebuiltOnlyWhenMapChanges)
{
  const auto map_a = make_map({crossing_border(30.0)});
  const auto map_b = make_map({crossing_border(30.0), crossing_border(50.0)});
  MrmRoadBorderStopPlanner planner;
  planner.initialize(nullptr, make_vehicle_info(), make_params());
  planner.set_lanelet_map(map_a);
  EXPECT_EQ(planner.boundary_index().size(), 1U);
  planner.set_lanelet_map(map_a);
  EXPECT_EQ(planner.boundary_index().size(), 1U);
  planner.set_lanelet_map(map_b);
  EXPECT_EQ(planner.boundary_index().size(), 2U);
  planner.set_lanelet_map(nullptr);
  EXPECT_TRUE(planner.boundary_index().empty());
}

}  // namespace autoware::in_lane_mrm_planner
