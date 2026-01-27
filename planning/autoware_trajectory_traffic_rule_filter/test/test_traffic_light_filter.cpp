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

#include "autoware/trajectory_traffic_rule_filter/filters/traffic_light_filter.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <vector>

using autoware::trajectory_traffic_rule_filter::plugin::TrafficLightFilter;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_planning_msgs::msg::TrajectoryPoint;

class TrafficLightFilterTest : public ::testing::Test
{
protected:
  void SetUp() override { filter_ = std::make_shared<TrafficLightFilter>(); }

  std::shared_ptr<TrafficLightFilter> filter_;
};

TEST_F(TrafficLightFilterTest, IsFeasibleEmptyInput)
{
  std::vector<TrajectoryPoint> points;
  EXPECT_TRUE(filter_->is_feasible(points));
}

TEST_F(TrafficLightFilterTest, IsFeasibleNoMap)
{
  TrajectoryPoint p;
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  std::vector<TrajectoryPoint> points = {p};

  EXPECT_TRUE(filter_->is_feasible(points));
}

TEST_F(TrafficLightFilterTest, IsFeasibleWithRedLightIntersection)
{
  // 1. Create Lanelet Map with Traffic Light
  lanelet::Point3d p1(1, 0, 0, 0);
  lanelet::Point3d p2(2, 0, 0, 0);  // Stop line points
  lanelet::LineString3d stop_line(lanelet::utils::getId(), {p1, p2});

  // Traffic Light Regulatory Element
  lanelet::LineString3d light_shape(
    lanelet::utils::getId(),
    {lanelet::Point3d(lanelet::utils::getId(), 10, 10, 5)});  // Dummy shape
  light_shape.setId(100);                                     // Set ID to match signal

  auto traffic_light_re = lanelet::TrafficLight::make(
    lanelet::utils::getId(), lanelet::AttributeMap(), {light_shape}, stop_line);

  lanelet::Point3d l1(3, -5, -5, 0);
  lanelet::Point3d l2(4, 5, -5, 0);
  lanelet::Point3d r1(5, -5, 5, 0);
  lanelet::Point3d r2(6, 5, 5, 0);
  lanelet::LineString3d left(lanelet::utils::getId(), {l1, l2});
  lanelet::LineString3d right(lanelet::utils::getId(), {r1, r2});

  lanelet::Lanelet lanelet(lanelet::utils::getId(), left, right);
  lanelet.addRegulatoryElement(traffic_light_re);

  std::shared_ptr<lanelet::LaneletMap> map = lanelet::utils::createMap({lanelet});

  // 2. Set Map
  filter_->set_lanelet_map(map, nullptr, nullptr);

  // 3. Create Traffic Light Signal (Red)
  auto signals = std::make_shared<TrafficLightGroupArray>();
  TrafficLightGroup group;
  group.traffic_light_group_id = 100;
  TrafficLightElement element;
  element.color = TrafficLightElement::RED;
  element.shape = TrafficLightElement::CIRCLE;
  element.status = TrafficLightElement::SOLID_ON;
  element.confidence = 1.0;
  group.elements.push_back(element);
  signals->traffic_light_groups.push_back(group);

  filter_->set_traffic_lights(signals);

  // 4. Create Trajectory intersecting the stop line
  // Stop line: x=5, y from -5 to 5.
  lanelet::Point3d sl1(lanelet::utils::getId(), 5, -5, 0);
  lanelet::Point3d sl2(lanelet::utils::getId(), 5, 5, 0);
  lanelet::LineString3d stop_line_geom(lanelet::utils::getId(), {sl1, sl2});

  // Re-create RE with proper stop line
  auto traffic_light_re_2 = lanelet::TrafficLight::make(
    lanelet::utils::getId(), lanelet::AttributeMap(), {light_shape}, stop_line_geom);
  // Re-use light_shape with ID 100, but let's change signal ID to 101 for second part if needed,
  // but simpler to just use 101 for everything in second part.
  light_shape.setId(101);

  // Re-make because modifying ID of shared primitive might be tricky if used elsewhere,
  // but here it's fine.
  traffic_light_re_2 = lanelet::TrafficLight::make(
    lanelet::utils::getId(), lanelet::AttributeMap(), {light_shape}, stop_line_geom);

  lanelet::Lanelet lanelet_2(lanelet::utils::getId(), left, right);
  lanelet_2.addRegulatoryElement(traffic_light_re_2);
  std::shared_ptr<lanelet::LaneletMap> map_2 = lanelet::utils::createMap({lanelet_2});

  filter_->set_lanelet_map(map_2, nullptr, nullptr);

  // Update signal ID
  signals->traffic_light_groups[0].traffic_light_group_id = 101;
  filter_->set_traffic_lights(signals);

  // Trajectory crossing x=5
  std::vector<TrajectoryPoint> points;
  TrajectoryPoint tp1;
  tp1.pose.position.x = 0;
  tp1.pose.position.y = 0;
  TrajectoryPoint tp2;
  tp2.pose.position.x = 10;
  tp2.pose.position.y = 0;
  points.push_back(tp1);
  points.push_back(tp2);

  EXPECT_FALSE(filter_->is_feasible(points))
    << "Should return false when crossing red light stop line";
}

TEST_F(TrafficLightFilterTest, IsFeasibleWithGreenLight)
{
  // Setup similar to Red Light test but with Green signal
  // Stop line: x=5, y from -5 to 5.
  lanelet::Point3d sl1(lanelet::utils::getId(), 5, -5, 0);
  lanelet::Point3d sl2(lanelet::utils::getId(), 5, 5, 0);
  lanelet::LineString3d stop_line_geom(lanelet::utils::getId(), {sl1, sl2});

  lanelet::LineString3d light_shape(
    lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 10, 10, 5)});
  light_shape.setId(102);

  auto traffic_light_re = lanelet::TrafficLight::make(
    lanelet::utils::getId(), lanelet::AttributeMap(), {light_shape}, stop_line_geom);

  lanelet::Point3d l1(lanelet::utils::getId(), 0, -5, 0);
  lanelet::Point3d l2(lanelet::utils::getId(), 10, -5, 0);
  lanelet::Point3d r1(lanelet::utils::getId(), 0, 5, 0);
  lanelet::Point3d r2(lanelet::utils::getId(), 10, 5, 0);
  lanelet::LineString3d left(lanelet::utils::getId(), {l1, l2});
  lanelet::LineString3d right(lanelet::utils::getId(), {r1, r2});

  lanelet::Lanelet lanelet(lanelet::utils::getId(), left, right);
  lanelet.addRegulatoryElement(traffic_light_re);

  std::shared_ptr<lanelet::LaneletMap> map = lanelet::utils::createMap({lanelet});

  filter_->set_lanelet_map(map, nullptr, nullptr);

  auto signals = std::make_shared<TrafficLightGroupArray>();
  TrafficLightGroup group;
  group.traffic_light_group_id = 102;
  TrafficLightElement element;
  element.color = TrafficLightElement::GREEN;  // GREEN
  element.shape = TrafficLightElement::CIRCLE;
  element.status = TrafficLightElement::SOLID_ON;
  group.elements.push_back(element);
  signals->traffic_light_groups.push_back(group);

  filter_->set_traffic_lights(signals);

  // Trajectory crossing x=5
  std::vector<TrajectoryPoint> points;
  TrajectoryPoint tp1;
  tp1.pose.position.x = 0;
  tp1.pose.position.y = 0;
  TrajectoryPoint tp2;
  tp2.pose.position.x = 10;
  tp2.pose.position.y = 0;
  points.push_back(tp1);
  points.push_back(tp2);

  EXPECT_TRUE(filter_->is_feasible(points)) << "Should return true for green light";
}

TEST_F(TrafficLightFilterTest, IsFeasibleWithRedLightNoIntersection)
{
  // Setup Red Light but trajectory does not cross it
  lanelet::Point3d sl1(lanelet::utils::getId(), 5, -5, 0);
  lanelet::Point3d sl2(lanelet::utils::getId(), 5, 5, 0);
  lanelet::LineString3d stop_line_geom(lanelet::utils::getId(), {sl1, sl2});

  lanelet::LineString3d light_shape(
    lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 10, 10, 5)});
  light_shape.setId(103);

  auto traffic_light_re = lanelet::TrafficLight::make(
    lanelet::utils::getId(), lanelet::AttributeMap(), {light_shape}, stop_line_geom);

  lanelet::Point3d l1(lanelet::utils::getId(), 0, -5, 0);
  lanelet::Point3d l2(lanelet::utils::getId(), 10, -5, 0);
  lanelet::Point3d r1(lanelet::utils::getId(), 0, 5, 0);
  lanelet::Point3d r2(lanelet::utils::getId(), 10, 5, 0);
  lanelet::LineString3d left(lanelet::utils::getId(), {l1, l2});
  lanelet::LineString3d right(lanelet::utils::getId(), {r1, r2});

  lanelet::Lanelet lanelet(lanelet::utils::getId(), left, right);
  lanelet.addRegulatoryElement(traffic_light_re);

  std::shared_ptr<lanelet::LaneletMap> map = lanelet::utils::createMap({lanelet});

  filter_->set_lanelet_map(map, nullptr, nullptr);

  auto signals = std::make_shared<TrafficLightGroupArray>();
  TrafficLightGroup group;
  group.traffic_light_group_id = 103;
  TrafficLightElement element;
  element.color = TrafficLightElement::RED;
  element.shape = TrafficLightElement::CIRCLE;
  element.status = TrafficLightElement::SOLID_ON;
  group.elements.push_back(element);
  signals->traffic_light_groups.push_back(group);

  filter_->set_traffic_lights(signals);

  // Trajectory NOT crossing x=5 (stops before)
  std::vector<TrajectoryPoint> points;
  TrajectoryPoint tp1;
  tp1.pose.position.x = 0;
  tp1.pose.position.y = 0;
  TrajectoryPoint tp2;
  tp2.pose.position.x = 4;
  tp2.pose.position.y = 0;
  points.push_back(tp1);
  points.push_back(tp2);

  EXPECT_TRUE(filter_->is_feasible(points)) << "Should return true if red light is not crossed";
}
