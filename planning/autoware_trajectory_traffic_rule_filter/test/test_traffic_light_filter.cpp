// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/trajectory_traffic_rule_filter/filters/traffic_light_filter.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <csignal>
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
  void SetUp() override
  {
    filter_ = std::make_shared<TrafficLightFilter>();
    autoware::vehicle_info_utils::VehicleInfo vehicle_info;
    vehicle_info.max_longitudinal_offset_m = 0.0;
    filter_->set_vehicle_info(vehicle_info);
  }

  // Helper to create a simple straight lanelet map with a traffic light
  void create_and_set_map(lanelet::Id light_id, double stop_line_x)
  {
    // 1. Create Stop Line
    lanelet::Point3d sl1(lanelet::utils::getId(), stop_line_x, -5, 0);
    lanelet::Point3d sl2(lanelet::utils::getId(), stop_line_x, 5, 0);
    lanelet::LineString3d stop_line(lanelet::utils::getId(), {sl1, sl2});

    // 2. Create Traffic Light Shape (Dummy visual)
    lanelet::Point3d light_pt(lanelet::utils::getId(), stop_line_x + 5, 5, 5);
    lanelet::LineString3d light_shape(lanelet::utils::getId(), {light_pt});

    // 3. Create Regulatory Element
    auto traffic_light_re =
      lanelet::TrafficLight::make(light_id, lanelet::AttributeMap(), {light_shape}, stop_line);

    // 4. Create Lanelet Boundaries
    lanelet::Point3d l1(lanelet::utils::getId(), 0, -5, 0);
    lanelet::Point3d l2(lanelet::utils::getId(), 20, -5, 0);
    lanelet::Point3d r1(lanelet::utils::getId(), 0, 5, 0);
    lanelet::Point3d r2(lanelet::utils::getId(), 20, 5, 0);

    lanelet::LineString3d left(lanelet::utils::getId(), {l1, l2});
    lanelet::LineString3d right(lanelet::utils::getId(), {r1, r2});

    // 5. Create Lanelet and add RE
    lanelet::Lanelet lanelet(lanelet::utils::getId(), left, right);
    lanelet.addRegulatoryElement(traffic_light_re);

    // 6. Create and Set Map
    std::shared_ptr<lanelet::LaneletMap> map = lanelet::utils::createMap({lanelet});
    filter_->set_lanelet_map(map, nullptr, nullptr);
  }

  // Helper to set traffic light signal
  void set_traffic_light_signal(lanelet::Id id, uint8_t color)
  {
    auto signals = std::make_shared<TrafficLightGroupArray>();
    TrafficLightGroup group;
    group.traffic_light_group_id = id;

    TrafficLightElement element;
    element.color = color;
    element.shape = TrafficLightElement::CIRCLE;
    element.status = TrafficLightElement::SOLID_ON;
    element.confidence = 1.0;

    group.elements.push_back(element);
    signals->traffic_light_groups.push_back(group);

    filter_->set_traffic_lights(signals);
  }

  // Helper to create a straight trajectory
  static std::vector<TrajectoryPoint> create_trajectory(double start_x, double end_x)
  {
    std::vector<TrajectoryPoint> points;
    TrajectoryPoint tp1;
    tp1.pose.position.x = start_x;
    tp1.pose.position.y = 0;

    TrajectoryPoint tp2;
    tp2.pose.position.x = end_x;
    tp2.pose.position.y = 0;

    points.push_back(tp1);
    points.push_back(tp2);
    return points;
  }

  std::shared_ptr<TrafficLightFilter> filter_;
};

TEST_F(TrafficLightFilterTest, IsFeasibleEmptyInput)
{
  std::vector<TrajectoryPoint> points;
  EXPECT_TRUE(filter_->is_feasible(points));
}

TEST_F(TrafficLightFilterTest, IsFeasibleNoMap)
{
  auto points = create_trajectory(0.0, 1.0);
  EXPECT_TRUE(filter_->is_feasible(points));
}

TEST_F(TrafficLightFilterTest, IsInfeasibleWithRedLightIntersection)
{
  const lanelet::Id light_id = 100;
  const double stop_x = 5.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::RED);

  // Trajectory crossing stop line (0 -> 10)
  auto points = create_trajectory(0.0, 10.0);

  EXPECT_FALSE(filter_->is_feasible(points))
    << "Should return false when crossing red light stop line";
}

TEST_F(TrafficLightFilterTest, IsFeasibleWithGreenLight)
{
  const lanelet::Id light_id = 101;
  const double stop_x = 5.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::GREEN);

  // Trajectory crossing stop line (0 -> 10)
  auto points = create_trajectory(0.0, 10.0);

  EXPECT_TRUE(filter_->is_feasible(points)) << "Should return true for green light";
}

TEST_F(TrafficLightFilterTest, IsFeasibleWithRedLightNoIntersection)
{
  const lanelet::Id light_id = 102;
  const double stop_x = 5.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::RED);

  // Trajectory stops before stop line (0 -> 4)
  auto points = create_trajectory(0.0, 4.0);

  EXPECT_TRUE(filter_->is_feasible(points)) << "Should return true if red light is not crossed";
}

TEST_F(TrafficLightFilterTest, IsInfeasibleWithFrontOverhang)
{
  const lanelet::Id light_id = 103;
  const double stop_x = 5.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::RED);

  // Trajectory stopping ahead of stop line (0 -> 4.0)
  auto points = create_trajectory(0.0, 4.0);
  // Front overhang going over the stop line
  autoware::vehicle_info_utils::VehicleInfo vehicle_info;
  vehicle_info.max_longitudinal_offset_m = 2.0;
  filter_->set_vehicle_info(vehicle_info);

  EXPECT_FALSE(filter_->is_feasible(points))
    << "Should return false when crossing red light stop line";
}
