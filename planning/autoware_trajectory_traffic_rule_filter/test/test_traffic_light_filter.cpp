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

    // Initialize parameters
    traffic_rule_filter::Params params;
    params.traffic_light_filter.deceleration_limit = 2.8;
    params.traffic_light_filter.jerk_limit = 5.0;
    params.traffic_light_filter.delay_response_time = 0.5;
    params.traffic_light_filter.crossing_time_limit = 2.75;
    params.traffic_light_filter.treat_amber_light_as_red_light = false;
    filter_->set_parameters(params);
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
    lanelet::Point3d l2(lanelet::utils::getId(), 100, -5, 0);
    lanelet::Point3d r1(lanelet::utils::getId(), 0, 5, 0);
    lanelet::Point3d r2(lanelet::utils::getId(), 100, 5, 0);

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
  static std::vector<TrajectoryPoint> create_trajectory(
    double start_x, double end_x, float velocity = 5.0)
  {
    std::vector<TrajectoryPoint> points;
    TrajectoryPoint tp1;
    tp1.pose.position.x = start_x;
    tp1.pose.position.y = 0;
    tp1.longitudinal_velocity_mps = velocity;

    TrajectoryPoint tp2;
    tp2.pose.position.x = end_x;
    tp2.pose.position.y = 0;
    tp2.longitudinal_velocity_mps = velocity;

    points.push_back(tp1);
    points.push_back(tp2);
    return points;
  }

  std::shared_ptr<TrafficLightFilter> filter_;
};

TEST_F(TrafficLightFilterTest, IsFeasibleEmptyInput)
{
  std::vector<TrajectoryPoint> points;
  EXPECT_FALSE(filter_->is_feasible(points)) << "Should not be feasible without a map";
}

TEST_F(TrafficLightFilterTest, IsFeasibleNoMap)
{
  auto points = create_trajectory(0.0, 1.0);
  EXPECT_FALSE(filter_->is_feasible(points)) << "Should not be feasible without a map";
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

TEST_F(TrafficLightFilterTest, IsInfeasibleWithAmberLightCanStop)
{
  const lanelet::Id light_id = 200;
  const double stop_x = 20.0;  // Stop line at 20m

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::AMBER);

  // Ego at 0m, velocity 5m/s.
  // stop_x is 20m away.
  // Stoppable distance is roughly 5^2 / (2 * 2.8) + 5 * 0.5 = 6.96m.
  // Since 6.96 < 20.0, it IS stoppable.
  // can_pass_amber_light should return false (ego MUST stop if it can).

  auto points = create_trajectory(0.0, 30.0, 5.0);

  EXPECT_FALSE(filter_->is_feasible(points)) << "Should return false if amber light can be stopped";
}

TEST_F(TrafficLightFilterTest, IsFeasibleWithAmberLightCannotStop)
{
  const lanelet::Id light_id = 201;
  const double stop_x = 5.0;  // Stop line at 5m

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::AMBER);

  // Ego at 0m, velocity 10m/s.
  // stop_x is 5m away.
  // Stoppable distance is roughly 10^2 / (2 * 2.8) + 10 * 0.5 = 22.85m.
  // Since 22.85 > 5.0, it is NOT stoppable.

  // Reachable distance: v * crossing_time_limit = 10 * 2.75 = 27.5m.
  // Since 5.0 < 27.5, it IS reachable.
  // can_pass_amber_light should return true (ego CANNOT stop and CAN pass).

  auto points = create_trajectory(0.0, 10.0, 10.0);

  EXPECT_TRUE(filter_->is_feasible(points))
    << "Should return true if amber light cannot be stopped but is reachable";
}

TEST_F(TrafficLightFilterTest, IsInfeasibleWithAmberLightCanStopAndCannotPass)
{
  const lanelet::Id light_id = 202;
  const double stop_x = 150.0;  // Stop line at 150m

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::AMBER);

  // Ego at 0m, velocity 10m/s.
  // stop_x is 150m away.
  // Stoppable distance is 110m. 110 < 150, so it IS stoppable.
  // Reachable distance = v * T_amber. 10 * 1.0 = 10m.
  // stop_x > 10 -> NOT reachable.
  // This is a scenario where ego can stop and cannot pass.

  // Let's adjust params to create the desired scenario.
  traffic_rule_filter::Params params;
  params.traffic_light_filter.deceleration_limit = -0.5;  // Very weak braking
  params.traffic_light_filter.delay_response_time = 1.0;
  params.traffic_light_filter.crossing_time_limit = 1.0;  // Short amber
  params.traffic_light_filter.treat_amber_light_as_red_light = false;
  filter_->set_parameters(params);

  auto points = create_trajectory(0.0, 200.0, 10.0);

  EXPECT_FALSE(filter_->is_feasible(points))
    << "Should return false if ego can stop but cannot pass";
}

TEST_F(TrafficLightFilterTest, IsInfeasibleWithAmberLightAsRedLight)
{
  const lanelet::Id light_id = 300;
  const double stop_x = 5.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::AMBER);

  traffic_rule_filter::Params params;
  params.traffic_light_filter.deceleration_limit = 2.8;
  params.traffic_light_filter.delay_response_time = 0.5;
  params.traffic_light_filter.crossing_time_limit = 2.75;
  params.traffic_light_filter.treat_amber_light_as_red_light = true;
  filter_->set_parameters(params);

  // Even if it's NOT stoppable (ego at 0m, velocity 10m/s, stop at 5m),
  // it should be rejected because it's treated as red.
  auto points = create_trajectory(0.0, 10.0, 10.0);

  EXPECT_FALSE(filter_->is_feasible(points))
    << "Should return false for amber light when treat_amber_light_as_red_light is true";
}
