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

#include "autoware/trajectory_validator/filters/traffic_rule/traffic_light_filter.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using autoware::trajectory_validator::FilterContext;
using autoware::trajectory_validator::plugin::traffic_rule::TrafficLightFilter;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_planning_msgs::msg::TrajectoryPoint;

class TrafficLightFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_traffic_light_filter_node");
    filter_ = std::make_shared<TrafficLightFilter>();
    autoware::vehicle_info_utils::VehicleInfo vehicle_info;
    vehicle_info.max_longitudinal_offset_m = 0.0;
    filter_->set_vehicle_info(vehicle_info);

    // Initialize default parameters
    node_->declare_parameter("traffic_light.deceleration_limit", 2.8);
    node_->declare_parameter("traffic_light.jerk_limit", 5.0);
    node_->declare_parameter("traffic_light.delay_response_time", 0.5);
    node_->declare_parameter("traffic_light.crossing_time_limit", 2.75);
    node_->declare_parameter("traffic_light.treat_amber_light_as_red_light", false);
    node_->declare_parameter("traffic_light.checked_trajectory_length.deceleration_limit", 999.9);
    node_->declare_parameter("traffic_light.checked_trajectory_length.jerk_limit", 999.9);

    validator::Params params;
    params.traffic_light.deceleration_limit = 2.8;
    params.traffic_light.jerk_limit = 5.0;
    params.traffic_light.delay_response_time = 0.5;
    params.traffic_light.crossing_time_limit = 2.75;
    params.traffic_light.treat_amber_light_as_red_light = false;
    params.traffic_light.checked_trajectory_length.deceleration_limit = 999.9;
    params.traffic_light.checked_trajectory_length.jerk_limit = 999.9;
    filter_->update_parameters(params);

    context_.traffic_light_signals = std::make_shared<TrafficLightGroupArray>();
    // Set high velocity/acceleration to always check the whole trajectory
    // (checked_trajectory_length feature)
    nav_msgs::msg::Odometry odometry;
    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    odometry.twist.twist.linear.x = 999.9f;
    accel.accel.accel.linear.x = 999.9f;
    context_.odometry = std::make_shared<nav_msgs::msg::Odometry>(odometry);
    context_.acceleration = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>(accel);
  }

  // Helper to create a simple straight lanelet map with a traffic light
  void create_and_set_map(
    lanelet::Id light_id, double stop_line_x, const std::string & turn_direction = "else")
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
    lanelet::Point3d l2(lanelet::utils::getId(), 200, -5, 0);
    lanelet::Point3d r1(lanelet::utils::getId(), 0, 5, 0);
    lanelet::Point3d r2(lanelet::utils::getId(), 200, 5, 0);

    lanelet::LineString3d left(lanelet::utils::getId(), {l1, l2});
    lanelet::LineString3d right(lanelet::utils::getId(), {r1, r2});

    // 5. Create Lanelet and add RE
    lanelet::Lanelet lanelet(lanelet::utils::getId(), left, right);
    lanelet.attributes()["turn_direction"] = turn_direction;
    lanelet.addRegulatoryElement(traffic_light_re);

    // 6. Create and Set Map
    context_.lanelet_map = lanelet::utils::createMap({lanelet});
  }

  // Helper to set traffic light signal
  void set_traffic_light_signal(
    lanelet::Id id, uint8_t color, uint8_t shape = TrafficLightElement::CIRCLE)
  {
    set_traffic_light_signals(id, {{color, shape}});
  }

  // Helper to set multiple traffic light signal elements
  void set_traffic_light_signals(
    lanelet::Id id, const std::vector<std::pair<uint8_t, uint8_t>> & elements_data)
  {
    auto signals = std::make_shared<TrafficLightGroupArray>();
    TrafficLightGroup group;
    group.traffic_light_group_id = id;

    for (const auto & data : elements_data) {
      TrafficLightElement element;
      element.color = data.first;
      element.shape = data.second;
      element.status = TrafficLightElement::SOLID_ON;
      element.confidence = 1.0;
      group.elements.push_back(element);
    }
    signals->traffic_light_groups.push_back(group);

    context_.traffic_light_signals = signals;
  }

  // Helper to create an intersection map with multiple lanelets
  void create_intersection_map(lanelet::Id light_id, double stop_x)
  {
    // Common start
    lanelet::Point3d p_start_l(lanelet::utils::getId(), 0, 2.5, 0);
    lanelet::Point3d p_start_r(lanelet::utils::getId(), 0, -2.5, 0);
    lanelet::Point3d p_stop_l(lanelet::utils::getId(), stop_x, 2.5, 0);
    lanelet::Point3d p_stop_r(lanelet::utils::getId(), stop_x, -2.5, 0);

    lanelet::LineString3d stop_line(lanelet::utils::getId(), {p_stop_l, p_stop_r});
    lanelet::LineString3d light_shape(lanelet::utils::getId(), {p_stop_l});  // dummy
    auto traffic_light_re =
      lanelet::TrafficLight::make(light_id, lanelet::AttributeMap(), {light_shape}, stop_line);

    // Straight
    lanelet::Point3d p_straight_l(lanelet::utils::getId(), stop_x + 10, 2.5, 0);
    lanelet::Point3d p_straight_r(lanelet::utils::getId(), stop_x + 10, -2.5, 0);
    lanelet::Lanelet straight_ll(
      lanelet::utils::getId(),
      lanelet::LineString3d(lanelet::utils::getId(), {p_start_l, p_stop_l, p_straight_l}),
      lanelet::LineString3d(lanelet::utils::getId(), {p_start_r, p_stop_r, p_straight_r}));
    straight_ll.attributes()["turn_direction"] = "straight";
    straight_ll.addRegulatoryElement(traffic_light_re);

    // Left (45 deg turn)
    lanelet::Point3d p_left_l(lanelet::utils::getId(), stop_x + 5, 10.0, 0);
    lanelet::Point3d p_left_r(lanelet::utils::getId(), stop_x + 10, 5.0, 0);
    lanelet::Lanelet left_ll(
      lanelet::utils::getId(),
      lanelet::LineString3d(lanelet::utils::getId(), {p_start_l, p_stop_l, p_left_l}),
      lanelet::LineString3d(lanelet::utils::getId(), {p_start_r, p_stop_r, p_left_r}));
    left_ll.attributes()["turn_direction"] = "left";
    left_ll.addRegulatoryElement(traffic_light_re);

    // Right (45 deg turn)
    lanelet::Point3d p_right_l(lanelet::utils::getId(), stop_x + 10, -5.0, 0);
    lanelet::Point3d p_right_r(lanelet::utils::getId(), stop_x + 5, -10.0, 0);
    lanelet::Lanelet right_ll(
      lanelet::utils::getId(),
      lanelet::LineString3d(lanelet::utils::getId(), {p_start_l, p_stop_l, p_right_l}),
      lanelet::LineString3d(lanelet::utils::getId(), {p_start_r, p_stop_r, p_right_r}));
    right_ll.attributes()["turn_direction"] = "right";
    right_ll.addRegulatoryElement(traffic_light_re);

    context_.lanelet_map = lanelet::utils::createMap({straight_ll, left_ll, right_ll});
  }

  // Helper to create a straight trajectory
  static std::vector<TrajectoryPoint> create_trajectory(
    double start_x, double end_x, float velocity = 5.0, double y = 0.0)
  {
    std::vector<TrajectoryPoint> points;
    TrajectoryPoint tp1;
    tp1.pose.position.x = start_x;
    tp1.pose.position.y = y;
    tp1.longitudinal_velocity_mps = velocity;
    tp1.time_from_start = rclcpp::Duration::from_seconds(0.0);

    TrajectoryPoint tp2;
    tp2.pose.position.x = end_x;
    tp2.pose.position.y = y;
    tp2.longitudinal_velocity_mps = velocity;
    tp2.time_from_start = rclcpp::Duration::from_seconds(
      std::abs(end_x - start_x) / std::max(0.1f, std::abs(velocity)));

    points.push_back(tp1);
    points.push_back(tp2);
    return points;
  }

  // Helper to create a curved trajectory
  static std::vector<TrajectoryPoint> create_curved_trajectory(
    const std::vector<std::pair<double, double>> & waypoints, float velocity = 5.0)
  {
    std::vector<TrajectoryPoint> points;
    double total_dist = 0.0;
    for (size_t i = 0; i < waypoints.size(); ++i) {
      if (i > 0) {
        total_dist += std::hypot(
          waypoints[i].first - waypoints[i - 1].first,
          waypoints[i].second - waypoints[i - 1].second);
      }
      TrajectoryPoint tp;
      tp.pose.position.x = waypoints[i].first;
      tp.pose.position.y = waypoints[i].second;
      tp.longitudinal_velocity_mps = velocity;
      tp.time_from_start = rclcpp::Duration::from_seconds(total_dist / std::max(0.1f, velocity));
      points.push_back(tp);
    }
    return points;
  }

  std::shared_ptr<TrafficLightFilter> filter_;
  std::shared_ptr<rclcpp::Node> node_;
  FilterContext context_;
};

TEST_F(TrafficLightFilterTest, IsFeasibleEmptyInput)
{
  std::vector<TrajectoryPoint> points;
  create_and_set_map(0, 0);
  set_traffic_light_signal(0, TrafficLightElement::RED);
  EXPECT_TRUE(filter_->is_feasible(points, context_))
    << "Empty trajectory should always be feasible (cannot cross a traffic light if empty)";
}

TEST_F(TrafficLightFilterTest, IsInfeasibleWithoutMapAndSignals)
{
  const auto points = create_trajectory(0.0, 1.0);
  context_.lanelet_map = nullptr;
  context_.traffic_light_signals = nullptr;
  EXPECT_FALSE(filter_->is_feasible(points, context_))
    << "Should not be feasible without a map or traffic light signals";
}
TEST_F(TrafficLightFilterTest, IsInfeasibleWithoutMap)
{
  auto points = create_trajectory(0.0, 1.0);
  // dummy map and light signal
  context_.lanelet_map = nullptr;
  set_traffic_light_signal(0, TrafficLightElement::RED);
  EXPECT_FALSE(filter_->is_feasible(points, context_))
    << "Should not be feasible without a map (cannot verify whether a trajectory crosses a traffic "
       "light)";
}
TEST_F(TrafficLightFilterTest, IsInfeasibleWithoutSignals)
{
  auto points = create_trajectory(0.0, 1.0);
  create_and_set_map(0, 0);
  context_.traffic_light_signals = nullptr;
  EXPECT_FALSE(filter_->is_feasible(points, context_))
    << "Should not be feasible without traffic light signals (cannot verify whether a trajectory "
       "crosses a traffic "
       "light)";
}

TEST_F(TrafficLightFilterTest, IsInfeasibleWithRedLightIntersection)
{
  const lanelet::Id light_id = 100;
  const double stop_x = 5.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::RED);

  // Trajectory crossing stop line (0 -> 10)
  auto points = create_trajectory(0.0, 10.0);

  EXPECT_FALSE(filter_->is_feasible(points, context_))
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

  EXPECT_TRUE(filter_->is_feasible(points, context_)) << "Should return true for green light";
}

TEST_F(TrafficLightFilterTest, IsFeasibleWithRedLightNoIntersection)
{
  const lanelet::Id light_id = 102;
  const double stop_x = 5.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::RED);

  // Trajectory stops before stop line (0 -> 4)
  auto points = create_trajectory(0.0, 4.0);

  EXPECT_TRUE(filter_->is_feasible(points, context_))
    << "Should return true if red light is not crossed";
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

  EXPECT_FALSE(filter_->is_feasible(points, context_))
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

  EXPECT_FALSE(filter_->is_feasible(points, context_))
    << "Should return false if amber light can be stopped";
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

  EXPECT_TRUE(filter_->is_feasible(points, context_))
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
  validator::Params params;
  params.traffic_light.deceleration_limit = -0.5;  // Very weak braking
  params.traffic_light.delay_response_time = 1.0;
  params.traffic_light.crossing_time_limit = 1.0;  // Short amber
  params.traffic_light.treat_amber_light_as_red_light = false;
  filter_->update_parameters(params);

  auto points = create_trajectory(0.0, 200.0, 10.0);

  EXPECT_FALSE(filter_->is_feasible(points, context_))
    << "Should return false if ego can stop but cannot pass";
}

TEST_F(TrafficLightFilterTest, IsInfeasibleWithAmberLightAsRedLight)
{
  const lanelet::Id light_id = 300;
  const double stop_x = 5.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::AMBER);

  validator::Params params;
  params.traffic_light.deceleration_limit = 2.8;
  params.traffic_light.delay_response_time = 0.5;
  params.traffic_light.crossing_time_limit = 2.75;
  params.traffic_light.treat_amber_light_as_red_light = true;
  filter_->update_parameters(params);

  // Even if it's NOT stoppable (ego at 0m, velocity 10m/s, stop at 5m),
  // it should be rejected because it's treated as red.
  auto points = create_trajectory(0.0, 10.0, 10.0);

  EXPECT_FALSE(filter_->is_feasible(points, context_))
    << "Should return false for amber light when treat_amber_light_as_red_light is true";
}

TEST_F(TrafficLightFilterTest, IsFeasibleWithRedCircleAndMatchingGreenRightArrow)
{
  const lanelet::Id light_id = 400;
  const double stop_x = 5.0;

  create_and_set_map(light_id, stop_x, "right");
  set_traffic_light_signals(
    light_id, {{TrafficLightElement::RED, TrafficLightElement::CIRCLE},
               {TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW}});

  auto points = create_trajectory(0.0, 10.0);
  EXPECT_TRUE(filter_->is_feasible(points, context_))
    << "Should return true for red circle + green right arrow when turning right";
}

TEST_F(TrafficLightFilterTest, IsFeasibleWithRedCircleAndMatchingGreenLeftArrow)
{
  const lanelet::Id light_id = 401;
  const double stop_x = 5.0;

  create_and_set_map(light_id, stop_x, "left");
  set_traffic_light_signals(
    light_id, {{TrafficLightElement::RED, TrafficLightElement::CIRCLE},
               {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW}});

  auto points = create_trajectory(0.0, 10.0);
  EXPECT_TRUE(filter_->is_feasible(points, context_))
    << "Should return true for red circle + green left arrow when turning left";
}

TEST_F(TrafficLightFilterTest, IsFeasibleWithRedCircleAndMatchingGreenUpArrow)
{
  const lanelet::Id light_id = 402;
  const double stop_x = 5.0;

  create_and_set_map(light_id, stop_x, "straight");
  set_traffic_light_signals(
    light_id, {{TrafficLightElement::RED, TrafficLightElement::CIRCLE},
               {TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW}});

  auto points = create_trajectory(0.0, 10.0);
  EXPECT_TRUE(filter_->is_feasible(points, context_))
    << "Should return true for red circle + green up arrow when going straight";
}

TEST_F(TrafficLightFilterTest, IsInfeasibleWithRedCircleAndMismatchingGreenArrow)
{
  const lanelet::Id light_id = 403;
  const double stop_x = 5.0;

  create_and_set_map(light_id, stop_x, "right");
  // Red circle + green LEFT arrow, but lanelet is RIGHT turn.
  set_traffic_light_signals(
    light_id, {{TrafficLightElement::RED, TrafficLightElement::CIRCLE},
               {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW}});

  auto points = create_trajectory(0.0, 10.0);
  EXPECT_FALSE(filter_->is_feasible(points, context_))
    << "Should return false for mismatching green arrow";
}

TEST_F(TrafficLightFilterTest, IsInfeasibleWithRedCircleAndGreenArrowNoTurnDirection)
{
  const lanelet::Id light_id = 404;
  const double stop_x = 5.0;

  create_and_set_map(light_id, stop_x, "else");
  set_traffic_light_signals(
    light_id, {{TrafficLightElement::RED, TrafficLightElement::CIRCLE},
               {TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW}});

  auto points = create_trajectory(0.0, 10.0);
  EXPECT_FALSE(filter_->is_feasible(points, context_))
    << "Should return false for green arrow if turn_direction is else/unknown";
}

TEST_F(TrafficLightFilterTest, IsFeasibleComplexIntersection)
{
  const lanelet::Id light_id = 500;
  const double stop_x = 5.0;
  create_intersection_map(light_id, stop_x);

  auto traj_straight = create_curved_trajectory({{0, 0}, {10, 0}});
  auto traj_left = create_curved_trajectory({{0, 0}, {5, 0}, {10, 5}});
  auto traj_right = create_curved_trajectory({{0, 0}, {5, 0}, {10, -5}});

  // 1. Only RED
  set_traffic_light_signal(light_id, TrafficLightElement::RED);
  EXPECT_FALSE(filter_->is_feasible(traj_straight, context_));
  EXPECT_FALSE(filter_->is_feasible(traj_left, context_));
  EXPECT_FALSE(filter_->is_feasible(traj_right, context_));

  // 2. RED + STRAIGHT GREEN ARROW
  set_traffic_light_signals(
    light_id, {{TrafficLightElement::RED, TrafficLightElement::CIRCLE},
               {TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW}});
  EXPECT_TRUE(filter_->is_feasible(traj_straight, context_));
  EXPECT_FALSE(filter_->is_feasible(traj_left, context_));
  EXPECT_FALSE(filter_->is_feasible(traj_right, context_));

  // 3. RED + LEFT GREEN ARROW
  set_traffic_light_signals(
    light_id, {{TrafficLightElement::RED, TrafficLightElement::CIRCLE},
               {TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW}});
  EXPECT_FALSE(filter_->is_feasible(traj_straight, context_));
  EXPECT_TRUE(filter_->is_feasible(traj_left, context_));
  EXPECT_FALSE(filter_->is_feasible(traj_right, context_));

  // 4. RED + RIGHT GREEN ARROW
  set_traffic_light_signals(
    light_id, {{TrafficLightElement::RED, TrafficLightElement::CIRCLE},
               {TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW}});
  EXPECT_FALSE(filter_->is_feasible(traj_straight, context_));
  EXPECT_FALSE(filter_->is_feasible(traj_left, context_));
  EXPECT_TRUE(filter_->is_feasible(traj_right, context_));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
