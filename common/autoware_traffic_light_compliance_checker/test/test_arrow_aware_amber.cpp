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

#include "autoware/traffic_light_compliance_checker/traffic_light_compliance_checker.hpp"
#include "autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp"
#include "autoware_lanelet2_extension/utility/utilities.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace
{
using autoware::traffic_light_compliance_checker::Inputs;
using autoware::traffic_light_compliance_checker::Parameters;
using autoware::traffic_light_compliance_checker::TrafficLightComplianceChecker;
using autoware::traffic_light_compliance_checker::ViolationType;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::TrajectoryPoint;

constexpr lanelet::Id k_light_id = 1000;
constexpr double k_stop_line_x = 30.0;

Parameters make_default_params()
{
  Parameters p{};
  p.deceleration_limit = 1.0;
  p.jerk_limit = 1.0;
  p.delay_response_time = 0.0;
  p.crossing_time_limit = 0.1;  // force amber pass kinematics to fail when stop is required
  p.treat_amber_light_as_red_light = false;
  p.treat_unknown_light_as_red_light = false;
  p.enable_arrow_aware_amber_passing = true;
  p.stop_overshoot_margin = 0.0;
  p.allow_if_cannot_stop_distance = 0.0;
  p.min_lookahead_distance = 100.0;
  p.stable_duration_threshold_red = 0.0;
  p.stable_duration_threshold_amber = 0.0;
  p.stable_duration_threshold_unknown = 0.0;
  p.amber_rejection_hysteresis_duration = 0.0;
  p.ego_stopped_velocity_threshold = 0.01;
  p.checked_trajectory_length.deceleration_limit = 999.0;
  p.checked_trajectory_length.jerk_limit = 999.0;
  return p;
}

autoware::vehicle_info_utils::VehicleInfo make_vehicle_info()
{
  autoware::vehicle_info_utils::VehicleInfo info;
  info.max_longitudinal_offset_m = 0.0;
  return info;
}

std::shared_ptr<lanelet::LaneletMap> create_map(
  const std::string & turn_direction, const bool with_static_arrow)
{
  lanelet::Point3d sl1(lanelet::utils::getId(), k_stop_line_x, -5.0, 0.0);
  lanelet::Point3d sl2(lanelet::utils::getId(), k_stop_line_x, 5.0, 0.0);
  lanelet::LineString3d stop_line(lanelet::utils::getId(), {sl1, sl2});

  lanelet::Point3d light_pt1(lanelet::utils::getId(), k_stop_line_x + 5.0, 5.0, 5.0);
  lanelet::Point3d light_pt2(lanelet::utils::getId(), k_stop_line_x + 5.0, 4.0, 5.0);
  lanelet::LineString3d light_shape(lanelet::utils::getId(), {light_pt1, light_pt2});
  if (with_static_arrow) {
    light_shape.attributes()["subtype"] = "right_arrow";
  } else {
    light_shape.attributes()["subtype"] = "red_yellow_green";
  }

  auto traffic_light_re = lanelet::autoware::AutowareTrafficLight::make(
    k_light_id, lanelet::AttributeMap(), {light_shape}, stop_line);

  lanelet::Point3d l1(lanelet::utils::getId(), 0.0, -5.0, 0.0);
  lanelet::Point3d l2(lanelet::utils::getId(), 200.0, -5.0, 0.0);
  lanelet::Point3d r1(lanelet::utils::getId(), 0.0, 5.0, 0.0);
  lanelet::Point3d r2(lanelet::utils::getId(), 200.0, 5.0, 0.0);
  lanelet::LineString3d left(lanelet::utils::getId(), {l1, l2});
  lanelet::LineString3d right(lanelet::utils::getId(), {r1, r2});

  lanelet::Lanelet lanelet(lanelet::utils::getId(), left, right);
  if (!turn_direction.empty()) {
    lanelet.attributes()["turn_direction"] = turn_direction;
  }
  lanelet.addRegulatoryElement(traffic_light_re);

  return lanelet::utils::createMap({lanelet});
}

LaneletRoute create_route(const lanelet::Id lanelet_id)
{
  LaneletRoute route;
  LaneletSegment segment;
  segment.preferred_primitive.id = lanelet_id;
  route.segments.push_back(segment);
  return route;
}

std::vector<TrajectoryPoint> create_crossing_trajectory(const double velocity)
{
  std::vector<TrajectoryPoint> trajectory;
  constexpr double spacing = 1.0;
  constexpr double end_x = 60.0;
  const auto duration_sec = end_x / velocity;
  for (double x = 0.0; x <= end_x + 1e-6; x += spacing) {
    TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = 0.0;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = static_cast<float>(velocity);
    point.acceleration_mps2 = 0.0f;
    point.time_from_start = rclcpp::Duration::from_seconds((x / end_x) * duration_sec);
    trajectory.push_back(point);
  }
  return trajectory;
}

TrafficLightElement make_element(uint8_t color, uint8_t shape = TrafficLightElement::CIRCLE)
{
  TrafficLightElement element;
  element.color = color;
  element.shape = shape;
  element.status = TrafficLightElement::SOLID_ON;
  element.confidence = 1.0f;
  return element;
}

TrafficLightGroupArray make_signals(const TrafficLightElement & element)
{
  TrafficLightGroup group;
  group.traffic_light_group_id = k_light_id;
  group.elements.push_back(element);

  TrafficLightGroupArray signals;
  signals.traffic_light_groups.push_back(group);
  return signals;
}

Inputs make_inputs(
  const std::shared_ptr<lanelet::LaneletMap> & map, const TrafficLightGroupArray & signals,
  const rclcpp::Time & time)
{
  const auto lanelet_id = map->laneletLayer.begin()->id();
  Inputs input;
  input.trajectory = create_crossing_trajectory(10.0);
  input.map = map;
  input.route = create_route(lanelet_id);
  input.signals = signals;
  input.current_time = time;
  input.current_velocity = 10.0;
  input.current_acceleration = 0.0;
  return input;
}
}  // namespace

class ArrowAwareAmberTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    params_ = make_default_params();
    checker_ = std::make_unique<TrafficLightComplianceChecker>(params_, make_vehicle_info());
  }

  void feed_signal(const TrafficLightGroupArray & signals, const rclcpp::Time & time)
  {
    // Drive tracker state using a non-crossing check path via check() itself.
    auto map = create_map("right", true);
    auto input = make_inputs(map, signals, time);
    // Short trajectory that does not cross the stop line, only to update tracker state.
    input.trajectory = create_crossing_trajectory(10.0);
    // Truncate before stop line so no violation is recorded while priming history.
    input.trajectory.erase(
      std::remove_if(
        input.trajectory.begin(), input.trajectory.end(),
        [](const TrajectoryPoint & p) { return p.pose.position.x > k_stop_line_x - 5.0; }),
      input.trajectory.end());
    ASSERT_FALSE(input.trajectory.empty());
    const auto result = checker_->check(input, true, true);
    ASSERT_TRUE(result.has_value());
  }

  Parameters params_;
  std::unique_ptr<TrafficLightComplianceChecker> checker_;
};

TEST_F(ArrowAwareAmberTest, GreenToAmberOnTurnLaneWithArrowAllowsPass)
{
  const rclcpp::Time t0(100, 0, RCL_ROS_TIME);
  const rclcpp::Time t1(100, 200000000, RCL_ROS_TIME);

  feed_signal(make_signals(make_element(TrafficLightElement::GREEN)), t0);

  auto map = create_map("right", true);
  auto input = make_inputs(map, make_signals(make_element(TrafficLightElement::AMBER)), t1);
  const auto result = checker_->check(input, true, true);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->violations.empty());
}

TEST_F(ArrowAwareAmberTest, RedToAmberStillViolates)
{
  const rclcpp::Time t0(100, 0, RCL_ROS_TIME);
  const rclcpp::Time t1(100, 200000000, RCL_ROS_TIME);

  feed_signal(make_signals(make_element(TrafficLightElement::RED)), t0);

  auto map = create_map("right", true);
  auto input = make_inputs(map, make_signals(make_element(TrafficLightElement::AMBER)), t1);
  const auto result = checker_->check(input, true, true);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->violations.empty());
  EXPECT_EQ(result->violations.front().type, ViolationType::AMBER_LIGHT);
}

TEST_F(ArrowAwareAmberTest, NonTurnLaneStillViolates)
{
  const rclcpp::Time t0(100, 0, RCL_ROS_TIME);
  const rclcpp::Time t1(100, 200000000, RCL_ROS_TIME);

  // Prime tracker with green using a turn+arrow map (tracker is per TL id, not lane).
  feed_signal(make_signals(make_element(TrafficLightElement::GREEN)), t0);

  auto map = create_map("", true);  // no turn_direction
  auto input = make_inputs(map, make_signals(make_element(TrafficLightElement::AMBER)), t1);
  const auto result = checker_->check(input, true, true);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->violations.empty());
  EXPECT_EQ(result->violations.front().type, ViolationType::AMBER_LIGHT);
}

TEST_F(ArrowAwareAmberTest, NoStaticArrowStillViolates)
{
  const rclcpp::Time t0(100, 0, RCL_ROS_TIME);
  const rclcpp::Time t1(100, 200000000, RCL_ROS_TIME);

  feed_signal(make_signals(make_element(TrafficLightElement::GREEN)), t0);

  auto map = create_map("right", false);
  auto input = make_inputs(map, make_signals(make_element(TrafficLightElement::AMBER)), t1);
  const auto result = checker_->check(input, true, true);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->violations.empty());
  EXPECT_EQ(result->violations.front().type, ViolationType::AMBER_LIGHT);
}

TEST_F(ArrowAwareAmberTest, FeatureDisabledStillViolates)
{
  params_.enable_arrow_aware_amber_passing = false;
  checker_ = std::make_unique<TrafficLightComplianceChecker>(params_, make_vehicle_info());

  const rclcpp::Time t0(100, 0, RCL_ROS_TIME);
  const rclcpp::Time t1(100, 200000000, RCL_ROS_TIME);

  feed_signal(make_signals(make_element(TrafficLightElement::GREEN)), t0);

  auto map = create_map("right", true);
  auto input = make_inputs(map, make_signals(make_element(TrafficLightElement::AMBER)), t1);
  const auto result = checker_->check(input, true, true);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->violations.empty());
  EXPECT_EQ(result->violations.front().type, ViolationType::AMBER_LIGHT);
}

TEST_F(ArrowAwareAmberTest, GreenRightArrowAllowsPass)
{
  const rclcpp::Time t0(100, 0, RCL_ROS_TIME);

  auto map = create_map("right", true);
  auto input = make_inputs(
    map, make_signals(make_element(TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW)),
    t0);
  const auto result = checker_->check(input, true, true);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->violations.empty());
}
