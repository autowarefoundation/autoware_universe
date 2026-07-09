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

#include "autoware/trajectory_processor/trajectory_modifier_plugins/traffic_light_stop.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_trajectory_processor/trajectory_modifier_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::trajectory_modifier::plugin::InputData;
using autoware::trajectory_modifier::plugin::TrafficLightStop;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware::trajectory_processor::plugin::NodeContext;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

TrajectoryPoint create_trajectory_point(
  double x, double y, double velocity,
  const rclcpp::Duration & time_from_start = rclcpp::Duration(0, 0))
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = 0.0;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = static_cast<float>(velocity);
  point.acceleration_mps2 = 0.0F;
  point.time_from_start = time_from_start;
  return point;
}

TrajectoryPoints create_straight_trajectory(
  double start_x, double end_x, double velocity, double spacing = 1.0)
{
  TrajectoryPoints trajectory;
  const auto duration_sec = std::abs(end_x - start_x) / std::max(0.1, std::abs(velocity));
  const auto num_points = static_cast<size_t>(std::abs(end_x - start_x) / spacing) + 1;
  for (size_t i = 0; i < num_points; ++i) {
    const auto x = start_x + static_cast<double>(i) * spacing;
    if (x > end_x + 1e-6) {
      break;
    }
    const auto ratio = (end_x - start_x) == 0.0 ? 0.0 : (x - start_x) / (end_x - start_x);
    trajectory.push_back(create_trajectory_point(
      x, 0.0, velocity, rclcpp::Duration::from_seconds(ratio * duration_sec)));
  }
  if (trajectory.empty() || trajectory.back().pose.position.x < end_x - 1e-6) {
    trajectory.push_back(
      create_trajectory_point(end_x, 0.0, velocity, rclcpp::Duration::from_seconds(duration_sec)));
  }
  return trajectory;
}

Odometry::ConstSharedPtr make_odometry(
  double x, double y, double velocity, const rclcpp::Time & stamp)
{
  Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.header.stamp = stamp;
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.position.y = y;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = velocity;
  return std::make_shared<const Odometry>(odometry);
}

AccelWithCovarianceStamped::ConstSharedPtr make_acceleration(double accel_x)
{
  AccelWithCovarianceStamped acceleration;
  acceleration.accel.accel.linear.x = accel_x;
  return std::make_shared<const AccelWithCovarianceStamped>(acceleration);
}

TrafficLightGroupArray::ConstSharedPtr make_traffic_light_signal(lanelet::Id id, uint8_t color)
{
  TrafficLightGroup group;
  group.traffic_light_group_id = id;

  TrafficLightElement element;
  element.color = color;
  element.shape = TrafficLightElement::CIRCLE;
  element.status = TrafficLightElement::SOLID_ON;
  element.confidence = 1.0;
  group.elements.push_back(element);

  auto signals = std::make_shared<TrafficLightGroupArray>();
  signals->traffic_light_groups.push_back(group);
  return signals;
}

std::shared_ptr<lanelet::LaneletMap> create_lanelet_map(lanelet::Id light_id, double stop_line_x)
{
  lanelet::Point3d sl1(lanelet::utils::getId(), stop_line_x, -5, 0);
  lanelet::Point3d sl2(lanelet::utils::getId(), stop_line_x, 5, 0);
  lanelet::LineString3d stop_line(lanelet::utils::getId(), {sl1, sl2});

  lanelet::Point3d light_pt(lanelet::utils::getId(), stop_line_x + 5, 5, 5);
  lanelet::LineString3d light_shape(lanelet::utils::getId(), {light_pt});

  auto traffic_light_re =
    lanelet::TrafficLight::make(light_id, lanelet::AttributeMap(), {light_shape}, stop_line);

  lanelet::Point3d l1(lanelet::utils::getId(), 0, -5, 0);
  lanelet::Point3d l2(lanelet::utils::getId(), 200, -5, 0);
  lanelet::Point3d r1(lanelet::utils::getId(), 0, 5, 0);
  lanelet::Point3d r2(lanelet::utils::getId(), 200, 5, 0);

  lanelet::LineString3d left(lanelet::utils::getId(), {l1, l2});
  lanelet::LineString3d right(lanelet::utils::getId(), {r1, r2});

  lanelet::Lanelet lanelet(lanelet::utils::getId(), left, right);
  lanelet.addRegulatoryElement(traffic_light_re);

  return lanelet::utils::createMap({lanelet});
}

LaneletRoute::ConstSharedPtr create_route(lanelet::Id lanelet_id)
{
  auto route = std::make_shared<LaneletRoute>();
  LaneletSegment segment;
  segment.preferred_primitive.id = lanelet_id;
  route->segments.push_back(segment);
  return route;
}

InputData create_input_data(
  Odometry::ConstSharedPtr current_odometry,
  AccelWithCovarianceStamped::ConstSharedPtr current_acceleration,
  std::shared_ptr<lanelet::LaneletMap> lanelet_map = nullptr,
  LaneletRoute::ConstSharedPtr route = nullptr,
  TrafficLightGroupArray::ConstSharedPtr traffic_light_signals = nullptr)
{
  InputData input;
  input.current_odometry = std::move(current_odometry);
  input.current_acceleration = std::move(current_acceleration);
  input.lanelet_map = std::move(lanelet_map);
  input.route = std::move(route);
  input.traffic_light_signals = std::move(traffic_light_signals);
  return input;
}

}  // namespace

class TrafficLightStopIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    auto node_options = rclcpp::NodeOptions{};
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    autoware::test_utils::updateNodeOptions(
      node_options, {autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<rclcpp::Node>("test_traffic_light_stop_node", node_options);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();

    set_up_default_params();

    context_ = std::make_shared<NodeContext>(*node_, time_keeper_, true);
    plugin_ = std::make_unique<TrafficLightStop>();
    plugin_->initialize("test_traffic_light_stop", context_);
    plugin_->update_params(params_);
    odometry_stamp_ = node_->now();
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    plugin_.reset();
    context_.reset();
    node_.reset();
  }

  void set_up_default_params()
  {
    params_.use_traffic_light_stop = true;
    params_.trajectory_time_step = 0.1;

    params_.stopping_constraints.nominal_deceleration = 1.0;
    params_.stopping_constraints.maximum_deceleration = 4.0;
    params_.stopping_constraints.jerk_limit = 3.0;
    params_.stopping_constraints.arrived_distance_threshold = 0.5;

    auto & tl = params_.traffic_light_stop;
    tl.stop_margin = 0.5;
    tl.stop_for_red_light = true;
    tl.stop_for_amber_light = true;
    tl.treat_amber_light_as_red = false;
    tl.treat_unknown_light_as_red = false;
    tl.overshoot_tolerance = 0.0;
    tl.th_stable_duration_red = 0.0;
    tl.th_stable_duration_amber = 0.0;
    tl.th_amber_rejection_hysteresis = 0.0;
    tl.crossing_time_limit = 2.75;
  }

  void create_and_set_map(lanelet::Id light_id, double stop_line_x)
  {
    lanelet_map_ = create_lanelet_map(light_id, stop_line_x);
    route_ = create_route(lanelet_map_->laneletLayer.begin()->id());
    stop_line_x_ = stop_line_x;
  }

  void set_traffic_light_signal(lanelet::Id id, uint8_t color)
  {
    traffic_light_signals_ = make_traffic_light_signal(id, color);
  }

  InputData make_default_input(double velocity = 5.0)
  {
    return create_input_data(
      make_odometry(0.0, 0.0, velocity, odometry_stamp_), make_acceleration(0.0), lanelet_map_,
      route_, traffic_light_signals_);
  }

  void expect_not_modified(
    TrajectoryPoints & trajectory, const InputData & input, const std::string & message = "")
  {
    const bool modified = plugin_->modify_trajectory(trajectory, input);
    EXPECT_FALSE(modified) << message;
  }

  void expect_modified_with_stop_before_stop_line(
    TrajectoryPoints & trajectory, const InputData & input, const std::string & message = "")
  {
    const bool modified = plugin_->modify_trajectory(trajectory, input);
    ASSERT_TRUE(modified) << message;
    EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
    EXPECT_LT(trajectory.back().pose.position.x, stop_line_x_);

    const auto ego_front_offset = context_->vehicle_info.max_longitudinal_offset_m;
    const auto expected_stop_margin = params_.traffic_light_stop.stop_margin + ego_front_offset;
    EXPECT_NEAR(stop_line_x_ - trajectory.back().pose.position.x, expected_stop_margin, 0.5)
      << message;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::unique_ptr<TrafficLightStop> plugin_;
  trajectory_modifier_params::Params params_;
  std::shared_ptr<NodeContext> context_;

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_;
  LaneletRoute::ConstSharedPtr route_;
  TrafficLightGroupArray::ConstSharedPtr traffic_light_signals_;
  double stop_line_x_{0.0};
  rclcpp::Time odometry_stamp_;
};

TEST_F(TrafficLightStopIntegrationTest, TrajectoryNotModifiedWhenDisabled)
{
  params_.use_traffic_light_stop = false;
  plugin_->update_params(params_);

  auto trajectory = create_straight_trajectory(0.0, 10.0, 5.0);
  expect_not_modified(trajectory, InputData{}, "Plugin disabled should not modify trajectory");
}

TEST_F(TrafficLightStopIntegrationTest, TrajectoryNotModifiedForEmptyTrajectory)
{
  create_and_set_map(0, 5.0);
  set_traffic_light_signal(0, TrafficLightElement::RED);

  TrajectoryPoints empty_trajectory;
  expect_not_modified(
    empty_trajectory, make_default_input(), "Empty trajectory should not be modified");
}

TEST_F(TrafficLightStopIntegrationTest, TrajectoryNotModifiedWithoutMap)
{
  create_and_set_map(0, 15.0);
  set_traffic_light_signal(0, TrafficLightElement::RED);

  auto trajectory = create_straight_trajectory(0.0, 16.0, 5.0);
  const auto input = create_input_data(
    make_odometry(0.0, 0.0, 5.0, odometry_stamp_), make_acceleration(0.0), nullptr, route_,
    traffic_light_signals_);

  expect_not_modified(trajectory, input, "Missing map should not modify trajectory");
}

TEST_F(TrafficLightStopIntegrationTest, TrajectoryNotModifiedWithoutSignals)
{
  create_and_set_map(0, 15.0);

  auto trajectory = create_straight_trajectory(0.0, 16.0, 5.0);
  const auto input = create_input_data(
    make_odometry(0.0, 0.0, 5.0, odometry_stamp_), make_acceleration(0.0), lanelet_map_, route_,
    nullptr);

  expect_not_modified(trajectory, input, "Missing signals should not modify trajectory");
}

TEST_F(TrafficLightStopIntegrationTest, TrajectoryNotModifiedWithoutRoute)
{
  create_and_set_map(0, 15.0);
  set_traffic_light_signal(0, TrafficLightElement::RED);

  auto trajectory = create_straight_trajectory(0.0, 16.0, 5.0);
  const auto input = create_input_data(
    make_odometry(0.0, 0.0, 5.0, odometry_stamp_), make_acceleration(0.0), lanelet_map_, nullptr,
    traffic_light_signals_);

  expect_not_modified(trajectory, input, "Missing route should not modify trajectory");
}

TEST_F(TrafficLightStopIntegrationTest, TrajectoryModifiedWithRedLightIntersection)
{
  const lanelet::Id light_id = 100;
  const double stop_x = 15.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::RED);

  auto trajectory = create_straight_trajectory(0.0, 16.0, 5.0);
  expect_modified_with_stop_before_stop_line(
    trajectory, make_default_input(), "Should insert stop point when crossing red light stop line");
}

TEST_F(TrafficLightStopIntegrationTest, TrajectoryNotModifiedWithGreenLight)
{
  const lanelet::Id light_id = 101;
  const double stop_x = 15.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::GREEN);

  auto trajectory = create_straight_trajectory(0.0, 16.0, 5.0);
  expect_not_modified(trajectory, make_default_input(), "Green light should not modify trajectory");
}

TEST_F(TrafficLightStopIntegrationTest, TrajectoryNotModifiedWithTrajectoryEndingBeforeRedLight)
{
  const lanelet::Id light_id = 102;
  const double stop_x = 15.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::RED);

  const auto ego_front_offset = context_->vehicle_info.max_longitudinal_offset_m;
  const auto end_x = stop_x - ego_front_offset - 0.5;
  auto trajectory = create_straight_trajectory(0.0, end_x, 5.0);

  expect_not_modified(
    trajectory, make_default_input(),
    "Trajectory ending before stop line (accounting for overhang) should not be modified");
}

TEST_F(TrafficLightStopIntegrationTest, TrajectoryModifiedWithRedLightFrontOverhang)
{
  const lanelet::Id light_id = 103;
  const double stop_x = 15.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::RED);

  auto trajectory = create_straight_trajectory(0.0, 13.0, 5.0);
  expect_modified_with_stop_before_stop_line(
    trajectory, make_default_input(),
    "Vehicle front overhang crossing stop line should trigger stop insertion");
}

TEST_F(TrafficLightStopIntegrationTest, TrajectoryModifiedWithAmberLightCanStop)
{
  const lanelet::Id light_id = 200;
  const double stop_x = 10.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::AMBER);

  auto trajectory = create_straight_trajectory(0.0, 11.0, 5.0);
  expect_modified_with_stop_before_stop_line(
    trajectory, make_default_input(), "Should insert stop point when amber light is stoppable");
}

TEST_F(TrafficLightStopIntegrationTest, TrajectoryNotModifiedWithAmberLightCannotStopAndCanPass)
{
  const lanelet::Id light_id = 201;
  const double stop_x = 5.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::AMBER);

  auto trajectory = create_straight_trajectory(0.0, 10.0, 10.0);
  expect_not_modified(
    trajectory, make_default_input(10.0),
    "Should not modify trajectory when amber light cannot be stopped but is reachable");
}

TEST_F(TrafficLightStopIntegrationTest, TrajectoryModifiedWithAmberLightAsRedLight)
{
  const lanelet::Id light_id = 300;
  const double stop_x = 15.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::AMBER);

  params_.traffic_light_stop.treat_amber_light_as_red = true;
  plugin_->update_params(params_);

  auto trajectory = create_straight_trajectory(0.0, 16.0, 10.0);
  const bool modified = plugin_->modify_trajectory(trajectory, make_default_input(10.0));
  EXPECT_TRUE(modified) << "Amber treated as red should insert stop point even when not stoppable";
}

TEST_F(TrafficLightStopIntegrationTest, TrajectoryNotModifiedWithUnknownLight)
{
  const lanelet::Id light_id = 301;
  const double stop_x = 10.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::UNKNOWN);

  auto trajectory = create_straight_trajectory(0.0, 11.0, 5.0);
  expect_not_modified(
    trajectory, make_default_input(),
    "Unknown light without treat-as-red parameter should not modify trajectory");
}

TEST_F(TrafficLightStopIntegrationTest, TrajectoryModifiedWithUnknownLightAsRedLight)
{
  const lanelet::Id light_id = 302;
  const double stop_x = 10.0;

  create_and_set_map(light_id, stop_x);
  set_traffic_light_signal(light_id, TrafficLightElement::UNKNOWN);

  params_.traffic_light_stop.treat_unknown_light_as_red = true;
  plugin_->update_params(params_);

  auto trajectory = create_straight_trajectory(0.0, 16.0, 10.0);
  const bool modified = plugin_->modify_trajectory(trajectory, make_default_input(10.0));
  EXPECT_TRUE(modified)
    << "Unknown treated as red should insert stop point even when not stoppable";
}
