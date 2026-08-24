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

#include "autoware/trajectory_validator/filters/traffic_rule/crosswalk_filter.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using autoware::trajectory_validator::FilterContext;
using autoware::trajectory_validator::plugin::traffic_rule::CrosswalkFilter;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;

namespace
{
constexpr double k_stop_line_x = 10.0;
constexpr double k_far_stop_line_x = 150.0;
constexpr double k_stop_duration = 1.0;
}  // namespace

class CrosswalkFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_crosswalk_filter_node");
    filter_ = std::make_shared<CrosswalkFilter>();

    autoware::vehicle_info_utils::VehicleInfo vehicle_info;
    vehicle_info.max_longitudinal_offset_m = 0.0;
    filter_->set_vehicle_info(vehicle_info);

    validator::Params params;
    params.crosswalk.object_types = {"pedestrian", "bicycle"};
    params.crosswalk.lon_detection_margin = 2.0;
    params.crosswalk.lat_detection_margin = 1.0;
    params.crosswalk.object_clear_time_th = 0.3;
    params.crosswalk.distance_hysteresis_th = 0.3;
    params.crosswalk.overshoot_tolerance = 0.0;
    params.crosswalk.nominal_deceleration = 1.0;
    params.crosswalk.nominal_jerk = 1.0;
    params.crosswalk.stop_duration = k_stop_duration;
    params.crosswalk.arrived_distance_threshold = 5.0;
    filter_->update_parameters(params);

    context_.route = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>();
    context_.predicted_objects = std::make_shared<PredictedObjects>();

    auto acceleration = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
    acceleration->accel.accel.linear.x = 0.0f;
    context_.acceleration = acceleration;

    auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
    odometry->header.stamp = node_->now();
    odometry->twist.twist.linear.x = 5.0f;
    context_.odometry = odometry;
  }

  void set_odometry(const float velocity, const rclcpp::Time & stamp)
  {
    auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
    odometry->header.stamp = stamp;
    odometry->twist.twist.linear.x = velocity;
    context_.odometry = odometry;
  }

  void create_road_only_map()
  {
    lanelet::Point3d l1(lanelet::utils::getId(), 0.0, 2.0, 0.0);
    lanelet::Point3d l2(lanelet::utils::getId(), 200.0, 2.0, 0.0);
    lanelet::Point3d r1(lanelet::utils::getId(), 0.0, -2.0, 0.0);
    lanelet::Point3d r2(lanelet::utils::getId(), 200.0, -2.0, 0.0);

    lanelet::LineString3d left(lanelet::utils::getId(), {l1, l2});
    lanelet::LineString3d right(lanelet::utils::getId(), {r1, r2});
    lanelet::Lanelet road(lanelet::utils::getId(), left, right);
    road.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;

    context_.lanelet_map = lanelet::utils::createMap({road});

    auto route = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>();
    autoware_planning_msgs::msg::LaneletSegment segment;
    segment.preferred_primitive.id = road.id();
    route->segments.push_back(segment);
    context_.route = route;
  }

  // cSpell: ignore unsignaled
  /// Build a road lanelet with an unsignaled crosswalk RE.
  /// Stop line is at @p stop_line_x. Pedestrian detection end-caps are around y = ±6..±8.
  void create_and_set_map_with_crosswalk(const double stop_line_x)
  {
    // Road lanelet along +X
    lanelet::Point3d rl1(lanelet::utils::getId(), 0.0, 2.0, 0.0);
    lanelet::Point3d rl2(lanelet::utils::getId(), 200.0, 2.0, 0.0);
    lanelet::Point3d rr1(lanelet::utils::getId(), 0.0, -2.0, 0.0);
    lanelet::Point3d rr2(lanelet::utils::getId(), 200.0, -2.0, 0.0);
    lanelet::LineString3d road_left(lanelet::utils::getId(), {rl1, rl2});
    lanelet::LineString3d road_right(lanelet::utils::getId(), {rr1, rr2});
    lanelet::Lanelet road(lanelet::utils::getId(), road_left, road_right);
    road.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;

    // Crosswalk lanelet across the road along +Y (pedestrian direction)
    const double cw_x0 = stop_line_x - 2.0;
    const double cw_x1 = stop_line_x + 2.0;
    lanelet::Point3d cl1(lanelet::utils::getId(), cw_x0, -6.0, 0.0);
    lanelet::Point3d cl2(lanelet::utils::getId(), cw_x0, 6.0, 0.0);
    lanelet::Point3d cr1(lanelet::utils::getId(), cw_x1, -6.0, 0.0);
    lanelet::Point3d cr2(lanelet::utils::getId(), cw_x1, 6.0, 0.0);
    lanelet::LineString3d cw_left(lanelet::utils::getId(), {cl1, cl2});
    lanelet::LineString3d cw_right(lanelet::utils::getId(), {cr1, cr2});
    lanelet::Lanelet crosswalk_lanelet(lanelet::utils::getId(), cw_left, cw_right);
    crosswalk_lanelet.attributes()[lanelet::AttributeName::Subtype] =
      lanelet::AttributeValueString::Crosswalk;

    // Stop line across the road
    lanelet::Point3d sl1(lanelet::utils::getId(), stop_line_x, -5.0, 0.0);
    lanelet::Point3d sl2(lanelet::utils::getId(), stop_line_x, 5.0, 0.0);
    lanelet::LineString3d stop_line(lanelet::utils::getId(), {sl1, sl2});

    // Crosswalk polygon (same footprint as the crosswalk lanelet)
    lanelet::Point3d p1(lanelet::utils::getId(), cw_x0, -6.0, 0.0);
    lanelet::Point3d p2(lanelet::utils::getId(), cw_x1, -6.0, 0.0);
    lanelet::Point3d p3(lanelet::utils::getId(), cw_x1, 6.0, 0.0);
    lanelet::Point3d p4(lanelet::utils::getId(), cw_x0, 6.0, 0.0);
    lanelet::Polygon3d crosswalk_area{
      lanelet::LineString3d{lanelet::utils::getId(), lanelet::Points3d{p1, p2, p3, p4}}};

    auto crosswalk_re = lanelet::autoware::Crosswalk::make(
      lanelet::utils::getId(), lanelet::AttributeMap(), crosswalk_lanelet, crosswalk_area,
      {stop_line});
    road.addRegulatoryElement(crosswalk_re);

    context_.lanelet_map = lanelet::utils::createMap({road, crosswalk_lanelet});

    auto route = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>();
    autoware_planning_msgs::msg::LaneletSegment segment;
    segment.preferred_primitive.id = road.id();
    route->segments.push_back(segment);
    context_.route = route;
  }

  static PredictedObject make_pedestrian(const double x, const double y)
  {
    PredictedObject obj;
    obj.object_id = autoware_utils_uuid::generate_uuid();
    obj.kinematics.initial_pose_with_covariance.pose.position.x = x;
    obj.kinematics.initial_pose_with_covariance.pose.position.y = y;
    obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
    obj.shape.type = autoware_perception_msgs::msg::Shape::CYLINDER;
    obj.shape.dimensions.x = 0.5;
    obj.shape.dimensions.y = 0.5;
    obj.shape.dimensions.z = 1.7;
    obj.classification.resize(1);
    obj.classification.front().label = ObjectClassification::PEDESTRIAN;
    obj.classification.front().probability = 1.0f;
    obj.existence_probability = 1.0f;
    return obj;
  }

  void set_pedestrians(const std::vector<PredictedObject> & pedestrians)
  {
    auto objects = std::make_shared<PredictedObjects>();
    objects->objects = pedestrians;
    context_.predicted_objects = objects;
  }

  void set_pedestrian_at(const double x, const double y)
  {
    set_pedestrians({make_pedestrian(x, y)});
  }

  void clear_objects() { context_.predicted_objects = std::make_shared<PredictedObjects>(); }

  static std::vector<TrajectoryPoint> create_trajectory(
    const double start_x, const double end_x, const float velocity = 5.0f)
  {
    std::vector<TrajectoryPoint> points;
    TrajectoryPoint tp1;
    tp1.pose.position.x = start_x;
    tp1.pose.position.y = 0.0;
    tp1.pose.orientation.w = 1.0;
    tp1.longitudinal_velocity_mps = velocity;
    tp1.time_from_start = rclcpp::Duration::from_seconds(0.0);

    TrajectoryPoint tp2;
    tp2.pose.position.x = end_x;
    tp2.pose.position.y = 0.0;
    tp2.pose.orientation.w = 1.0;
    tp2.longitudinal_velocity_mps = velocity;
    tp2.time_from_start = rclcpp::Duration::from_seconds(
      std::abs(end_x - start_x) / std::max(0.1f, std::abs(velocity)));

    points.push_back(tp1);
    points.push_back(tp2);
    return points;
  }

  void expect_feasibility(
    const std::vector<TrajectoryPoint> & points, const bool expected_feasible,
    const std::string & message = "")
  {
    autoware_internal_planning_msgs::msg::CandidateTrajectory candidate_trajectory;
    candidate_trajectory.points = points;
    const auto res = filter_->is_feasible(candidate_trajectory, context_);
    ASSERT_TRUE(res.has_value()) << "is_feasible should not return an error: "
                                 << (res.has_value() ? "" : res.error()) << " " << message;
    EXPECT_EQ(res->is_feasible, expected_feasible) << message;
  }

  std::shared_ptr<CrosswalkFilter> filter_;
  std::shared_ptr<rclcpp::Node> node_;
  FilterContext context_;
};

TEST_F(CrosswalkFilterTest, EmptyTrajectory)
{
  create_and_set_map_with_crosswalk(k_stop_line_x);
  set_pedestrian_at(k_stop_line_x, -7.0);

  expect_feasibility({}, true, "empty trajectory should be accepted");
}

TEST_F(CrosswalkFilterTest, NoCrosswalkInRoute)
{
  create_road_only_map();
  set_odometry(5.0f, node_->now());

  expect_feasibility(
    create_trajectory(0.0, 80.0, 5.0f), true,
    "trajectory on a route without crosswalks should be accepted");
}

TEST_F(CrosswalkFilterTest, NoCrosswalkIntersectingTrajectory)
{
  // Crosswalk stop line is far ahead; with ego speed 5 m/s the checked traj length
  // does not reach it.
  create_and_set_map_with_crosswalk(k_far_stop_line_x);
  set_odometry(5.0f, node_->now());
  set_pedestrian_at(k_far_stop_line_x, -7.0);

  expect_feasibility(
    create_trajectory(0.0, 40.0, 5.0f), true,
    "crosswalk outside the checked trajectory should be ignored");
}

TEST_F(CrosswalkFilterTest, IntersectingCrosswalkWithoutTargetObjects)
{
  create_and_set_map_with_crosswalk(k_stop_line_x);
  set_odometry(5.0f, node_->now());
  clear_objects();

  expect_feasibility(
    create_trajectory(0.0, 80.0, 5.0f), true,
    "intersecting crosswalk with no target objects should be accepted");
}

TEST_F(CrosswalkFilterTest, IntersectingCrosswalkWithTargetObjectEgoMoving)
{
  create_and_set_map_with_crosswalk(k_stop_line_x);
  set_odometry(5.0f, node_->now());
  // Pedestrian in the sidewalk-side detection end-cap (south of the crosswalk)
  set_pedestrian_at(k_stop_line_x, -7.0);

  expect_feasibility(
    create_trajectory(0.0, 80.0, 5.0f), false,
    "crossing while a target object is waiting and ego is moving should be rejected");
}

TEST_F(CrosswalkFilterTest, StopDurationGateWhileEgoStoppedAtCrosswalk)
{
  create_and_set_map_with_crosswalk(k_stop_line_x);
  set_pedestrian_at(k_stop_line_x, -7.0);

  // Ego is stopped near the stop line; planned trajectory still crosses the crosswalk.
  const auto stamp0 = node_->now();
  set_odometry(0.0f, stamp0);
  const auto traj = create_trajectory(8.0, 80.0, 5.0f);

  expect_feasibility(
    traj, false, "should be rejected before the required stop duration has elapsed");

  set_odometry(0.0f, stamp0 + rclcpp::Duration::from_seconds(k_stop_duration + 0.2));
  expect_feasibility(
    traj, true, "should be accepted after ego has waited longer than stop_duration");
}

TEST_F(CrosswalkFilterTest, NewTargetObjectExtendsStopDuration)
{
  create_and_set_map_with_crosswalk(k_stop_line_x);

  // Place pedestrians in opposite sidewalk detection end-caps so they are tracked separately.
  const auto first_object = make_pedestrian(k_stop_line_x, -7.0);
  const auto second_object = make_pedestrian(k_stop_line_x, 7.0);
  set_pedestrians({first_object});

  const auto stamp0 = node_->now();
  set_odometry(0.0f, stamp0);
  const auto traj = create_trajectory(8.0, 80.0, 5.0f);

  expect_feasibility(traj, false, "initially infeasible while waiting for the first object");

  // Just before the first object's wait completes, a second target object appears.
  set_odometry(0.0f, stamp0 + rclcpp::Duration::from_seconds(k_stop_duration - 0.2));
  set_pedestrians({first_object, second_object});
  expect_feasibility(
    traj, false, "still infeasible when a new target object appears near the end of the wait");

  // Past the original first-object deadline: second object has not waited long enough yet.
  set_odometry(0.0f, stamp0 + rclcpp::Duration::from_seconds(1.5 * k_stop_duration));
  set_pedestrians({first_object, second_object});
  expect_feasibility(
    traj, false,
    "should remain infeasible after the first object's wait because the second object is newer");

  // After waiting a full stop_duration from when the second object appeared.
  set_odometry(0.0f, stamp0 + rclcpp::Duration::from_seconds(2.0 * k_stop_duration + 0.2));
  set_pedestrians({first_object, second_object});
  expect_feasibility(
    traj, true, "should become feasible after waiting stop_duration for the newer object");
}
