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

#include "../../../src/filters/safety/collision_check_filter/assessment.hpp"
#include "../../../src/filters/safety/collision_check_filter/parameter.hpp"
#include "../../../src/filters/safety/collision_check_filter/trajectory_utils.hpp"

#include <gtest/gtest.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
namespace
{
constexpr double kDefaultTimeResolution = GlobalParams{}.time_resolution;

geometry_msgs::msg::Pose create_pose(const double x, const double y, const double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);
  return pose;
}

geometry_msgs::msg::Twist create_twist(
  const double linear_x, const double angular_z = 0.0, const double linear_y = 0.0)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear_x;
  twist.linear.y = linear_y;
  twist.angular.z = angular_z;
  return twist;
}

TrajectoryPoints create_straight_trajectory_points(const std::vector<double> & xs)
{
  TrajectoryPoints traj_points;
  traj_points.reserve(xs.size());
  for (const auto x : xs) {
    TrajectoryPoint point;
    point.pose = create_pose(x, 0.0, 0.0);
    traj_points.push_back(point);
  }
  return traj_points;
}

TrajectoryPoints create_straight_timed_trajectory_points(
  const std::vector<double> & xs, const std::vector<double> & times)
{
  if (xs.size() != times.size()) {
    throw std::invalid_argument("xs and times must have the same size");
  }

  TrajectoryPoints traj_points;
  traj_points.reserve(xs.size());
  for (size_t i = 0; i < xs.size(); ++i) {
    TrajectoryPoint point;
    point.pose = create_pose(xs.at(i), 0.0, 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(times.at(i));
    traj_points.push_back(point);
  }
  return traj_points;
}

nav_msgs::msg::Odometry::ConstSharedPtr create_odometry(
  const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Twist & twist = create_twist(0.0))
{
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose = pose;
  odometry->twist.twist = twist;
  odometry->header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  return odometry;
}

FilterContext create_filter_context(const nav_msgs::msg::Odometry::ConstSharedPtr & odometry)
{
  FilterContext context;
  context.odometry = odometry;
  return context;
}

AssessmentTrajectories make_assessment_trajectories(
  const bool map_based, const bool constant_curvature)
{
  return AssessmentTrajectories{map_based, constant_curvature};
}

template <typename ParamMap>
ParamMap make_param_map_with_assessment_trajectories(const AssessmentTrajectories & assessment)
{
  ParamMap param_map;
  for (const auto & [label, class_name] : kObjectClassifications) {
    (void)label;
    param_map[class_name].assessment_trajectories = assessment;
  }
  param_map[kCollisionCheckParamBaseKey].assessment_trajectories = assessment;
  return param_map;
}

autoware_perception_msgs::msg::Shape create_bounding_box_shape(
  const double length = 4.0, const double width = 2.0)
{
  autoware_perception_msgs::msg::Shape shape;
  shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  shape.dimensions.x = length;
  shape.dimensions.y = width;
  shape.dimensions.z = 1.5;
  return shape;
}

autoware_perception_msgs::msg::Shape create_cylinder_shape(const double diameter = 2.0)
{
  autoware_perception_msgs::msg::Shape shape;
  shape.type = autoware_perception_msgs::msg::Shape::CYLINDER;
  shape.dimensions.x = diameter;
  shape.dimensions.y = diameter;
  shape.dimensions.z = 1.5;
  return shape;
}

autoware_perception_msgs::msg::PredictedPath create_straight_predicted_path(
  const double y, const double confidence, const std::vector<double> & xs)
{
  autoware_perception_msgs::msg::PredictedPath predicted_path;
  predicted_path.confidence = confidence;
  predicted_path.time_step = rclcpp::Duration::from_seconds(kDefaultTimeResolution);
  predicted_path.path.reserve(xs.size());
  for (const auto x : xs) {
    predicted_path.path.push_back(create_pose(x, y, 0.0));
  }
  return predicted_path;
}

autoware_perception_msgs::msg::PredictedObject create_predicted_object(
  const geometry_msgs::msg::Pose & initial_pose, const geometry_msgs::msg::Twist & initial_twist,
  const autoware_perception_msgs::msg::Shape & shape,
  const std::vector<autoware_perception_msgs::msg::PredictedPath> & predicted_paths)
{
  autoware_perception_msgs::msg::PredictedObject object;
  object.object_id = autoware_utils_uuid::generate_uuid();
  object.kinematics.initial_pose_with_covariance.pose = initial_pose;
  object.kinematics.initial_twist_with_covariance.twist = initial_twist;
  object.kinematics.predicted_paths = predicted_paths;
  object.shape = shape;
  object.classification.resize(1);
  object.classification.front().label =
    autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  object.classification.front().probability = 1.0F;
  return object;
}

VehicleInfo create_vehicle_info()
{
  VehicleInfo vehicle_info;
  vehicle_info.max_longitudinal_offset_m = 4.0;
  vehicle_info.min_longitudinal_offset_m = -1.0;
  vehicle_info.vehicle_width_m = 2.0;
  return vehicle_info;
}

void expect_same_polygon(const Polygon2d & actual, const Polygon2d & expected)
{
  ASSERT_EQ(actual.outer().size(), expected.outer().size());
  for (size_t i = 0; i < actual.outer().size(); ++i) {
    EXPECT_DOUBLE_EQ(actual.outer().at(i).x(), expected.outer().at(i).x());
    EXPECT_DOUBLE_EQ(actual.outer().at(i).y(), expected.outer().at(i).y());
  }
}

size_t footprint_count(const FootprintTrajectory & footprints)
{
  return std::visit([](const auto & polygons) { return polygons.size(); }, footprints);
}

Polygon2d footprint_to_polygon2d(const FootprintTrajectory & footprints, const size_t index)
{
  return std::visit(
    [&](const auto & polygons) {
      Polygon2d polygon;
      const auto & points = polygons.at(index);
      polygon.outer().reserve(points.size() + 1U);
      for (const auto & point : points) {
        polygon.outer().push_back(point);
      }
      if (!polygon.outer().empty()) {
        polygon.outer().push_back(polygon.outer().front());
      }
      return polygon;
    },
    footprints);
}

}  // namespace

TEST(TrajectoryUtilitiesTest, ComputePoseTrajectoryInterpolatesAndClamps)
{
  const auto traj_points = create_straight_trajectory_points({0.0, 10.0, 20.0});
  const TravelDistanceTrajectory distances = {0.0, 5.0, 15.0, 25.0};

  const auto poses = trajectory::pose::compute_pose_trajectory(traj_points, distances);

  ASSERT_EQ(poses.size(), distances.size());
  EXPECT_DOUBLE_EQ(poses.at(0).position.x, 0.0);
  EXPECT_DOUBLE_EQ(poses.at(1).position.x, 5.0);
  EXPECT_DOUBLE_EQ(poses.at(2).position.x, 15.0);
  EXPECT_DOUBLE_EQ(poses.at(3).position.x, 20.0);
  EXPECT_DOUBLE_EQ(poses.at(3).position.y, 0.0);
}

TEST(TrajectoryUtilitiesTest, ComputePoseTrajectoryInterpolatesOrientationSpherically)
{
  TrajectoryPoint start_point;
  start_point.pose = create_pose(0.0, 0.0, 0.0);

  TrajectoryPoint end_point;
  end_point.pose = create_pose(10.0, 0.0, M_PI / 3.0);

  const TrajectoryPoints traj_points = {start_point, end_point};
  const TravelDistanceTrajectory distances = {5.0};

  const auto poses = trajectory::pose::compute_pose_trajectory(traj_points, distances);

  ASSERT_EQ(poses.size(), 1u);
  EXPECT_DOUBLE_EQ(poses.at(0).position.x, 5.0);
  EXPECT_DOUBLE_EQ(poses.at(0).position.y, 0.0);
  EXPECT_NEAR(tf2::getYaw(poses.at(0).orientation), M_PI / 6.0, 1e-6);
}

TEST(TrajectoryUtilitiesTest, ComputePoseTrajectoryFromTimeInterpolatesAndExtrapolates)
{
  const auto traj_points =
    create_straight_timed_trajectory_points({0.0, 10.0, 20.0}, {0.0, 1.0, 2.0});
  const TimeTrajectory times = {-0.5, 0.5, 1.5, 3.0};

  const auto poses = trajectory::pose::compute_pose_trajectory_from_time(traj_points, times);

  ASSERT_EQ(poses.size(), times.size());
  EXPECT_DOUBLE_EQ(poses.at(0).position.x, -5.0);
  EXPECT_DOUBLE_EQ(poses.at(1).position.x, 5.0);
  EXPECT_DOUBLE_EQ(poses.at(2).position.x, 15.0);
  EXPECT_DOUBLE_EQ(poses.at(3).position.x, 30.0);
}

TEST(TrajectoryUtilitiesTest, ComputePoseTrajectoryFromTimeWithSinglePointReturnsSamePose)
{
  const auto traj_points = create_straight_timed_trajectory_points({3.0}, {1.0});
  const TimeTrajectory times = {-1.0, 0.0, 1.0, 2.0};

  const auto poses = trajectory::pose::compute_pose_trajectory_from_time(traj_points, times);

  ASSERT_EQ(poses.size(), times.size());
  for (const auto & pose : poses) {
    EXPECT_DOUBLE_EQ(pose.position.x, 3.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(tf2::getYaw(pose.orientation), 0.0);
  }
}

TEST(TrajectoryUtilitiesTest, ComputePoseTrajectoryFromTimeHandlesNonUniformTimeAndPositionSpacing)
{
  const auto traj_points =
    create_straight_timed_trajectory_points({0.0, 3.0, 9.0}, {0.0, 0.3, 1.5});
  const TimeTrajectory times = {0.15, 0.9, 1.8};

  const auto poses = trajectory::pose::compute_pose_trajectory_from_time(traj_points, times);

  ASSERT_EQ(poses.size(), times.size());
  EXPECT_NEAR(poses.at(0).position.x, 1.5, 1e-6);
  EXPECT_NEAR(poses.at(1).position.x, 6.0, 1e-6);
  EXPECT_NEAR(poses.at(2).position.x, 10.5, 1e-6);
}

TEST(TrajectoryUtilitiesTest, ComputeFootprintTrajectoryForObjectShapeMatchesUtility)
{
  const PoseTrajectory poses = {create_pose(1.0, 2.0, 0.0)};
  const auto shape = create_bounding_box_shape(4.0, 2.0);

  const auto footprints = trajectory::footprint::compute_footprint_trajectory(poses, shape);

  EXPECT_TRUE(std::holds_alternative<QuadTrajectory>(footprints));
  ASSERT_EQ(footprint_count(footprints), 1u);
  expect_same_polygon(
    footprint_to_polygon2d(footprints, 0U),
    autoware_utils_geometry::to_polygon2d(poses.front(), shape));
}

TEST(TrajectoryUtilitiesTest, ComputeFootprintTrajectoryForCylinderUsesNgonTrajectory)
{
  const PoseTrajectory poses = {create_pose(1.0, 2.0, 0.0)};
  const auto shape = create_cylinder_shape(2.0);

  const auto footprints = trajectory::footprint::compute_footprint_trajectory(poses, shape);

  EXPECT_TRUE(std::holds_alternative<QuadTrajectory>(footprints));
  ASSERT_EQ(footprint_count(footprints), 1u);
  expect_same_polygon(
    footprint_to_polygon2d(footprints, 0U), geometry::to_polygon2d(poses.front(), shape));
}

TEST(TrajectoryUtilitiesTest, ComputeFootprintTrajectoryForVehicleMatchesUtility)
{
  const PoseTrajectory poses = {create_pose(1.0, 2.0, 0.0)};
  const auto vehicle_info = create_vehicle_info();
  const trajectory::footprint::EgoDimensions ego_dimensions{
    vehicle_info.max_longitudinal_offset_m, -vehicle_info.min_longitudinal_offset_m,
    vehicle_info.vehicle_width_m};

  const auto footprints =
    trajectory::footprint::compute_footprint_trajectory(poses, ego_dimensions);

  EXPECT_TRUE(std::holds_alternative<QuadTrajectory>(footprints));
  ASSERT_EQ(footprint_count(footprints), 1u);
  expect_same_polygon(
    footprint_to_polygon2d(footprints, 0U),
    autoware_utils_geometry::to_footprint(
      poses.front(), vehicle_info.max_longitudinal_offset_m,
      -vehicle_info.min_longitudinal_offset_m, vehicle_info.vehicle_width_m));
}

TEST(TrajectoryUtilitiesTest, ComputeFootprintTrajectoryForVehicleAppliesSpecifiedDimensions)
{
  const PoseTrajectory poses = {create_pose(1.0, 2.0, 0.0)};
  const auto vehicle_info = create_vehicle_info();
  const trajectory::footprint::EgoDimensions ego_dimensions{
    vehicle_info.max_longitudinal_offset_m + 0.7, -vehicle_info.min_longitudinal_offset_m + 0.5,
    vehicle_info.vehicle_width_m + 0.6};

  const auto footprints =
    trajectory::footprint::compute_footprint_trajectory(poses, ego_dimensions);

  EXPECT_TRUE(std::holds_alternative<QuadTrajectory>(footprints));
  ASSERT_EQ(footprint_count(footprints), 1u);
  expect_same_polygon(
    footprint_to_polygon2d(footprints, 0U),
    autoware_utils_geometry::to_footprint(
      poses.front(), vehicle_info.max_longitudinal_offset_m + 0.7,
      -vehicle_info.min_longitudinal_offset_m + 0.5, vehicle_info.vehicle_width_m + 0.6));
}

TEST(TrajectoryUtilitiesTest, ObjectIdentificationClassificationConstructorSetsDefaults)
{
  const auto identification = TrajectoryIdentification{"EGO"};

  EXPECT_EQ(identification.classification, "EGO");
  EXPECT_EQ(identification.stamp.sec, 0);
  EXPECT_EQ(identification.stamp.nanosec, 0u);
  EXPECT_TRUE(identification.trajectory_type.empty());
}

TEST(TrajectoryUtilitiesTest, ObjectIdentificationObjectConstructorBuildsIdsFromObject)
{
  auto object = create_predicted_object(
    create_pose(0.0, 0.0, 0.0), create_twist(1.0), create_bounding_box_shape(), {});
  object.classification.resize(2);
  object.classification.at(0).label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  object.classification.at(0).probability = 0.1F;
  object.classification.at(1).label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  object.classification.at(1).probability = 0.9F;

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 34;

  const auto identification = TrajectoryIdentification{object, stamp, "map_based_predicted_path"};

  EXPECT_EQ(identification.classification, "car");
  EXPECT_EQ(identification.stamp.sec, stamp.sec);
  EXPECT_EQ(identification.stamp.nanosec, stamp.nanosec);
  EXPECT_EQ(
    identification.object_id_string(), autoware_utils_uuid::to_hex_string(object.object_id));
}

TEST(TrajectoryUtilitiesTest, GenerateEgoTrajectoryBuildsConsistentTrajectoryData)
{
  auto vehicle_info = create_vehicle_info();
  const auto traj_points = create_straight_trajectory_points({0.0, 10.0, 20.0});
  const auto initial_twist = create_twist(2.0);
  const trajectory::footprint::EgoDimensions ego_dimensions{
    vehicle_info.max_longitudinal_offset_m, -vehicle_info.min_longitudinal_offset_m,
    vehicle_info.vehicle_width_m};

  const auto trajectory_data = trajectory::generate_ego_trajectory(
    initial_twist, 0.0, 0.0, 1.05, GlobalParams{}.time_resolution, traj_points, ego_dimensions);

  ASSERT_EQ(trajectory_data.getObjectIdentification().classification, "EGO");
  ASSERT_TRUE(trajectory_data.getObjectIdentification().trajectory_type.empty());
  EXPECT_DOUBLE_EQ(trajectory_data.getTimes().front(), 0.0);
  EXPECT_NEAR(trajectory_data.getTimes().back(), 1.05, 1e-6);
  EXPECT_NEAR(trajectory_data.getDistances().back(), 2.1, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().back().position.x, 2.1, 1e-6);
  expect_same_polygon(
    footprint_to_polygon2d(trajectory_data.getFootprints(), 0U),
    autoware_utils_geometry::to_footprint(
      trajectory_data.getPoses().front(), vehicle_info.max_longitudinal_offset_m,
      -vehicle_info.min_longitudinal_offset_m, vehicle_info.vehicle_width_m));
}

TEST(TrajectoryUtilitiesTest, GenerateTimedEgoTrajectoryProjectsCurrentPoseOntoTrajectory)
{
  auto vehicle_info = create_vehicle_info();
  const auto traj_points =
    create_straight_timed_trajectory_points({0.0, 10.0, 20.0}, {0.0, 1.0, 2.0});
  const auto odometry = create_odometry(create_pose(5.0, 1.0, 0.0));
  const auto context = create_filter_context(odometry);
  const trajectory::footprint::EgoDimensions ego_dimensions{
    vehicle_info.max_longitudinal_offset_m, -vehicle_info.min_longitudinal_offset_m,
    vehicle_info.vehicle_width_m};

  const auto trajectory_data = trajectory::generate_ego_trajectory(
    traj_points, context, 0.25, GlobalParams{}.time_resolution, ego_dimensions);

  ASSERT_EQ(trajectory_data.getObjectIdentification().classification, "EGO");
  ASSERT_EQ(trajectory_data.size(), 3u);
  EXPECT_DOUBLE_EQ(trajectory_data.getTimes().front(), 0.0);
  EXPECT_NEAR(trajectory_data.getTimes().at(1), 0.1, 1e-6);
  EXPECT_NEAR(trajectory_data.getTimes().at(2), 0.2, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().front().position.x, 5.0, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().front().position.y, 0.0, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().at(1).position.x, 6.0, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().at(2).position.x, 7.0, 1e-6);
  EXPECT_NEAR(trajectory_data.getDistances().front(), 0.0, 1e-6);
  EXPECT_NEAR(trajectory_data.getDistances().at(1), 1.0, 1e-6);
  EXPECT_NEAR(trajectory_data.getDistances().at(2), 2.0, 1e-6);
}

TEST(TrajectoryUtilitiesTest, GenerateTimedEgoTrajectoryAllowsExtrapolationBeforeTrajectoryStart)
{
  auto vehicle_info = create_vehicle_info();
  const auto traj_points =
    create_straight_timed_trajectory_points({0.0, 10.0, 20.0}, {0.0, 1.0, 2.0});
  const auto odometry = create_odometry(create_pose(-5.0, 0.0, 0.0));
  const auto context = create_filter_context(odometry);
  const trajectory::footprint::EgoDimensions ego_dimensions{
    vehicle_info.max_longitudinal_offset_m, -vehicle_info.min_longitudinal_offset_m,
    vehicle_info.vehicle_width_m};

  const double projected_time =
    trajectory::detail::project_current_pose_on_trajectory(traj_points, odometry->pose.pose);
  const auto trajectory_data = trajectory::generate_ego_trajectory(
    traj_points, context, 0.25, GlobalParams{}.time_resolution, ego_dimensions);

  EXPECT_NEAR(projected_time, -0.5, 1e-6);
  ASSERT_EQ(trajectory_data.size(), 3u);
  EXPECT_DOUBLE_EQ(trajectory_data.getTimes().front(), 0.0);
  EXPECT_NEAR(trajectory_data.getPoses().front().position.x, -5.0, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().at(1).position.x, -4.0, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().at(2).position.x, -3.0, 1e-6);
  EXPECT_NEAR(trajectory_data.getDistances().at(1), 1.0, 1e-6);
  EXPECT_NEAR(trajectory_data.getDistances().at(2), 2.0, 1e-6);
}

TEST(TrajectoryUtilitiesTest, GenerateTimedEgoTrajectoryWithSinglePointReturnsSingleSample)
{
  auto vehicle_info = create_vehicle_info();
  const auto traj_points = create_straight_timed_trajectory_points({3.0}, {1.0});
  const auto odometry = create_odometry(create_pose(8.0, 1.0, 0.0));
  const auto context = create_filter_context(odometry);
  const trajectory::footprint::EgoDimensions ego_dimensions{
    vehicle_info.max_longitudinal_offset_m, -vehicle_info.min_longitudinal_offset_m,
    vehicle_info.vehicle_width_m};

  const double projected_time =
    trajectory::detail::project_current_pose_on_trajectory(traj_points, odometry->pose.pose);
  const auto trajectory_data = trajectory::generate_ego_trajectory(
    traj_points, context, 1.0, GlobalParams{}.time_resolution, ego_dimensions);

  EXPECT_NEAR(projected_time, 1.0, 1e-6);
  ASSERT_EQ(trajectory_data.size(), 1u);
  EXPECT_DOUBLE_EQ(trajectory_data.getTimes().front(), 0.0);
  EXPECT_DOUBLE_EQ(trajectory_data.getDistances().front(), 0.0);
  EXPECT_DOUBLE_EQ(trajectory_data.getPoses().front().position.x, 3.0);
  EXPECT_DOUBLE_EQ(trajectory_data.getPoses().front().position.y, 0.0);
}

TEST(TrajectoryUtilitiesTest, GenerateTimedEgoTrajectoryHandlesNonUniformTimeAndPositionSpacing)
{
  auto vehicle_info = create_vehicle_info();
  const auto traj_points =
    create_straight_timed_trajectory_points({0.0, 3.0, 9.0}, {0.0, 0.3, 1.5});
  const auto odometry = create_odometry(create_pose(6.0, 0.5, 0.0));
  const auto context = create_filter_context(odometry);
  const trajectory::footprint::EgoDimensions ego_dimensions{
    vehicle_info.max_longitudinal_offset_m, -vehicle_info.min_longitudinal_offset_m,
    vehicle_info.vehicle_width_m};

  const double projected_time =
    trajectory::detail::project_current_pose_on_trajectory(traj_points, odometry->pose.pose);
  const auto trajectory_data = trajectory::generate_ego_trajectory(
    traj_points, context, 0.25, GlobalParams{}.time_resolution, ego_dimensions);

  EXPECT_NEAR(projected_time, 0.9, 1e-6);

  ASSERT_EQ(trajectory_data.size(), 3u);
  EXPECT_DOUBLE_EQ(trajectory_data.getTimes().front(), 0.0);
  EXPECT_NEAR(trajectory_data.getTimes().at(1), 0.1, 1e-6);
  EXPECT_NEAR(trajectory_data.getTimes().at(2), 0.2, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().front().position.x, 6.0, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().at(1).position.x, 6.5, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().at(2).position.x, 7.0, 1e-6);
  EXPECT_NEAR(trajectory_data.getDistances().front(), 0.0, 1e-6);
  EXPECT_NEAR(trajectory_data.getDistances().at(1), 0.5, 1e-6);
  EXPECT_NEAR(trajectory_data.getDistances().at(2), 1.0, 1e-6);
}

TEST(TrajectoryUtilitiesTest, GeneratePredictedPathTrajectoryUsesHighestConfidencePath)
{
  const auto shape = create_bounding_box_shape(4.0, 2.0);
  const auto initial_pose = create_pose(0.0, 0.0, 0.0);
  const auto initial_twist = create_twist(1.0);
  const std::vector<autoware_perception_msgs::msg::PredictedPath> predicted_paths = {
    create_straight_predicted_path(10.0, 0.1, {0.0, 1.0, 2.0, 3.0, 4.0}),
    create_straight_predicted_path(0.0, 0.9, {0.0, 1.0, 2.0, 3.0, 4.0})};
  const auto object = create_predicted_object(initial_pose, initial_twist, shape, predicted_paths);

  const auto trajectory_data = trajectory::generate_predicted_path_trajectory(
    object, 0.0, 0.0, rclcpp::Duration::from_seconds(0.1), 0.35, builtin_interfaces::msg::Time{},
    kDefaultTimeResolution);

  EXPECT_EQ(trajectory_data.getObjectIdentification().trajectory_type, "map_based_predicted_path");
  EXPECT_NEAR(trajectory_data.getTimes().front(), 0.1, 1e-6);
  EXPECT_NEAR(trajectory_data.getTimes().back(), 0.35, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().at(0).position.x, 0.1, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().at(1).position.x, 0.2, 1e-6);
  EXPECT_NEAR(trajectory_data.getPoses().at(2).position.x, 0.3, 1e-6);
  EXPECT_DOUBLE_EQ(trajectory_data.getPoses().at(0).position.y, 0.0);
}

TEST(TrajectoryUtilitiesTest, ComputeLongitudinalVelocityUsesPathYawForPathLongerThanEpsilon)
{
  // The path length is intentionally between 1e-3 and 1e3 to catch threshold typos.
  const PoseTrajectory points = {create_pose(0.0, 0.0, M_PI_2), create_pose(1.0, 0.0, M_PI_2)};
  const auto object = create_predicted_object(
    create_pose(0.5, 0.0, 0.0), create_twist(2.0), create_bounding_box_shape(), {});

  const auto longitudinal_velocity =
    rss_deceleration::compute_longitudinal_velocity(points, object);

  EXPECT_NEAR(longitudinal_velocity, 2.0, 1e-6);
}

TEST(TrajectoryUtilitiesTest, ComputeLongitudinalVelocityFallsBackToFrontPoseYawForDegeneratePath)
{
  const PoseTrajectory points = {
    create_pose(1.0, 2.0, M_PI / 3.0), create_pose(1.0, 2.0, -M_PI / 2.0)};
  const auto object = create_predicted_object(
    create_pose(1.0, 2.0, M_PI / 6.0), create_twist(2.0), create_bounding_box_shape(), {});

  const auto longitudinal_velocity =
    rss_deceleration::compute_longitudinal_velocity(points, object);

  EXPECT_NEAR(longitudinal_velocity, 2.0 * std::cos(M_PI / 6.0), 1e-6);
}

TEST(TrajectoryUtilitiesTest, ComputeLongitudinalVelocityThrowsOnEmptyPoints)
{
  const PoseTrajectory points;
  const auto object = create_predicted_object(
    create_pose(0.0, 0.0, 0.0), create_twist(2.0), create_bounding_box_shape(), {});

  EXPECT_THROW(
    rss_deceleration::compute_longitudinal_velocity(points, object), std::invalid_argument);
}

TEST(TrajectoryUtilitiesTest, GenerateConstantCurvaturePathTrajectoryMatchesPredictor)
{
  const auto shape = create_bounding_box_shape(4.0, 2.0);
  const auto initial_pose = create_pose(1.0, 2.0, 0.0);
  const auto initial_twist = create_twist(1.0, 1.0);
  const auto object = create_predicted_object(initial_pose, initial_twist, shape, {});

  const auto trajectory_data = trajectory::generate_constant_curvature_trajectory(
    object, 0.0, 0.0, rclcpp::Duration::from_seconds(0.0), 0.25, builtin_interfaces::msg::Time{},
    kDefaultTimeResolution);
  const auto [expected_times, expected_distances] =
    trajectory::time_distance::compute_motion_profile_1d(
      initial_twist, 0.0, 0.0, 0.0, 0.25, kDefaultTimeResolution);
  const auto expected_poses = trajectory::pose::constant_curvature_predictor::compute(
    initial_pose, initial_twist, expected_distances);

  ASSERT_EQ(trajectory_data.size(), expected_times.size());
  EXPECT_EQ(trajectory_data.getObjectIdentification().trajectory_type, "constant_curvature_path");
  for (size_t i = 0; i < expected_times.size(); ++i) {
    EXPECT_NEAR(trajectory_data.getTimes().at(i), expected_times.at(i), 1e-6);
    EXPECT_NEAR(trajectory_data.getPoses().at(i).position.x, expected_poses.at(i).position.x, 1e-6);
    EXPECT_NEAR(trajectory_data.getPoses().at(i).position.y, expected_poses.at(i).position.y, 1e-6);
    EXPECT_NEAR(
      tf2::getYaw(trajectory_data.getPoses().at(i).orientation),
      tf2::getYaw(expected_poses.at(i).orientation), 1e-6);
  }
}
TEST(TrajectoryUtilitiesTest, GenerateObjectTrajectoriesRespectsEnabledTypes)
{
  const auto shape = create_bounding_box_shape(4.0, 2.0);
  const auto object = create_predicted_object(
    create_pose(0.0, 0.0, 0.0), create_twist(1.0), shape,
    {create_straight_predicted_path(0.0, 1.0, {0.0, 1.0, 2.0})});

  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  odometry->pose.pose = create_pose(0.0, 0.0, 0.0);

  auto predicted_objects = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  predicted_objects->header.stamp = odometry->header.stamp;
  predicted_objects->objects.push_back(object);

  FilterContext context;
  context.odometry = odometry;
  context.predicted_objects = predicted_objects;

  const auto count_trajectory_type =
    [](const std::vector<TrajectoryData> & trajectories, const std::string & trajectory_type) {
      return std::count_if(trajectories.begin(), trajectories.end(), [&](const auto & trajectory) {
        return trajectory.getObjectIdentification().trajectory_type == trajectory_type;
      });
    };

  const auto all_enabled_drac_param_map = make_param_map_with_assessment_trajectories<DracParamMap>(
    make_assessment_trajectories(true, true));
  const auto all_enabled = collision_timing_assessment::generate_object_trajectories(
    context, 0.2, 0.0, 0.1, all_enabled_drac_param_map);
  EXPECT_EQ(all_enabled.size(), 2u);
  EXPECT_EQ(count_trajectory_type(all_enabled, "map_based_predicted_path"), 1);
  EXPECT_EQ(count_trajectory_type(all_enabled, "constant_curvature_path"), 1);

  const auto constant_curvature_drac_param_map =
    make_param_map_with_assessment_trajectories<DracParamMap>(
      make_assessment_trajectories(false, true));
  const auto constant_curvature_only = collision_timing_assessment::generate_object_trajectories(
    context, 0.2, 0.0, 0.1, constant_curvature_drac_param_map);
  ASSERT_EQ(constant_curvature_only.size(), 1u);
  EXPECT_EQ(
    constant_curvature_only.front().getObjectIdentification().trajectory_type,
    "constant_curvature_path");

  const auto predicted_path_only_drac_param_map =
    make_param_map_with_assessment_trajectories<DracParamMap>(
      make_assessment_trajectories(true, false));
  const auto predicted_path_only = collision_timing_assessment::generate_object_trajectories(
    context, 0.2, 0.0, 0.1, predicted_path_only_drac_param_map);
  EXPECT_EQ(predicted_path_only.size(), 1u);
  EXPECT_EQ(count_trajectory_type(predicted_path_only, "map_based_predicted_path"), 1);
}

TEST(TrajectoryUtilitiesTest, GenerateObjectTrajectoriesReturnsEmptyWhenAllTypesDisabled)
{
  const auto shape = create_bounding_box_shape(4.0, 2.0);
  const auto object = create_predicted_object(
    create_pose(0.0, 0.0, 0.0), create_twist(1.0), shape,
    {create_straight_predicted_path(0.0, 1.0, {0.0, 1.0, 2.0})});

  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  odometry->pose.pose = create_pose(0.0, 0.0, 0.0);

  auto predicted_objects = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  predicted_objects->header.stamp = odometry->header.stamp;
  predicted_objects->objects.push_back(object);

  FilterContext context;
  context.odometry = odometry;
  context.predicted_objects = predicted_objects;

  const auto drac_param_map = make_param_map_with_assessment_trajectories<DracParamMap>(
    make_assessment_trajectories(false, false));

  const auto trajectories = collision_timing_assessment::generate_object_trajectories(
    context, 0.2, 0.0, 0.1, drac_param_map);

  EXPECT_TRUE(trajectories.empty());
}

}  // namespace autoware::trajectory_validator::plugin::safety
