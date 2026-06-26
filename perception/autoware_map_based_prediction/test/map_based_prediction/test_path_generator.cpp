// Copyright 2024 TIER IV, inc.
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

#include "autoware/map_based_prediction/data_structure.hpp"
#include "autoware/map_based_prediction/path_generator/frenet.hpp"
#include "autoware/map_based_prediction/path_generator/path_generator.hpp"
#include "autoware/map_based_prediction/utils.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <tf2/utils.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjectKinematics;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjectKinematics;
using autoware_perception_msgs::msg::TrackedObjects;

TrackedObject generate_static_object(const int label)
{
  ObjectClassification classification;
  classification.probability = 1.0;
  classification.label = label;

  TrackedObjectKinematics kinematics;
  kinematics.pose_with_covariance = geometry_msgs::msg::PoseWithCovariance();    // At origin
  kinematics.twist_with_covariance = geometry_msgs::msg::TwistWithCovariance();  // Not moving
  kinematics.acceleration_with_covariance =
    geometry_msgs::msg::AccelWithCovariance();  // Not accelerating
  kinematics.orientation_availability = TrackedObjectKinematics::UNAVAILABLE;

  TrackedObject tracked_object;
  tracked_object.object_id = unique_identifier_msgs::msg::UUID();
  tracked_object.existence_probability = 1.0;
  tracked_object.classification.push_back(classification);
  tracked_object.kinematics = kinematics;

  return tracked_object;
}

TEST(PathGenerator, test_generatePathForNonVehicleObject)
{
  // Generate Path generator
  const double prediction_time_horizon = 10.0;
  const double prediction_sampling_time_interval = 0.5;
  const double min_crosswalk_user_velocity = 0.1;
  const autoware::map_based_prediction::PathGenerator path_generator =
    autoware::map_based_prediction::PathGenerator(
      prediction_sampling_time_interval, min_crosswalk_user_velocity);

  // Generate pedestrian object
  TrackedObject tracked_object = generate_static_object(ObjectClassification::PEDESTRIAN);

  // Generate predicted path
  const PredictedPath predicted_path =
    path_generator.generatePathForNonVehicleObject(tracked_object, prediction_time_horizon);

  // Check
  EXPECT_FALSE(predicted_path.path.empty());
  EXPECT_EQ(predicted_path.path[0].position.x, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.y, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.z, 0.0);
}

TEST(PathGenerator, test_generatePathForLowSpeedVehicle)
{
  // Generate Path generator
  const double prediction_time_horizon = 10.0;
  const double prediction_sampling_time_interval = 0.5;
  const double min_crosswalk_user_velocity = 0.1;
  const autoware::map_based_prediction::PathGenerator path_generator =
    autoware::map_based_prediction::PathGenerator(
      prediction_sampling_time_interval, min_crosswalk_user_velocity);

  // Generate dummy object
  TrackedObject tracked_object = generate_static_object(ObjectClassification::CAR);

  // Generate predicted path
  const PredictedPath predicted_path =
    path_generator.generatePathForLowSpeedVehicle(tracked_object, prediction_time_horizon);

  // Check
  EXPECT_FALSE(predicted_path.path.empty());
  EXPECT_EQ(predicted_path.path[0].position.x, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.y, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.z, 0.0);
}

TEST(PathGenerator, test_generatePathForOffLaneVehicle)
{
  // Generate Path generator
  const double prediction_time_horizon = 10.0;
  const double prediction_sampling_time_interval = 0.5;
  const double min_crosswalk_user_velocity = 0.1;
  const autoware::map_based_prediction::PathGenerator path_generator =
    autoware::map_based_prediction::PathGenerator(
      prediction_sampling_time_interval, min_crosswalk_user_velocity);

  // Generate dummy object
  TrackedObject tracked_object = generate_static_object(ObjectClassification::CAR);

  const PredictedPath predicted_path =
    path_generator.generatePathForOffLaneVehicle(tracked_object, prediction_time_horizon);

  // Check
  EXPECT_FALSE(predicted_path.path.empty());
  EXPECT_EQ(predicted_path.path[0].position.x, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.y, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.z, 0.0);
}

TEST(PathGenerator, test_generatePathForOnLaneVehicle)
{
  // Generate Path generator
  const double prediction_time_horizon = 10.0;
  const double lateral_control_time_horizon = 5.0;
  const double prediction_sampling_time_interval = 0.5;
  const double min_crosswalk_user_velocity = 0.1;
  const autoware::map_based_prediction::PathGenerator path_generator =
    autoware::map_based_prediction::PathGenerator(
      prediction_sampling_time_interval, min_crosswalk_user_velocity);

  // Generate dummy object
  TrackedObject tracked_object = generate_static_object(ObjectClassification::CAR);

  // Generate reference path
  autoware::map_based_prediction::PosePath ref_paths;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  ref_paths.push_back(pose);

  // Generate predicted path
  const PredictedPath predicted_path = path_generator.generatePathForOnLaneVehicle(
    tracked_object, ref_paths, prediction_time_horizon, lateral_control_time_horizon);

  // Check
  EXPECT_FALSE(predicted_path.path.empty());
  EXPECT_EQ(predicted_path.path[0].position.x, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.y, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.z, 0.0);
}

TEST(PathGenerator, test_generatePathForCrosswalkUser)
{
  // Generate Path generator
  const double prediction_time_horizon = 10.0;
  const double prediction_sampling_time_interval = 0.5;
  const double min_crosswalk_user_velocity = 0.1;
  const autoware::map_based_prediction::PathGenerator path_generator =
    autoware::map_based_prediction::PathGenerator(
      prediction_sampling_time_interval, min_crosswalk_user_velocity);

  // Generate dummy object
  TrackedObject tracked_object = generate_static_object(ObjectClassification::PEDESTRIAN);

  // Generate dummy crosswalk
  autoware::map_based_prediction::CrosswalkEdgePoints reachable_crosswalk;
  reachable_crosswalk.front_center_point << 0.0, 0.0;
  reachable_crosswalk.front_right_point << 1.0, 0.0;
  reachable_crosswalk.front_left_point << -1.0, 0.0;
  reachable_crosswalk.back_center_point << 0.0, 1.0;
  reachable_crosswalk.back_right_point << 1.0, 1.0;
  reachable_crosswalk.back_left_point << -1.0, 1.0;

  // Generate predicted path
  const auto predicted_path = path_generator.generatePathForCrosswalkUser(
    tracked_object, reachable_crosswalk, prediction_time_horizon);

  // Check
  EXPECT_FALSE(predicted_path.path.empty());
  EXPECT_EQ(predicted_path.path[0].position.x, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.y, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.z, 0.0);
}

namespace
{
geometry_msgs::msg::Pose make_pose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
  return pose;
}
}  // namespace

TEST(ShiftPathReference, zero_offset_is_noop)
{
  PredictedPath path;
  path.path.push_back(make_pose(1.0, 2.0, 0.3));
  path.path.push_back(make_pose(3.0, 4.0, 0.6));

  const PredictedPath original = path;
  autoware::map_based_prediction::shiftPathReference(path, Eigen::Vector2d(0.0, 0.0));

  ASSERT_EQ(path.path.size(), original.path.size());
  for (size_t i = 0; i < path.path.size(); ++i) {
    EXPECT_DOUBLE_EQ(path.path[i].position.x, original.path[i].position.x);
    EXPECT_DOUBLE_EQ(path.path[i].position.y, original.path[i].position.y);
  }
}

TEST(ShiftPathReference, straight_path_translates_by_offset)
{
  // yaw == 0: object-local axes align with world axes, so the offset adds directly.
  PredictedPath path;
  path.path.push_back(make_pose(1.0, 1.0, 0.0));
  path.path.push_back(make_pose(2.0, 1.0, 0.0));

  autoware::map_based_prediction::shiftPathReference(path, Eigen::Vector2d(0.5, 0.3));

  EXPECT_DOUBLE_EQ(path.path[0].position.x, 1.5);
  EXPECT_DOUBLE_EQ(path.path[0].position.y, 1.3);
  EXPECT_DOUBLE_EQ(path.path[1].position.x, 2.5);
  EXPECT_DOUBLE_EQ(path.path[1].position.y, 1.3);
}

TEST(ShiftPathReference, offset_rotates_with_pose_yaw)
{
  // yaw == +90 deg: a longitudinal (+x local) offset points along +y in the world frame.
  PredictedPath path;
  path.path.push_back(make_pose(5.0, 5.0, M_PI / 2.0));

  autoware::map_based_prediction::shiftPathReference(path, Eigen::Vector2d(1.0, 0.0));

  EXPECT_NEAR(path.path[0].position.x, 5.0, 1e-9);
  EXPECT_NEAR(path.path[0].position.y, 6.0, 1e-9);
}

TEST(ShiftPathReference, turning_path_is_displaced_and_reversible)
{
  // A curving path (varying yaw) gets each point displaced along its own heading, and the
  // inverse offset restores the original geometry exactly.
  PredictedPath path;
  path.path.push_back(make_pose(0.0, 0.0, 0.0));
  path.path.push_back(make_pose(1.0, 0.2, 0.5));
  path.path.push_back(make_pose(1.8, 0.8, 1.0));
  const PredictedPath original = path;

  constexpr double lr = 1.25;
  autoware::map_based_prediction::shiftPathReference(path, Eigen::Vector2d(lr, 0.0));

  // Every point moves by exactly lr (the displacement magnitude is offset-norm-preserving).
  for (size_t i = 0; i < path.path.size(); ++i) {
    const double dx = path.path[i].position.x - original.path[i].position.x;
    const double dy = path.path[i].position.y - original.path[i].position.y;
    EXPECT_NEAR(std::hypot(dx, dy), lr, 1e-9);
  }
  // The middle/last points point in different directions, so the displacement vectors differ.
  EXPECT_GT(
    std::abs(
      (path.path[2].position.y - original.path[2].position.y) -
      (path.path[0].position.y - original.path[0].position.y)),
    1e-3);

  // Reversible: shifting back by -lr restores the original points.
  autoware::map_based_prediction::shiftPathReference(path, Eigen::Vector2d(-lr, 0.0));
  for (size_t i = 0; i < path.path.size(); ++i) {
    EXPECT_NEAR(path.path[i].position.x, original.path[i].position.x, 1e-9);
    EXPECT_NEAR(path.path[i].position.y, original.path[i].position.y, 1e-9);
  }
}

namespace
{
PredictedObject make_box_object_with_path()
{
  PredictedObject object;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  object.shape.dimensions.z = 1.5;

  object.kinematics.initial_pose_with_covariance.pose = make_pose(10.0, 20.0, 0.0);

  PredictedPath path;
  path.path.push_back(make_pose(10.0, 20.0, 0.0));
  object.kinematics.predicted_paths.push_back(path);
  return object;
}

geometry_msgs::msg::Point32 make_point32(const float x, const float y)
{
  geometry_msgs::msg::Point32 point;
  point.x = x;
  point.y = y;
  return point;
}
}  // namespace

TEST(ExpandShapeToFootprintAndRecenter, noop_without_footprint)
{
  PredictedObject object = make_box_object_with_path();
  autoware::map_based_prediction::utils::expandShapeToFootprintAndRecenter(object);

  EXPECT_DOUBLE_EQ(object.shape.dimensions.x, 4.0);
  EXPECT_DOUBLE_EQ(object.shape.dimensions.y, 2.0);
  EXPECT_DOUBLE_EQ(object.kinematics.predicted_paths[0].path[0].position.x, 10.0);
}

TEST(ExpandShapeToFootprintAndRecenter, noop_when_footprint_within_box)
{
  PredictedObject object = make_box_object_with_path();
  // Footprint fully inside the 4x2 box.
  object.shape.footprint.points.push_back(make_point32(-1.0, -0.5));
  object.shape.footprint.points.push_back(make_point32(1.0, -0.5));
  object.shape.footprint.points.push_back(make_point32(1.0, 0.5));
  object.shape.footprint.points.push_back(make_point32(-1.0, 0.5));

  autoware::map_based_prediction::utils::expandShapeToFootprintAndRecenter(object);

  EXPECT_DOUBLE_EQ(object.shape.dimensions.x, 4.0);
  EXPECT_DOUBLE_EQ(object.shape.dimensions.y, 2.0);
  EXPECT_DOUBLE_EQ(object.kinematics.predicted_paths[0].path[0].position.x, 10.0);
  EXPECT_DOUBLE_EQ(object.kinematics.predicted_paths[0].path[0].position.y, 20.0);
}

TEST(ExpandShapeToFootprintAndRecenter, grows_box_and_recenters_on_protrusion)
{
  PredictedObject object = make_box_object_with_path();
  // Footprint protrudes forward to x = 3 (box half-length is 2).
  object.shape.footprint.points.push_back(make_point32(-2.0, -1.0));
  object.shape.footprint.points.push_back(make_point32(3.0, -1.0));
  object.shape.footprint.points.push_back(make_point32(3.0, 1.0));
  object.shape.footprint.points.push_back(make_point32(-2.0, 1.0));

  autoware::map_based_prediction::utils::expandShapeToFootprintAndRecenter(object);

  // Union bounds: x in [-2, 3] -> length 5, center offset +0.5; y unchanged.
  EXPECT_DOUBLE_EQ(object.shape.dimensions.x, 5.0);
  EXPECT_DOUBLE_EQ(object.shape.dimensions.y, 2.0);
  // Path (yaw 0) recenters by +0.5 in x.
  EXPECT_DOUBLE_EQ(object.kinematics.predicted_paths[0].path[0].position.x, 10.5);
  EXPECT_DOUBLE_EQ(object.kinematics.predicted_paths[0].path[0].position.y, 20.0);
  // The t=0 box center (initial pose) recenters by the same +0.5 in x.
  EXPECT_DOUBLE_EQ(object.kinematics.initial_pose_with_covariance.pose.position.x, 10.5);
  EXPECT_DOUBLE_EQ(object.kinematics.initial_pose_with_covariance.pose.position.y, 20.0);
  // Footprint is the convex hull of the original box and the footprint, re-expressed about the new
  // center. Here the footprint encloses the box, so the hull is the footprint rectangle shifted
  // by -0.5 in x: x in [-2.5, 2.5], y in [-1, 1]. Check bounds order-independently since the
  // boost hull may reorder the ring.
  ASSERT_FALSE(object.shape.footprint.points.empty());
  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  for (const auto & point : object.shape.footprint.points) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }
  EXPECT_FLOAT_EQ(min_x, -2.5f);
  EXPECT_FLOAT_EQ(max_x, 2.5f);
  EXPECT_FLOAT_EQ(min_y, -1.0f);
  EXPECT_FLOAT_EQ(max_y, 1.0f);
}

TEST(ExpandShapeToFootprintAndRecenter, footprint_box_merge_is_convex)
{
  PredictedObject object = make_box_object_with_path();
  // A narrow bump on the front face: footprint covers x in [2, 3], y in [-0.5, 0.5], while the body
  // box is x in [-2, 2], y in [-1, 1]. Neither shape contains the other, so their merge is a true
  // T-shape; the convex hull fills the re-entrant notches at the bump's base.
  object.shape.footprint.points.push_back(make_point32(2.0, -0.5));
  object.shape.footprint.points.push_back(make_point32(3.0, -0.5));
  object.shape.footprint.points.push_back(make_point32(3.0, 0.5));
  object.shape.footprint.points.push_back(make_point32(2.0, 0.5));

  autoware::map_based_prediction::utils::expandShapeToFootprintAndRecenter(object);

  // AABB union: x in [-2, 3] -> length 5, center offset +0.5; y unchanged.
  EXPECT_DOUBLE_EQ(object.shape.dimensions.x, 5.0);
  EXPECT_DOUBLE_EQ(object.shape.dimensions.y, 2.0);

  // The merged footprint is the convex hull of the box and the bump, re-expressed about the new
  // center (shifted by -0.5 in x). It must be convex and must NOT contain the re-entrant notch at
  // x = 1.5 (old x = 2.0), y = +/-0.5 that the raw T-shape would have.
  const auto & points = object.shape.footprint.points;
  ASSERT_GE(points.size(), 3u);
  for (const auto & point : points) {
    const bool is_notch =
      std::abs(point.x - 1.5f) < 1e-3f && std::abs(std::abs(point.y) - 0.5f) < 1e-3f;
    EXPECT_FALSE(is_notch);
  }
  // The bump's outer corners survive on the hull at x = 2.5 (old x = 3.0), y = +/-0.5.
  bool has_bump_corner = false;
  for (const auto & point : points) {
    if (std::abs(point.x - 2.5f) < 1e-3f && std::abs(std::abs(point.y) - 0.5f) < 1e-3f) {
      has_bump_corner = true;
    }
  }
  EXPECT_TRUE(has_bump_corner);

  // Every turn around the ring has the same sign: a strict definition of convexity.
  const size_t n = points.size();
  int sign = 0;
  for (size_t i = 0; i < n; ++i) {
    const auto & a = points[i];
    const auto & b = points[(i + 1) % n];
    const auto & c = points[(i + 2) % n];
    const double cross =
      static_cast<double>(b.x - a.x) * (c.y - b.y) - static_cast<double>(b.y - a.y) * (c.x - b.x);
    if (std::abs(cross) < 1e-6) continue;  // collinear vertices do not break convexity
    const int current = cross > 0.0 ? 1 : -1;
    if (sign == 0) {
      sign = current;
    } else {
      EXPECT_EQ(sign, current);
    }
  }
}

TEST(ExpandShapeToFootprintAndRecenter, disjoint_footprint_covers_both_shapes)
{
  PredictedObject object = make_box_object_with_path();
  // Pathological input: the footprint (x in [5, 6]) does not touch the body box (x in [-2, 2]).
  // The convex hull of the two disjoint shapes still covers both with a single ring, so neither is
  // dropped.
  object.shape.footprint.points.push_back(make_point32(5.0, -0.5));
  object.shape.footprint.points.push_back(make_point32(6.0, -0.5));
  object.shape.footprint.points.push_back(make_point32(6.0, 0.5));
  object.shape.footprint.points.push_back(make_point32(5.0, 0.5));

  autoware::map_based_prediction::utils::expandShapeToFootprintAndRecenter(object);

  // AABB union: x in [-2, 6] -> length 8, center offset +2; y unchanged.
  EXPECT_DOUBLE_EQ(object.shape.dimensions.x, 8.0);
  EXPECT_DOUBLE_EQ(object.shape.dimensions.y, 2.0);

  // Re-expressed about the new center (shifted by -2 in x): the footprint must still span both the
  // box (down to x = -4) and the far footprint (up to x = 4) - nothing dropped.
  ASSERT_FALSE(object.shape.footprint.points.empty());
  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  for (const auto & point : object.shape.footprint.points) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
  }
  EXPECT_FLOAT_EQ(min_x, -4.0f);
  EXPECT_FLOAT_EQ(max_x, 4.0f);
}

TEST(PathGenerator, test_generatePathToTargetPoint)
{
  // Generate Path generator
  const double prediction_sampling_time_interval = 0.5;
  const double min_crosswalk_user_velocity = 0.1;
  const autoware::map_based_prediction::PathGenerator path_generator =
    autoware::map_based_prediction::PathGenerator(
      prediction_sampling_time_interval, min_crosswalk_user_velocity);

  // Generate dummy object
  TrackedObject tracked_object = generate_static_object(ObjectClassification::CAR);

  // Generate target point
  Eigen::Vector2d target_point;
  target_point << 0.0, 0.0;

  // Generate predicted path
  const PredictedPath predicted_path =
    path_generator.generatePathToTargetPoint(tracked_object, target_point);

  // Check
  EXPECT_FALSE(predicted_path.path.empty());
  EXPECT_EQ(predicted_path.path[0].position.x, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.y, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.z, 0.0);
}

// ---------------------------------------------------------------------------
// Center-anchored combined model: forward heading integration in
// convertToPredictedPath. The center rides the Frenet path; the heading relaxes
// toward the center velocity direction with a lag set by the rear lever arm.
// These tests pin the behavior that replaced the old, divergent post-pass
// (applyBicycleYawToCenterPath).
// ---------------------------------------------------------------------------
namespace
{
using autoware::map_based_prediction::convertToPredictedPath;
using autoware::map_based_prediction::FrenetPath;
using autoware::map_based_prediction::FrenetPoint;
using autoware::map_based_prediction::PosePath;

TrackedObject make_box_object_with_yaw(const double yaw, const double x = 0.0, const double y = 0.0)
{
  TrackedObject object = generate_static_object(ObjectClassification::CAR);
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  object.shape.dimensions.z = 1.5;
  object.kinematics.pose_with_covariance.pose = make_pose(x, y, yaw);
  return object;
}

// Straight reference path of `count` poses spaced `step` apart along +x, all lateral offsets zero.
std::pair<PosePath, FrenetPath> make_straight_inputs(const size_t count, const double step)
{
  PosePath ref_path;
  FrenetPath frenet_path;
  for (size_t i = 0; i < count; ++i) {
    ref_path.push_back(make_pose(step * static_cast<double>(i), 0.0, 0.0));
    frenet_path.push_back(FrenetPoint{});  // d = 0 -> center stays on the reference path
  }
  return {ref_path, frenet_path};
}
}  // namespace

TEST(ConvertToPredictedPath, first_point_keeps_object_pose)
{
  TrackedObject object = make_box_object_with_yaw(0.9, 3.0, -2.0);
  const auto [ref_path, frenet_path] = make_straight_inputs(5, 2.0);

  const auto path = convertToPredictedPath(object, frenet_path, ref_path, 0.5, 1.0);

  ASSERT_FALSE(path.path.empty());
  EXPECT_DOUBLE_EQ(path.path[0].position.x, 3.0);
  EXPECT_DOUBLE_EQ(path.path[0].position.y, -2.0);
  EXPECT_NEAR(tf2::getYaw(path.path[0].orientation), 0.9, 1e-9);
}

TEST(ConvertToPredictedPath, high_speed_straight_heading_does_not_diverge)
{
  // The divergence repro: high speed (5 m chords ~ 10 m/s @ 0.5 s) + an initial heading error.
  // The old inversion-based post-pass oscillated to +/- 100 deg here; the integrator must decay.
  const double init_yaw = 0.2;
  TrackedObject object = make_box_object_with_yaw(init_yaw);
  const auto [ref_path, frenet_path] = make_straight_inputs(21, 5.0);
  const double rear_lever_arm = 1.0;

  const auto path = convertToPredictedPath(object, frenet_path, ref_path, 0.5, rear_lever_arm);

  ASSERT_EQ(path.path.size(), ref_path.size());
  EXPECT_NEAR(tf2::getYaw(path.path.front().orientation), init_yaw, 1e-9);
  double prev_abs = std::abs(init_yaw);
  for (size_t i = 1; i < path.path.size(); ++i) {
    const double yaw = tf2::getYaw(path.path[i].orientation);
    EXPECT_LE(std::abs(yaw), std::abs(init_yaw) + 1e-9);  // stays within the initial band
    EXPECT_LE(std::abs(yaw), prev_abs + 1e-9);            // monotonic decay toward the tangent
    prev_abs = std::abs(yaw);
  }
  EXPECT_NEAR(tf2::getYaw(path.path.back().orientation), 0.0, 1e-2);
}

TEST(ConvertToPredictedPath, zero_lever_arm_uses_azimuth_heading)
{
  // L -> 0 (non-bbox / tiny shapes): the relaxation gain saturates to 1, so the heading equals the
  // segment azimuth exactly -- bit-for-bit parity with the pre-bicycle behavior.
  TrackedObject object = make_box_object_with_yaw(0.5);
  const std::vector<std::pair<double, double>> pts = {
    {0.0, 0.0}, {1.0, 0.2}, {1.8, 0.8}, {2.2, 1.7}, {2.3, 2.7}};
  PosePath ref_path;
  FrenetPath frenet_path;
  for (const auto & [x, y] : pts) {
    ref_path.push_back(make_pose(x, y, 0.0));
    frenet_path.push_back(FrenetPoint{});
  }

  const auto path = convertToPredictedPath(object, frenet_path, ref_path, 0.5, 0.0);

  ASSERT_EQ(path.path.size(), ref_path.size());
  for (size_t i = 1; i < path.path.size(); ++i) {
    const double azimuth =
      autoware_utils::calc_azimuth_angle(path.path[i - 1].position, path.path[i].position);
    EXPECT_NEAR(tf2::getYaw(path.path[i].orientation), azimuth, 1e-9);
  }
}

TEST(ConvertToPredictedPath, idle_object_keeps_heading)
{
  // Zero chord between consecutive centers (object not advancing): heading frozen, no NaN, no
  // azimuth-of-zero-vector garbage.
  TrackedObject object = make_box_object_with_yaw(0.7);
  const PosePath ref_path(5, make_pose(0.0, 0.0, 0.0));
  const FrenetPath frenet_path(5, FrenetPoint{});

  const auto path = convertToPredictedPath(object, frenet_path, ref_path, 0.5, 1.0);

  ASSERT_EQ(path.path.size(), 5u);
  for (const auto & pose : path.path) {
    const double yaw = tf2::getYaw(pose.orientation);
    EXPECT_FALSE(std::isnan(yaw));
    EXPECT_NEAR(yaw, 0.7, 1e-9);
  }
}

TEST(ConvertToPredictedPath, low_speed_heading_no_overshoot)
{
  // Small chords (~L) with an initial heading error: heading approaches the straight-path tangent
  // (0) monotonically and never overshoots past it.
  const double init_yaw = 0.4;
  TrackedObject object = make_box_object_with_yaw(init_yaw);
  const auto [ref_path, frenet_path] = make_straight_inputs(30, 1.0);

  const auto path = convertToPredictedPath(object, frenet_path, ref_path, 0.5, 1.0);

  ASSERT_EQ(path.path.size(), ref_path.size());
  double prev = init_yaw;
  for (size_t i = 1; i < path.path.size(); ++i) {
    const double yaw = tf2::getYaw(path.path[i].orientation);
    EXPECT_GE(yaw, -1e-9);        // never overshoots below the tangent
    EXPECT_LE(yaw, prev + 1e-9);  // monotonic decrease
    prev = yaw;
  }
}

TEST(ConvertToPredictedPath, cornering_heading_lags_tangent)
{
  // Constant-curvature left turn: in steady state the body heading lags the path tangent by a
  // positive, bounded, roughly constant angle -- the emergent bicycle slip.
  TrackedObject object = make_box_object_with_yaw(0.0);
  const double radius = 20.0;
  const double d_phi = 0.05;  // [rad] per step
  const size_t count = 40;
  PosePath ref_path;
  FrenetPath frenet_path;
  for (size_t i = 0; i < count; ++i) {
    const double phi = d_phi * static_cast<double>(i);
    ref_path.push_back(make_pose(radius * std::sin(phi), radius * (1.0 - std::cos(phi)), 0.0));
    frenet_path.push_back(FrenetPoint{});
  }

  const auto path = convertToPredictedPath(object, frenet_path, ref_path, 0.5, 1.5);

  ASSERT_EQ(path.path.size(), count);
  double prev_lag = 0.0;
  for (size_t i = count / 2; i < count; ++i) {
    const double tangent =
      autoware_utils::calc_azimuth_angle(path.path[i - 1].position, path.path[i].position);
    const double lag =
      autoware_utils::normalize_radian(tangent - tf2::getYaw(path.path[i].orientation));
    EXPECT_GT(lag, 0.0);          // heading is behind the tangent on a left turn
    EXPECT_LT(lag, d_phi * 3.0);  // bounded and small
    if (i > count / 2) {
      EXPECT_NEAR(lag, prev_lag, 5e-3);  // ~constant -> steady state reached
    }
    prev_lag = lag;
  }
}
