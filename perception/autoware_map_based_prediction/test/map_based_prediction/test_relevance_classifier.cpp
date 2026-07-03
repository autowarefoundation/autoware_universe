// Copyright 2026 The Autoware Foundation
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

#include "autoware/map_based_prediction/relevance_classifier.hpp"

#include <tf2/LinearMath/Quaternion.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace
{

using autoware::map_based_prediction::computeCorridorDistance;
using autoware::map_based_prediction::Relevance;
using autoware::map_based_prediction::RelevanceClassifier;
using autoware::map_based_prediction::RelevanceParams;
using autoware_perception_msgs::msg::TrackedObject;

geometry_msgs::msg::Point makePoint(const double x, const double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  return point;
}

// A straight corridor along the x axis: (0,0) -> (100,0).
std::vector<geometry_msgs::msg::Point> makeStraightTrajectory()
{
  std::vector<geometry_msgs::msg::Point> points;
  for (int i = 0; i <= 10; ++i) {
    points.push_back(makePoint(10.0 * i, 0.0));
  }
  return points;
}

TrackedObject makeObject(
  const double x, const double y, const double yaw, const double velocity, const uint8_t id_seed)
{
  TrackedObject object;
  object.object_id.uuid.fill(id_seed);
  object.kinematics.pose_with_covariance.pose.position.x = x;
  object.kinematics.pose_with_covariance.pose.position.y = y;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  object.kinematics.pose_with_covariance.pose.orientation.x = quaternion.x();
  object.kinematics.pose_with_covariance.pose.orientation.y = quaternion.y();
  object.kinematics.pose_with_covariance.pose.orientation.z = quaternion.z();
  object.kinematics.pose_with_covariance.pose.orientation.w = quaternion.w();
  object.kinematics.twist_with_covariance.twist.linear.x = velocity;
  return object;
}

RelevanceParams makeEnabledParams()
{
  RelevanceParams params;
  params.enable = true;
  params.ego_trajectory_timeout = 1.0;
  params.always_relevant_radius = 20.0;
  params.lateral_margin_base = 3.0;
  params.lateral_margin_rate = 0.1;
  params.time_to_corridor_threshold = 5.0;
  params.demote_frame_count = 3;
  params.low_fidelity_time_horizon = 3.0;
  return params;
}

}  // namespace

TEST(ComputeCorridorDistance, degenerate_polyline_returns_nullopt)
{
  const std::vector<geometry_msgs::msg::Point> empty;
  EXPECT_FALSE(computeCorridorDistance(empty, makePoint(0.0, 0.0)).has_value());

  const std::vector<geometry_msgs::msg::Point> single{makePoint(0.0, 0.0)};
  EXPECT_FALSE(computeCorridorDistance(single, makePoint(0.0, 0.0)).has_value());
}

TEST(ComputeCorridorDistance, lateral_distance_and_arc_length)
{
  const auto polyline = makeStraightTrajectory();

  // 5 m beside the corridor at x = 30.
  const auto result = computeCorridorDistance(polyline, makePoint(30.0, 5.0));
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->lateral_distance, 5.0, 1e-6);
  EXPECT_NEAR(result->arc_length, 30.0, 1e-6);
  EXPECT_NEAR(result->nearest_point.x, 30.0, 1e-6);
  EXPECT_NEAR(result->nearest_point.y, 0.0, 1e-6);

  // Beyond the far end: clamped to the last point.
  const auto beyond = computeCorridorDistance(polyline, makePoint(110.0, 0.0));
  ASSERT_TRUE(beyond.has_value());
  EXPECT_NEAR(beyond->lateral_distance, 10.0, 1e-6);
  EXPECT_NEAR(beyond->arc_length, 100.0, 1e-6);
}

TEST(RelevanceClassifier, disabled_returns_high)
{
  auto params = makeEnabledParams();
  params.enable = false;
  RelevanceClassifier classifier(params);
  classifier.setEgoData(makeStraightTrajectory(), makePoint(0.0, 0.0), 0.0);

  const auto far_object = makeObject(50.0, 100.0, 0.0, 0.0, 1);
  EXPECT_EQ(classifier.classify(far_object, 0.0), Relevance::HIGH);
}

TEST(RelevanceClassifier, stale_trajectory_returns_high)
{
  RelevanceClassifier classifier(makeEnabledParams());
  classifier.setEgoData(makeStraightTrajectory(), makePoint(0.0, 0.0), 0.0);

  const auto far_object = makeObject(50.0, 100.0, 0.0, 0.0, 1);
  // 2.0 s after the trajectory stamp > ego_trajectory_timeout (1.0 s).
  EXPECT_EQ(classifier.classify(far_object, 2.0), Relevance::HIGH);
}

TEST(RelevanceClassifier, no_trajectory_returns_high)
{
  RelevanceClassifier classifier(makeEnabledParams());
  const auto far_object = makeObject(50.0, 100.0, 0.0, 0.0, 1);
  EXPECT_EQ(classifier.classify(far_object, 0.0), Relevance::HIGH);
}

TEST(RelevanceClassifier, object_near_ego_is_high)
{
  RelevanceClassifier classifier(makeEnabledParams());
  classifier.setEgoData(makeStraightTrajectory(), makePoint(0.0, 0.0), 0.0);

  // 100 m beside the corridor but only 15 m from ego (within always_relevant_radius).
  const auto object = makeObject(0.0, 15.0, 0.0, 0.0, 1);
  EXPECT_EQ(classifier.classify(object, 0.0), Relevance::HIGH);
}

TEST(RelevanceClassifier, object_inside_corridor_is_high)
{
  RelevanceClassifier classifier(makeEnabledParams());
  classifier.setEgoData(makeStraightTrajectory(), makePoint(0.0, 0.0), 0.0);

  // At arc length 50 the corridor half width is 3.0 + 0.1 * 50 = 8.0 m.
  const auto object = makeObject(50.0, 7.0, 0.0, 0.0, 1);
  EXPECT_EQ(classifier.classify(object, 0.0), Relevance::HIGH);
}

TEST(RelevanceClassifier, approaching_object_is_high)
{
  RelevanceClassifier classifier(makeEnabledParams());
  classifier.setEgoData(makeStraightTrajectory(), makePoint(0.0, 0.0), 0.0);

  // 30 m beside the corridor, heading straight at it (-y) with 10 m/s:
  // time to corridor = 3 s < threshold 5 s.
  const auto object = makeObject(50.0, 30.0, -M_PI_2, 10.0, 1);
  EXPECT_EQ(classifier.classify(object, 0.0), Relevance::HIGH);
}

TEST(RelevanceClassifier, distant_receding_object_demotes_after_hysteresis)
{
  RelevanceClassifier classifier(makeEnabledParams());
  classifier.setEgoData(makeStraightTrajectory(), makePoint(0.0, 0.0), 0.0);

  // 50 m beside the corridor, moving away from it (+y).
  const auto object = makeObject(50.0, 50.0, M_PI_2, 5.0, 1);

  // demote_frame_count = 3: the first two frames stay HIGH, the third demotes.
  EXPECT_EQ(classifier.classify(object, 0.0), Relevance::HIGH);
  EXPECT_EQ(classifier.classify(object, 0.1), Relevance::HIGH);
  EXPECT_EQ(classifier.classify(object, 0.2), Relevance::LOW);
  EXPECT_EQ(classifier.classify(object, 0.3), Relevance::LOW);
}

TEST(RelevanceClassifier, promotion_is_immediate)
{
  RelevanceClassifier classifier(makeEnabledParams());
  classifier.setEgoData(makeStraightTrajectory(), makePoint(0.0, 0.0), 0.0);

  const auto far_object = makeObject(50.0, 50.0, M_PI_2, 5.0, 1);
  classifier.classify(far_object, 0.0);
  classifier.classify(far_object, 0.1);
  ASSERT_EQ(classifier.classify(far_object, 0.2), Relevance::LOW);

  // The same object enters the corridor: promoted immediately.
  const auto near_object = makeObject(50.0, 2.0, M_PI_2, 5.0, 1);
  EXPECT_EQ(classifier.classify(near_object, 0.3), Relevance::HIGH);
}

TEST(RelevanceClassifier, history_cleanup_resets_hysteresis)
{
  RelevanceClassifier classifier(makeEnabledParams());
  classifier.setEgoData(makeStraightTrajectory(), makePoint(0.0, 0.0), 0.0);

  const auto object = makeObject(50.0, 50.0, M_PI_2, 5.0, 1);
  classifier.classify(object, 0.0);
  classifier.classify(object, 0.1);
  ASSERT_EQ(classifier.classify(object, 0.2), Relevance::LOW);

  // After the entry is removed, demotion must be re-earned.
  classifier.removeOldHistory(10.0, 2.0);
  classifier.setEgoData(makeStraightTrajectory(), makePoint(0.0, 0.0), 10.0);
  EXPECT_EQ(classifier.classify(object, 10.0), Relevance::HIGH);
}
