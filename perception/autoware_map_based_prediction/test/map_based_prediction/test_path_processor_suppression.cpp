// Copyright 2021 TIER IV, Inc.
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

#include "autoware/map_based_prediction/predictor_vehicle/path_processing.hpp"

#include <autoware/agnocast_wrapper/node.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction
{
class PathProcessorSuppressionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<autoware::agnocast_wrapper::Node>("path_processor_suppression_test");
    processor_ = std::make_unique<PathProcessor>(*node_);
    params_.prediction_time_horizon = 3.0;
    params_.max_lateral_accel = 0.01;
    params_.min_acceleration_before_curve = -0.01;
    processor_->setParams(params_);
    object_.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
    object_.kinematics.orientation_availability =
      autoware_perception_msgs::msg::TrackedObjectKinematics::AVAILABLE;
    object_.kinematics.twist_with_covariance.twist.linear.x = 10.0;
    object_.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object_.shape.dimensions.y = 1.0;
  }

  LaneletPathWithPathInfo candidate(
    const lanelet::Id id, const Maneuver maneuver, const lanelet::Id origin = 1)
  {
    // Distinct IDs form distinct conversion-cache keys; geometry is injected below.
    const lanelet::Lanelet lane(id, lanelet::LineString3d(id + 1), lanelet::LineString3d(id + 2));
    PredictedRefPath info{};
    info.maneuver = maneuver;
    info.origin_lanelet_id = origin;
    info.probability = maneuver == Maneuver::LANE_FOLLOW ? 1.0F : 0.3F;
    info.speed_limit = 20.0;
    return {lanelet::routing::LaneletPath({lane}), info};
  }

  LaneletPathWithPathInfo mappedCandidate(
    const lanelet::Id id, const Maneuver maneuver, const double end, const double y)
  {
    lanelet::Lanelet lane(
      id,
      lanelet::LineString3d(
        id + 1,
        {lanelet::Point3d(id + 3, 0.0, y + 1.0, 0.0), lanelet::Point3d(id + 4, end, y + 1.0, 0.0)}),
      lanelet::LineString3d(
        id + 2, {lanelet::Point3d(id + 5, 0.0, y - 1.0, 0.0),
                 lanelet::Point3d(id + 6, end, y - 1.0, 0.0)}));
    lane.setCenterline(
      lanelet::LineString3d(
        id + 7, {lanelet::Point3d(id + 8, 0.0, y, 0.0), lanelet::Point3d(id + 9, end / 2.0, y, 0.0),
                 lanelet::Point3d(id + 10, end, y, 0.0)}));
    lane.attributes()[lanelet::AttributeName::Subtype] = "road";
    lane.attributes()[lanelet::AttributeName::Location] = "urban";
    map_->add(lane);
    auto result = candidate(id, maneuver);
    result.first = lanelet::routing::LaneletPath({lane});
    return result;
  }

  void installMap()
  {
    std::shared_ptr<lanelet::traffic_rules::TrafficRules> rules =
      lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    processor_->setLaneletMap(map_, lanelet::routing::RoutingGraph::build(*map_, *rules), rules);
  }

  PosePath reference(const bool curve)
  {
    PosePath path;
    for (int i = 0; i <= 200; ++i) {
      const double arc = i * 0.5;
      const double yaw = curve ? arc / 30.0 : 0.0;
      geometry_msgs::msg::Pose pose;
      pose.position.x = curve ? 30.0 * std::sin(yaw) : arc;
      pose.position.y = curve ? 30.0 * (1.0 - std::cos(yaw)) : 0.0;
      pose.orientation.z = std::sin(yaw / 2.0);
      pose.orientation.w = std::cos(yaw / 2.0);
      path.push_back(pose);
    }
    return path;
  }

  void cache(
    const LaneletPathWithPathInfo & candidate, const PosePath & path, const double width = 4.0)
  {
    processor_->lru_cache_of_convert_path_type_.put(candidate.first, std::make_pair(path, width));
  }

  std::vector<PredictedRefPath> convert(
    const TrackedObject & object, const std::vector<LaneletPathWithPathInfo> & refs)
  {
    return processor_->convertPredictedReferencePath(object, refs);
  }

  bool accepted(const LaneletPathWithPathInfo & candidate)
  {
    const auto refs = processor_->convertPredictedReferencePath(object_, {candidate});
    if (refs.empty()) return false;
    const auto & ref = refs.front();
    const auto path = processor_->path_generator_->generatePathForOnLaneVehicle(
      object_, ref.path, params_.prediction_time_horizon, params_.lateral_control_time_horizon,
      ref.width, ref.speed_limit);
    return !path.path.empty() &&
           processor_->isLateralAccelerationConstraintSatisfied(
             processor_->toTrajectoryPoints(path, 10.0), params_.prediction_sampling_time_interval);
  }

  std::optional<PredictedObject> run(
    const LaneletPathWithPathInfo & center, const LaneletPathWithPathInfo & held)
  {
    return processor_->predictOnLane(object_, {}, {center}, {held}, nullptr, &processing_passes_);
  }

  std::optional<PredictedObject> processorFallback(const LaneletPathWithPathInfo & center)
  {
    return processor_->predictOnLane(object_, {}, {center}, {}, nullptr);
  }

  std::optional<PredictedObject> runAll(
    const std::vector<LaneletPathWithPathInfo> & lanelet_ref_paths,
    const std::vector<LaneletPathWithPathInfo> & suppressed)
  {
    return processor_->predictOnLane(
      object_, {}, lanelet_ref_paths, suppressed, nullptr, &processing_passes_);
  }

  lanelet::LaneletMapPtr map_{std::make_shared<lanelet::LaneletMap>()};
  std::shared_ptr<autoware::agnocast_wrapper::Node> node_;
  std::unique_ptr<PathProcessor> processor_;
  PathProcessor::Params params_;
  TrackedObject object_;
  size_t processing_passes_{0};
};

TEST_F(PathProcessorSuppressionTest, OnePointMappedCenterRestoresLaneChange)
{
  const auto center = mappedCandidate(100, Maneuver::LANE_FOLLOW, 20.0, 0.0);
  const auto held = mappedCandidate(200, Maneuver::LEFT_LANE_CHANGE, 100.0, 1.0);
  installMap();
  object_.kinematics.pose_with_covariance.pose.position.x = 20.6;
  params_.lane_change_path_suppression_keep_when_lane_ends = false;
  params_.check_lateral_acceleration_constraints = false;
  processor_->setParams(params_);
  const auto refs = convert(object_, {center});
  ASSERT_EQ(refs.size(), 1U);
  ASSERT_EQ(refs.front().path.size(), 1U);
  const auto lc_refs = convert(object_, {held});
  ASSERT_EQ(lc_refs.size(), 1U);
  ASSERT_GT(lc_refs.front().path.size(), 1U);
  const auto result = run(center, held);
  ASSERT_TRUE(result.has_value());
  // The one-point center still yields the generator's straight path as before, and the held LC
  // is restored because no map-following center survived.
  ASSERT_EQ(result->kinematics.predicted_paths.size(), 2U);
  EXPECT_DOUBLE_EQ(result->kinematics.predicted_paths[0].path.back().position.y, 0.0);
  EXPECT_GT(result->kinematics.predicted_paths[1].path.back().position.y, 0.1);
  EXPECT_EQ(processing_passes_, 2U);
}

TEST_F(PathProcessorSuppressionTest, OnePointCenterKeepsStraightPathWhenNothingIsSuppressed)
{
  // enable=false (or a detected LC) hands all candidates over unsuppressed: the one-point center
  // produces the straight path next to the LC path, exactly as before the suppression feature.
  const auto center = mappedCandidate(100, Maneuver::LANE_FOLLOW, 20.0, 0.0);
  const auto lc = mappedCandidate(200, Maneuver::LEFT_LANE_CHANGE, 100.0, 1.0);
  installMap();
  object_.kinematics.pose_with_covariance.pose.position.x = 20.6;
  params_.check_lateral_acceleration_constraints = false;
  processor_->setParams(params_);
  const auto result = runAll({center, lc}, {});
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->kinematics.predicted_paths.size(), 2U);
  EXPECT_DOUBLE_EQ(result->kinematics.predicted_paths[0].path.back().position.y, 0.0);
  EXPECT_GT(result->kinematics.predicted_paths[0].path.back().position.x, 40.0);
  EXPECT_GT(result->kinematics.predicted_paths[1].path.back().position.y, 0.1);
  EXPECT_EQ(processing_passes_, 1U);
}

TEST_F(PathProcessorSuppressionTest, OnePointCenterWithoutLaneChangeUsesStraightFallback)
{
  const auto center = mappedCandidate(100, Maneuver::LANE_FOLLOW, 20.0, 0.0);
  installMap();
  object_.kinematics.pose_with_covariance.pose.position.x = 20.6;
  const auto result = processorFallback(center);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->kinematics.predicted_paths.size(), 1U);
  EXPECT_GT(result->kinematics.predicted_paths.front().path.back().position.x, 40.0);
  EXPECT_DOUBLE_EQ(result->kinematics.predicted_paths.front().path.back().position.y, 0.0);
}

TEST_F(PathProcessorSuppressionTest, RestoresLaneChangeWhenCenterFailsLateralAcceleration)
{
  const auto center = candidate(100, Maneuver::LANE_FOLLOW);
  const auto held = candidate(200, Maneuver::LEFT_LANE_CHANGE);
  cache(center, reference(true));
  cache(held, reference(false));
  ASSERT_FALSE(accepted(center));
  ASSERT_TRUE(accepted(held));
  const auto result = run(center, held);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->kinematics.predicted_paths.size(), 1U);
  const auto & path = result->kinematics.predicted_paths.front();
  EXPECT_GT(path.path.back().position.x, 20.0);
  EXPECT_NEAR(path.path.back().position.y, 0.0, 1e-6);
  EXPECT_FLOAT_EQ(path.confidence, 1.0F);
  EXPECT_EQ(processing_passes_, 2U);
}

TEST_F(PathProcessorSuppressionTest, RestoresLaneChangeWhenCenterConversionIsEmpty)
{
  const auto center = candidate(100, Maneuver::LANE_FOLLOW);
  const auto held = candidate(200, Maneuver::LEFT_LANE_CHANGE);
  cache(center, {});
  cache(held, reference(false));
  ASSERT_FALSE(accepted(center));
  ASSERT_TRUE(accepted(held));
  auto offset_reference = reference(false);
  for (auto & pose : offset_reference) pose.position.y = 1.0;
  // Width 2m gives zero backlash for the 1m-wide object, forcing convergence to y=1.
  // With width 4m, the 1m backlash allows y=0 even when the held reference is used.
  cache(held, offset_reference, 2.0);
  const auto restored_refs = convert(object_, {held});
  ASSERT_EQ(restored_refs.size(), 1U);
  EXPECT_DOUBLE_EQ(restored_refs.front().path.back().position.y, 1.0);
  EXPECT_DOUBLE_EQ(restored_refs.front().width, 2.0);
  // A lateral endpoint distinguishes restored LC from the y=0 straight fallback.
  params_.check_lateral_acceleration_constraints = false;
  processor_->setParams(params_);
  const auto result = run(center, held);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->kinematics.predicted_paths.size(), 1U);
  EXPECT_GT(result->kinematics.predicted_paths.front().path.back().position.y, 0.1);
  EXPECT_EQ(processing_passes_, 2U);
}

TEST_F(PathProcessorSuppressionTest, RestoresOnlyHeldLaneChangesOfOriginsWithoutSurvivingCenter)
{
  // Origin 1: center survives, so its held LC stays suppressed.
  // Origin 2: center conversion is empty, so its held LC is restored.
  const auto center_a = candidate(100, Maneuver::LANE_FOLLOW, 1);
  const auto held_a = candidate(200, Maneuver::LEFT_LANE_CHANGE, 1);
  const auto center_b = candidate(300, Maneuver::LANE_FOLLOW, 2);
  const auto held_b = candidate(400, Maneuver::RIGHT_LANE_CHANGE, 2);
  cache(center_a, reference(false));
  cache(center_b, {});
  auto offset_reference = reference(false);
  for (auto & pose : offset_reference) pose.position.y = 1.0;
  cache(held_a, offset_reference, 2.0);
  cache(held_b, offset_reference, 2.0);
  params_.check_lateral_acceleration_constraints = false;
  processor_->setParams(params_);
  const auto result = runAll({center_a, center_b}, {held_a, held_b});
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->kinematics.predicted_paths.size(), 2U);
  EXPECT_NEAR(result->kinematics.predicted_paths[0].path.back().position.y, 0.0, 1e-6);
  EXPECT_GT(result->kinematics.predicted_paths[1].path.back().position.y, 0.1);
  EXPECT_FLOAT_EQ(result->kinematics.predicted_paths[0].confidence, 1.0F / 1.3F);
  EXPECT_FLOAT_EQ(result->kinematics.predicted_paths[1].confidence, 0.3F / 1.3F);
  EXPECT_EQ(processing_passes_, 2U);
}

TEST_F(PathProcessorSuppressionTest, SurvivingCenterDoesNotConvertHeldLaneChange)
{
  const auto center = candidate(100, Maneuver::LANE_FOLLOW);
  const auto held = candidate(200, Maneuver::LEFT_LANE_CHANGE);
  cache(center, reference(false));
  ASSERT_TRUE(accepted(center));
  // Any held conversion would access the unset routing graph. Count the processing passes.
  const auto result = run(center, held);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->kinematics.predicted_paths.size(), 1U);
  EXPECT_EQ(processing_passes_, 1U);
}
}  // namespace autoware::map_based_prediction
