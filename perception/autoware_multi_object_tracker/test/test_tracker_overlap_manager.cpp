// Copyright 2026 TIER IV, inc.
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

#include "autoware/multi_object_tracker/merger/detail/redundancy_check.hpp"
#include "autoware/multi_object_tracker/merger/tracker_overlap_manager.hpp"
#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/object_model/shapes_iou.hpp"
#include "autoware/multi_object_tracker/tracker/trackers/polygon_tracker.hpp"
#include "autoware/multi_object_tracker/tracker/trackers/vehicle_tracker.hpp"
#include "autoware/multi_object_tracker/types.hpp"
#include "autoware/multi_object_tracker/uncertainty/uncertainty_processor.hpp"
#include "test_bench.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <list>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace
{

namespace mot = autoware::multi_object_tracker;
namespace mot_detail = autoware::multi_object_tracker::detail;
using std::chrono_literals::operator""ms;

constexpr int kSpinStrong = 12;
constexpr int kSpinMedium = 6;
constexpr int kSpinWeak = 3;

rclcpp::Time baseTime()
{
  return rclcpp::Time(1000000000LL, RCL_ROS_TIME);
}

mot::types::DynamicObject makeBaseObject(
  const double x, const double y, const mot::classes::Label label, const rclcpp::Time & time)
{
  mot::types::DynamicObject obj;
  obj.time = time;
  obj.classification = {{label, 1.0F}};
  obj.pose.position.x = x;
  obj.pose.position.y = y;
  obj.pose.position.z = 0.0;
  obj.pose.orientation.w = 1.0;
  obj.pose_covariance.fill(0.0);
  obj.twist_covariance.fill(0.0);
  obj.kinematics.has_position_covariance = false;
  obj.kinematics.has_twist = false;
  obj.kinematics.has_twist_covariance = false;
  obj.kinematics.orientation_availability = mot::types::OrientationAvailability::AVAILABLE;
  obj.existence_probability = 0.9;
  obj.channel_index = 0;
  return obj;
}

mot::types::DynamicObject withUncertainty(const mot::types::DynamicObject & obj)
{
  mot::types::DynamicObjectList list;
  list.channel_index = obj.channel_index;
  list.objects = {obj};
  return mot::uncertainty::modelUncertainty(list).objects.front();
}

mot::types::DynamicObject makeBboxObject(
  const double x, const double y, const double length, const double width,
  const mot::classes::Label label, const rclcpp::Time & time)
{
  auto obj = makeBaseObject(x, y, label, time);
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions.x = length;
  obj.shape.dimensions.y = width;
  obj.shape.dimensions.z = 1.8;
  obj.area = length * width;
  return withUncertainty(obj);
}

mot::types::DynamicObject makePolygonObject(
  const double x, const double y, const double half_size, const rclcpp::Time & time)
{
  auto obj = makeBaseObject(x, y, mot::classes::Label::UNKNOWN, time);
  obj.shape.type = autoware_perception_msgs::msg::Shape::POLYGON;
  obj.shape.dimensions.z = 1.5;
  for (const auto & [px, py] : std::vector<std::pair<double, double>>{
         {half_size, half_size},
         {-half_size, half_size},
         {-half_size, -half_size},
         {half_size, -half_size}}) {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(px);
    point.y = static_cast<float>(py);
    point.z = 0.0;
    obj.shape.footprint.points.push_back(point);
  }
  obj.area = (2.0 * half_size) * (2.0 * half_size);
  return withUncertainty(obj);
}

// Feed the tracker n_updates static re-detections so measurement count grows and the position
// covariance converges (confidence builds up).
void spinUp(
  const std::shared_ptr<mot::Tracker> & tracker, const mot::types::DynamicObject & obj,
  const rclcpp::Time & start_time, const int n_updates, const mot::types::InputChannel & channel)
{
  rclcpp::Time time = start_time;
  for (int k = 0; k < n_updates; ++k) {
    time += rclcpp::Duration(100ms);
    tracker->predict(time);
    auto measurement = obj;
    measurement.time = time;
    tracker->updateWithMeasurement(measurement, time, channel);
  }
}

std::shared_ptr<mot::Tracker> makeVehicleTracker(
  const mot::types::DynamicObject & obj, const rclcpp::Time & time, const int n_updates,
  const mot::types::InputChannel & channel)
{
  auto tracker =
    std::make_shared<mot::VehicleTracker>(mot::object_model::normal_vehicle, time, obj);
  tracker->initializeExistenceProbabilities(channel.index, obj.existence_probability);
  spinUp(tracker, obj, time, n_updates, channel);
  return tracker;
}

std::shared_ptr<mot::Tracker> makePolygonTracker(
  const mot::types::DynamicObject & obj, const rclcpp::Time & time, const int n_updates,
  const mot::types::InputChannel & channel)
{
  mot::PolygonTrackerConfig polygon_config;
  polygon_config.enable_velocity_estimation = false;
  // enable_motion_output left empty => motion output disabled for all labels
  auto tracker = std::make_shared<mot::PolygonTracker>(time, obj, polygon_config);
  tracker->initializeExistenceProbabilities(channel.index, obj.existence_probability);
  spinUp(tracker, obj, time, n_updates, channel);
  return tracker;
}

class TrackerOverlapManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    channel_ = createInputChannelsConfig().front();
    config_ = createTrackerOverlapManagerConfig();
    manager_ = std::make_unique<mot::TrackerOverlapManager>(config_);
  }

  void runMerge(std::list<std::shared_ptr<mot::Tracker>> & trackers, const rclcpp::Time & time)
  {
    manager_->merge(trackers, time, cache_, std::nullopt);
  }

  static rclcpp::Time mergeTime()
  {
    // Past the spin-up updates (kSpinStrong * 100ms)
    return baseTime() + rclcpp::Duration(100ms) * (kSpinStrong + 1);
  }

  mot::types::InputChannel channel_;
  mot::TrackerOverlapManagerConfig config_;
  mot::AdaptiveThresholdCache cache_;
  std::unique_ptr<mot::TrackerOverlapManager> manager_;
};

}  // namespace

TEST_F(TrackerOverlapManagerTest, PolygonOnlyPairsDoNotMerge)
{
  const auto time = baseTime();
  const auto obj_a = makePolygonObject(0.0, 0.0, 1.0, time);
  const auto obj_b = makePolygonObject(0.5, 0.0, 1.0, time);

  // Asymmetric spin counts so the covariance tier yields a clear winner (a covariance tie would
  // make the pair unmergeable regardless of the polygon-only rule).
  std::list<std::shared_ptr<mot::Tracker>> trackers{
    makePolygonTracker(obj_a, time, kSpinStrong, channel_),
    makePolygonTracker(obj_b, time, kSpinMedium, channel_)};

  // Sanity: the pair must be mergeable in every respect except the polygon-only rule, so the
  // surviving pair proves the prohibition (not a missing precondition).
  std::vector<mot::types::DynamicObject> exported(2);
  size_t idx = 0;
  for (const auto & tracker : trackers) {
    ASSERT_TRUE(tracker->isConfident(cache_, std::nullopt, mergeTime()));
    ASSERT_TRUE(tracker->getTrackedObject(mergeTime(), exported[idx]));
    ASSERT_EQ(exported[idx].shape.type, autoware_perception_msgs::msg::Shape::POLYGON);
    ++idx;
  }
  ASSERT_TRUE(
    mot_detail::isRedundant(
      exported[0], exported[1], mot::classes::Label::UNKNOWN, mot::classes::Label::UNKNOWN,
      trackers.front()->getKnownObjectProbability(), trackers.back()->getKnownObjectProbability(),
      config_));

  runMerge(trackers, mergeTime());

  // Heavily overlapping, but both carry only a polygon shape: merging is prohibited.
  EXPECT_EQ(trackers.size(), 2U);
}

TEST_F(TrackerOverlapManagerTest, BboxTrackerAbsorbsOverlappingPolygon)
{
  const auto time = baseTime();
  const auto car_obj = makeBboxObject(0.0, 0.0, 4.5, 1.8, mot::classes::Label::CAR, time);
  const auto fragment_obj = makePolygonObject(1.0, 0.0, 0.8, time);

  std::list<std::shared_ptr<mot::Tracker>> trackers{
    makePolygonTracker(fragment_obj, time, kSpinMedium, channel_),
    makeVehicleTracker(car_obj, time, kSpinStrong, channel_)};

  runMerge(trackers, mergeTime());

  ASSERT_EQ(trackers.size(), 1U);
  EXPECT_EQ(trackers.front()->getHighestProbLabel(), mot::classes::Label::CAR);
}

TEST_F(TrackerOverlapManagerTest, ClassifiedFragmentAbsorbedByContainment)
{
  const auto time = baseTime();
  // A classified fragment fully inside a much larger same-class object, with IoU below
  // min_known_object_removal_iou (0.1): plain IoU keeps both; only the containment criterion
  // absorbs the fragment. Both use the same tracker type so direction is decided by
  // confidence/covariance, not priority.
  const auto car_obj = makeBboxObject(0.0, 0.0, 10.0, 2.4, mot::classes::Label::CAR, time);
  const auto fragment_obj = makeBboxObject(3.0, 0.3, 1.5, 1.2, mot::classes::Label::CAR, time);

  auto fragment_tracker = makeVehicleTracker(fragment_obj, time, kSpinMedium, channel_);
  auto car_tracker = makeVehicleTracker(car_obj, time, kSpinStrong, channel_);

  // Sanity: the exported shapes (after the vehicle shape model's size limits and convergence)
  // must still be below the plain-IoU merge threshold, or this test would not distinguish the
  // containment criterion.
  mot::types::DynamicObject car_exported;
  mot::types::DynamicObject fragment_exported;
  ASSERT_TRUE(car_tracker->getTrackedObject(mergeTime(), car_exported));
  ASSERT_TRUE(fragment_tracker->getTrackedObject(mergeTime(), fragment_exported));
  ASSERT_LT(mot::shapes::get2dIoU(car_exported, fragment_exported, 1e-2), 0.1);

  std::list<std::shared_ptr<mot::Tracker>> trackers{fragment_tracker, car_tracker};
  runMerge(trackers, mergeTime());

  ASSERT_EQ(trackers.size(), 1U);
  EXPECT_EQ(trackers.front(), car_tracker);
}

TEST_F(TrackerOverlapManagerTest, StarMergeAbsorbsMultipleFragmentsInOneCycle)
{
  const auto time = baseTime();
  const auto car_obj = makeBboxObject(0.0, 0.0, 4.5, 1.8, mot::classes::Label::CAR, time);
  const auto fragment_front = makePolygonObject(1.2, 0.0, 0.7, time);
  const auto fragment_rear = makePolygonObject(-1.2, 0.0, 0.7, time);

  std::list<std::shared_ptr<mot::Tracker>> trackers{
    makePolygonTracker(fragment_front, time, kSpinMedium, channel_),
    makeVehicleTracker(car_obj, time, kSpinStrong, channel_),
    makePolygonTracker(fragment_rear, time, kSpinMedium, channel_)};

  runMerge(trackers, mergeTime());

  // Multiple losers may merge into the same winner within one cycle (star shape).
  ASSERT_EQ(trackers.size(), 1U);
  EXPECT_EQ(trackers.front()->getHighestProbLabel(), mot::classes::Label::CAR);
}

TEST_F(TrackerOverlapManagerTest, ChainDoesNotBridgeWithinOneCycle)
{
  const auto time = baseTime();
  // A <- B <- C chain: B overlaps A (IoU ~0.33), C overlaps B (IoU ~0.21), C barely
  // touches A (IoU ~0.03, not redundant). Bridging C into A through B would merge
  // all three in one cycle; the star-forest rule must leave two trackers.
  const auto obj_a = makeBboxObject(0.0, 0.0, 4.0, 2.0, mot::classes::Label::CAR, time);
  const auto obj_b = makeBboxObject(1.5, 0.0, 2.0, 2.0, mot::classes::Label::CAR, time);
  const auto obj_c = makeBboxObject(2.8, 0.0, 2.0, 2.0, mot::classes::Label::CAR, time);

  const auto tracker_a = makeVehicleTracker(obj_a, time, kSpinStrong, channel_);

  std::list<std::shared_ptr<mot::Tracker>> trackers{
    makeVehicleTracker(obj_c, time, kSpinWeak, channel_), tracker_a,
    makeVehicleTracker(obj_b, time, kSpinMedium, channel_)};

  runMerge(trackers, mergeTime());

  EXPECT_EQ(trackers.size(), 2U);
  EXPECT_TRUE(
    std::any_of(trackers.begin(), trackers.end(), [&](const auto & t) { return t == tracker_a; }));
}

TEST_F(TrackerOverlapManagerTest, FreshBboxTrackerOutranksStalePartialVehicle)
{
  const auto time = baseTime();

  // A vehicle tracker fed only partial (cluster, trust_extension=false) measurements: it never
  // takes the NORMAL path, so it is never "fully measured" and must not absorb a fresh bbox tracker
  // even though it is confident and long-lived.
  mot::types::InputChannel cluster_channel = channel_;
  cluster_channel.index = 1;
  cluster_channel.trust_extension = false;

  auto partial_obj = makeBboxObject(0.5, 0.0, 4.5, 1.8, mot::classes::Label::CAR, time);
  partial_obj.trust_extension = false;  // cluster-origin: not a trustworthy full box
  auto stale_partial = makeVehicleTracker(partial_obj, time, kSpinStrong, cluster_channel);

  // A fresh, fully-measured (trust_extension=true) bbox tracker updated up to just before merge.
  const auto fresh_obj = makeBboxObject(0.0, 0.0, 4.5, 1.8, mot::classes::Label::CAR, time);
  auto fresh_bbox = makeVehicleTracker(fresh_obj, time, kSpinStrong, channel_);

  // Sanity: the partial tracker is stale on full measurements, the fresh one is not.
  ASSERT_GT(stale_partial->getElapsedTimeFromFullMeasurement(mergeTime()), 0.5);
  ASSERT_LT(fresh_bbox->getElapsedTimeFromFullMeasurement(mergeTime()), 0.5);
  ASSERT_TRUE(fresh_bbox->isConfident(cache_, std::nullopt, mergeTime()));

  std::list<std::shared_ptr<mot::Tracker>> trackers{stale_partial, fresh_bbox};
  runMerge(trackers, mergeTime());

  // The fresh, fully-measured tracker wins and absorbs the stale partial one.
  ASSERT_EQ(trackers.size(), 1U);
  EXPECT_EQ(trackers.front(), fresh_bbox);
}

TEST_F(TrackerOverlapManagerTest, StalePartialVehicleDoesNotAbsorbImmatureFreshTracker)
{
  const auto time = baseTime();

  mot::types::InputChannel cluster_channel = channel_;
  cluster_channel.index = 1;
  cluster_channel.trust_extension = false;

  auto partial_obj = makeBboxObject(0.5, 0.0, 4.5, 1.8, mot::classes::Label::CAR, time);
  partial_obj.trust_extension = false;
  auto stale_partial = makeVehicleTracker(partial_obj, time, kSpinStrong, cluster_channel);

  // A just-spawned bbox tracker: fully measured at spawn but not yet confident (count < 2).
  const auto fresh_time = mergeTime() - rclcpp::Duration(100ms);
  const auto fresh_obj = makeBboxObject(0.0, 0.0, 4.5, 1.8, mot::classes::Label::CAR, fresh_time);
  auto fresh_new = makeVehicleTracker(fresh_obj, fresh_time, 0, channel_);

  ASSERT_LT(fresh_new->getElapsedTimeFromFullMeasurement(mergeTime()), 0.5);
  ASSERT_FALSE(fresh_new->isConfident(cache_, std::nullopt, mergeTime()));

  std::list<std::shared_ptr<mot::Tracker>> trackers{stale_partial, fresh_new};
  runMerge(trackers, mergeTime());

  // The stale partial tracker outranks-loses to the fresh one, but the fresh one is not yet
  // confident, so the winner-eligibility check defers the merge: both trackers survive. The stale
  // tracker must never absorb the fresh detection.
  EXPECT_EQ(trackers.size(), 2U);
  EXPECT_TRUE(
    std::any_of(trackers.begin(), trackers.end(), [&](const auto & t) { return t == fresh_new; }));
}

TEST_F(TrackerOverlapManagerTest, ChainConflictAppliesStrongerMergeFirst)
{
  const auto time = baseTime();
  // A <- B <- C chain; the star-forest rule defers one edge. The stronger edge applies: fresh bbox
  // A absorbs stale partial B, deferring the stale C-into-B edge. B is absorbed; A and C survive.
  mot::types::InputChannel cluster_channel = channel_;
  cluster_channel.index = 1;
  cluster_channel.trust_extension = false;

  auto obj_a =
    makeBboxObject(0.0, 0.0, 4.0, 2.0, mot::classes::Label::CAR, time);  // fresh full box
  auto obj_b = makeBboxObject(1.5, 0.0, 2.0, 2.0, mot::classes::Label::CAR, time);
  auto obj_c = makeBboxObject(2.8, 0.0, 2.0, 2.0, mot::classes::Label::CAR, time);
  obj_b.trust_extension = false;  // stale partial
  obj_c.trust_extension = false;  // stale partial

  auto fresh_a = makeVehicleTracker(obj_a, time, kSpinStrong, channel_);
  auto stale_b = makeVehicleTracker(obj_b, time, kSpinStrong, cluster_channel);
  auto stale_c = makeVehicleTracker(obj_c, time, kSpinMedium, cluster_channel);

  // Sanity: A is fresh, B and C are stale; B outranks C on covariance (more updates).
  ASSERT_LT(fresh_a->getElapsedTimeFromFullMeasurement(mergeTime()), 0.35);
  ASSERT_GT(stale_b->getElapsedTimeFromFullMeasurement(mergeTime()), 0.35);
  ASSERT_GT(stale_c->getElapsedTimeFromFullMeasurement(mergeTime()), 0.35);

  std::list<std::shared_ptr<mot::Tracker>> trackers{stale_c, fresh_a, stale_b};
  runMerge(trackers, mergeTime());

  // Star-forest leaves two trackers, but the fresh edge (B into A) wins the conflict: B is gone,
  // A and C survive.
  ASSERT_EQ(trackers.size(), 2U);
  EXPECT_TRUE(
    std::any_of(trackers.begin(), trackers.end(), [&](const auto & t) { return t == fresh_a; }));
  EXPECT_TRUE(
    std::any_of(trackers.begin(), trackers.end(), [&](const auto & t) { return t == stale_c; }));
  EXPECT_FALSE(
    std::any_of(trackers.begin(), trackers.end(), [&](const auto & t) { return t == stale_b; }));
}

TEST_F(TrackerOverlapManagerTest, NonVehicleTrackersAlwaysCountFullyMeasured)
{
  const auto time = baseTime();
  const auto obj = makePolygonObject(0.0, 0.0, 1.0, time);
  auto tracker = makePolygonTracker(obj, time, kSpinWeak, channel_);

  // Freshness on full measurements is a vehicle-tracker concept; other trackers never go stale.
  EXPECT_DOUBLE_EQ(tracker->getElapsedTimeFromFullMeasurement(mergeTime()), 0.0);
}

TEST_F(TrackerOverlapManagerTest, GateAdmitsLargePolygonByFootprintExtent)
{
  const auto time = baseTime();
  // A wide unknown cluster whose footprint reaches over the car while its center sits well beyond
  // the car's own cover; the gate sizes the cluster by its footprint (its dimensions are zero).
  const auto car_obj = makeBboxObject(0.0, 0.0, 4.5, 1.8, mot::classes::Label::CAR, time);
  const auto cluster_obj = makePolygonObject(5.0, 0.0, 4.5, time);

  auto car = makeVehicleTracker(car_obj, time, kSpinStrong, channel_);
  auto cluster = makePolygonTracker(cluster_obj, time, kSpinMedium, channel_);

  std::list<std::shared_ptr<mot::Tracker>> trackers{car, cluster};
  runMerge(trackers, mergeTime());

  ASSERT_EQ(trackers.size(), 1U);
  EXPECT_EQ(trackers.front(), car);
}

TEST_F(TrackerOverlapManagerTest, OutcomeIsIndependentOfListOrder)
{
  // Three separated cars, each over-segmented into a polygon fragment, plus one isolated
  // polygon. The merge outcome (survivor labels and positions) must not depend on the order
  // trackers appear in the list.
  const auto buildScenario = [&](const std::vector<size_t> & order) {
    const auto time = baseTime();
    std::vector<std::shared_ptr<mot::Tracker>> built;
    for (double car_x : {0.0, 20.0, 40.0}) {
      built.push_back(makeVehicleTracker(
        makeBboxObject(car_x, 0.0, 4.5, 1.8, mot::classes::Label::CAR, time), time, kSpinStrong,
        channel_));
      built.push_back(makePolygonTracker(
        makePolygonObject(car_x + 1.0, 0.0, 0.8, time), time, kSpinMedium, channel_));
    }
    built.push_back(
      makePolygonTracker(makePolygonObject(100.0, 0.0, 1.0, time), time, kSpinMedium, channel_));

    std::list<std::shared_ptr<mot::Tracker>> trackers;
    for (const size_t idx : order) {
      trackers.push_back(built[idx]);
    }
    return trackers;
  };

  const auto surviving_positions = [&](std::list<std::shared_ptr<mot::Tracker>> & trackers) {
    std::multiset<std::tuple<int, int, int>> result;
    for (const auto & tracker : trackers) {
      mot::types::DynamicObject obj;
      EXPECT_TRUE(tracker->getTrackedObject(mergeTime(), obj));
      result.emplace(
        static_cast<int>(tracker->getHighestProbLabel()),
        static_cast<int>(std::round(obj.pose.position.x)),
        static_cast<int>(std::round(obj.pose.position.y)));
    }
    return result;
  };

  const std::vector<std::vector<size_t>> orders = {
    {0, 1, 2, 3, 4, 5, 6},
    {6, 5, 4, 3, 2, 1, 0},
    {3, 0, 6, 2, 5, 1, 4},
    {1, 3, 5, 0, 2, 4, 6},
  };

  std::optional<std::multiset<std::tuple<int, int, int>>> reference;
  for (const auto & order : orders) {
    auto trackers = buildScenario(order);
    runMerge(trackers, mergeTime());
    EXPECT_EQ(trackers.size(), 4U);  // 3 cars + isolated polygon
    const auto survivors = surviving_positions(trackers);
    if (!reference) {
      reference = survivors;
    } else {
      EXPECT_EQ(survivors, *reference);
    }
  }
}
