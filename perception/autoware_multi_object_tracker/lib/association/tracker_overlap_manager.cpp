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

#include "autoware/multi_object_tracker/association/tracker_overlap_manager.hpp"

#include "autoware/multi_object_tracker/association/scoring/redundancy_check.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <algorithm>
#include <list>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using OmPoint = bg::model::point<double, 2, bg::cs::cartesian>;
using OmValue = std::pair<OmPoint, size_t>;  // (position, index into valid_trackers)

TrackerOverlapManager::TrackerOverlapManager(const TrackerOverlapManagerConfig & config)
: config_(config)
{
}

bool TrackerOverlapManager::canMergeTarget(
  const Tracker & target, const Tracker & other, const rclcpp::Time & time,
  const AdaptiveThresholdCache & threshold_cache,
  const std::optional<geometry_msgs::msg::Pose> & ego_pose) const
{
  // 0. Compare tracker priority: higher priority (lower index) is never absorbed
  if (target.getTrackerPriority() < other.getTrackerPriority()) {
    return false;
  }

  // 1. If the other is not confident, do not remove the target
  if (!other.isConfident(threshold_cache, ego_pose, time)) {
    return false;
  }

  // 2. Compare known class probability
  const float target_known_prob = target.getKnownObjectProbability();
  const float other_known_prob = other.getKnownObjectProbability();
  constexpr float min_known_prob = 0.2;

  if (target_known_prob >= min_known_prob) {
    // Target class is known
    if (other_known_prob < min_known_prob) {
      // Other is unknown -> keep target
      return false;
    }
    // Both known: compare per-channel existence probabilities
    const std::vector<types::ExistenceProbability> target_existence_prob =
      target.getExistenceProbabilityVector();
    const std::vector<types::ExistenceProbability> other_existence_prob =
      other.getExistenceProbabilityVector();
    constexpr float prob_buffer = 0.4;

    for (const auto & other_prob : other_existence_prob) {
      float target_prob_val = 0.001f;
      for (const auto & target_prob : target_existence_prob) {
        if (target_prob.channel_index == other_prob.channel_index) {
          target_prob_val = target_prob.existence_probability;
          break;
        }
      }
      if (target_prob_val + prob_buffer < other_prob.existence_probability) {
        // Other significantly outperforms target on this channel -> remove target
        return true;
      }
    }

    // No large per-channel difference: remove the one with larger position uncertainty
    return target.getPositionCovarianceDeterminant() > other.getPositionCovarianceDeterminant();
  }

  // 3. Target class is unknown
  if (other_known_prob < min_known_prob) {
    // Both unknown: remove the one with larger position uncertainty
    return target.getPositionCovarianceDeterminant() > other.getPositionCovarianceDeterminant();
  }
  // Other is known, target is unknown -> remove target
  return true;
}

void TrackerOverlapManager::merge(
  std::list<std::shared_ptr<Tracker>> & tracker_list, const rclcpp::Time & time,
  const AdaptiveThresholdCache & threshold_cache,
  const std::optional<geometry_msgs::msg::Pose> & ego_pose)
{
  // Local cache of per-tracker data to avoid repeated virtual calls
  struct TrackerData
  {
    std::shared_ptr<Tracker> tracker;
    types::DynamicObject object;
    classes::Label label;
    bool is_unknown;
    int tracker_priority;
    int measurement_count;
    double elapsed_time;
    bool is_valid;

    explicit TrackerData(const std::shared_ptr<Tracker> & t)
    : tracker(t),
      object(),
      label(classes::Label::UNKNOWN),
      is_unknown(false),
      tracker_priority(0),
      measurement_count(0),
      elapsed_time(0.0),
      is_valid(false)
    {
    }
  };

  std::vector<TrackerData> valid_trackers;
  valid_trackers.reserve(tracker_list.size());

  // First pass: collect valid trackers and their data
  for (const auto & tracker : tracker_list) {
    TrackerData data(tracker);
    if (!tracker->getTrackedObject(time, data.object)) {
      continue;
    }
    data.label = tracker->getHighestProbLabel();
    data.is_unknown = (data.label == classes::Label::UNKNOWN);
    data.tracker_priority = tracker->getTrackerPriority();
    data.measurement_count = tracker->getTotalMeasurementCount();
    data.elapsed_time = tracker->getElapsedTimeFromLastUpdate(time);
    data.is_valid = true;
    valid_trackers.push_back(std::move(data));
  }

  // Sort by priority: lower index -> higher priority, then non-unknown, then more measurements
  std::sort(
    valid_trackers.begin(), valid_trackers.end(), [](const TrackerData & a, const TrackerData & b) {
      if (a.tracker_priority != b.tracker_priority) {
        return a.tracker_priority < b.tracker_priority;
      }
      if (a.is_unknown != b.is_unknown) {
        return b.is_unknown;  // Non-unknown first
      }
      if (a.measurement_count != b.measurement_count) {
        return a.measurement_count > b.measurement_count;
      }
      return a.elapsed_time < b.elapsed_time;
    });

  // Build R-tree for spatial lookup
  bgi::rtree<OmValue, bgi::quadratic<16>> rtree;
  std::vector<OmValue> rtree_points;
  rtree_points.reserve(valid_trackers.size());
  for (size_t i = 0; i < valid_trackers.size(); ++i) {
    const auto & data = valid_trackers[i];
    if (!data.is_valid) continue;
    OmPoint p(data.object.pose.position.x, data.object.pose.position.y);
    rtree_points.emplace_back(p, i);
  }
  rtree.insert(rtree_points.begin(), rtree_points.end());

  std::vector<size_t> to_remove;
  to_remove.reserve(valid_trackers.size() / 4);

  // Second pass: find and merge overlapping trackers
  for (size_t i = 0; i < valid_trackers.size(); ++i) {
    auto & data1 = valid_trackers[i];
    if (!data1.is_valid || !data1.tracker->isConfident(threshold_cache, ego_pose, time)) continue;

    const auto max_search_dist_sq_opt =
      get_map_value_if_exists(config_.pruning_distance_thresholds_sq, data1.label);
    if (!max_search_dist_sq_opt) continue;
    const double max_search_dist_sq = max_search_dist_sq_opt->get();

    std::vector<OmValue> nearby;
    nearby.reserve(16);
    rtree.query(
      bgi::satisfies([&](const OmValue & v) {
        if (v.second <= i) return false;  // Skip already-processed and self
        const double dx = bg::get<0>(v.first) - data1.object.pose.position.x;
        const double dy = bg::get<1>(v.first) - data1.object.pose.position.y;
        return dx * dx + dy * dy <= max_search_dist_sq;
      }),
      std::back_inserter(nearby));

    for (const auto & [p2, idx2] : nearby) {
      auto & data2 = valid_trackers[idx2];
      if (!data2.is_valid) continue;

      if (
        canMergeTarget(*data2.tracker, *data1.tracker, time, threshold_cache, ego_pose) &&
        isRedundant(
          data1.object, data2.object, data1.label, data2.label,
          data1.tracker->getKnownObjectProbability(), data2.tracker->getKnownObjectProbability(),
          config_)) {
        // Merge data2 (lower priority) into data1 (higher priority)

        // Existence probabilities
        data1.tracker->updateTotalExistenceProbability(
          data2.tracker->getTotalExistenceProbability());
        data1.tracker->mergeExistenceProbabilities(data2.tracker->getExistenceProbabilityVector());

        // Classification: only update if source is known
        if (!data2.is_unknown) {
          data1.tracker->updateClassification(data2.tracker->getClassification());
        }

        // Shape: prefer lower shape type (bounding box < cylinder < convex hull)
        if (data1.object.shape.type > data2.object.shape.type) {
          data1.tracker->setObjectShape(data2.object.shape);
        }

        data2.is_valid = false;
        to_remove.push_back(idx2);
      }
    }
  }

  // Final pass: remove merged trackers
  std::unordered_set<std::shared_ptr<Tracker>> trackers_to_remove;
  trackers_to_remove.reserve(to_remove.size());
  for (const auto idx : to_remove) {
    trackers_to_remove.insert(valid_trackers[idx].tracker);
  }
  tracker_list.remove_if([&trackers_to_remove](const std::shared_ptr<Tracker> & t) {
    return trackers_to_remove.count(t) > 0;
  });
}

}  // namespace autoware::multi_object_tracker
