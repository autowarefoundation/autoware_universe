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
#include <array>
#include <cmath>
#include <list>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using OmPoint = bg::model::point<double, 2, bg::cs::cartesian>;
using OmValue = std::pair<OmPoint, size_t>;  // (position, index into snapshots)

namespace
{

constexpr float min_known_prob = 0.2f;

// Per-tracker state captured once per merge cycle. All pair decisions read snapshots, never the
// live tracker, so the outcome is a pure function of tracker states — independent of list order
// and of merges applied later in the same cycle.
struct TrackerSnapshot
{
  std::shared_ptr<Tracker> tracker;
  geometry_msgs::msg::Point position;
  classes::Label label{classes::Label::UNKNOWN};
  bool is_unknown{true};
  int priority{0};
  float known_prob{0.0f};
  double cov_det{0.0};
  int measurement_count{0};
  std::array<uint8_t, 16> uuid{};
  std::vector<types::ExistenceProbability> existence_probs;
  // Filled lazily, only for trackers that appear in a distance-gated pair
  std::optional<bool> confident;
  std::optional<bool> object_valid;
  types::DynamicObject object;
};

// True when lhs significantly outperforms rhs on at least one input channel.
// A channel missing from rhs counts as near-zero probability.
bool dominatesOnAnyChannel(
  const std::vector<types::ExistenceProbability> & lhs,
  const std::vector<types::ExistenceProbability> & rhs)
{
  constexpr float prob_buffer = 0.4f;
  for (const auto & lhs_prob : lhs) {
    float rhs_prob_val = 0.001f;
    for (const auto & rhs_prob : rhs) {
      if (rhs_prob.channel_index == lhs_prob.channel_index) {
        rhs_prob_val = rhs_prob.existence_probability;
        break;
      }
    }
    if (rhs_prob_val + prob_buffer < lhs_prob.existence_probability) {
      return true;
    }
  }
  return false;
}

}  // namespace

TrackerOverlapManager::TrackerOverlapManager(const TrackerOverlapManagerConfig & config)
: config_(config)
{
}

void TrackerOverlapManager::merge(
  std::list<std::shared_ptr<Tracker>> & tracker_list, const rclcpp::Time & time,
  const AdaptiveThresholdCache & threshold_cache,
  const std::optional<geometry_msgs::msg::Pose> & ego_pose)
{
  // ---- Snapshot pass: cheap scalars only ----
  // Position comes from the motion model (no shape/classification assembly); the full object is
  // fetched lazily per gated pair.
  std::vector<TrackerSnapshot> snapshots;
  snapshots.reserve(tracker_list.size());
  for (const auto & tracker : tracker_list) {
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Twist twist;
    std::array<double, 36> pose_cov{};
    std::array<double, 36> twist_cov{};
    if (!tracker->getMotionState(time, pose, pose_cov, twist, twist_cov)) {
      continue;
    }
    TrackerSnapshot snap;
    snap.tracker = tracker;
    snap.position = pose.position;
    snap.label = tracker->getHighestProbLabel();
    snap.is_unknown = (snap.label == classes::Label::UNKNOWN);
    snap.priority = tracker->getTrackerPriority();
    snap.known_prob = tracker->getKnownObjectProbability();
    snap.cov_det = tracker->getPositionCovarianceDeterminant();
    snap.measurement_count = tracker->getTotalMeasurementCount();
    snap.uuid = tracker->getUUID().uuid;
    snap.existence_probs = tracker->getExistenceProbabilityVector();
    snapshots.push_back(std::move(snap));
  }
  if (snapshots.size() < 2) {
    return;
  }

  const auto is_confident = [&](TrackerSnapshot & snap) {
    if (!snap.confident) {
      snap.confident = snap.tracker->isConfident(threshold_cache, ego_pose, time);
    }
    return *snap.confident;
  };
  const auto get_object = [&](TrackerSnapshot & snap) -> const types::DynamicObject * {
    if (!snap.object_valid) {
      snap.object_valid = snap.tracker->getTrackedObject(time, snap.object);
    }
    return *snap.object_valid ? &snap.object : nullptr;
  };

  // ---- Candidate pair discovery ----
  // Each tracker queries with its own label radius; pairs are deduplicated, so the effective
  // gate is dist <= max(radius_a, radius_b) regardless of which side discovers the pair.
  bgi::rtree<OmValue, bgi::quadratic<16>> rtree;
  {
    std::vector<OmValue> rtree_points;
    rtree_points.reserve(snapshots.size());
    for (size_t i = 0; i < snapshots.size(); ++i) {
      rtree_points.emplace_back(OmPoint(snapshots[i].position.x, snapshots[i].position.y), i);
    }
    rtree.insert(rtree_points.begin(), rtree_points.end());
  }

  std::vector<std::pair<size_t, size_t>> candidate_pairs;
  std::vector<OmValue> nearby;
  for (size_t i = 0; i < snapshots.size(); ++i) {
    const auto max_search_dist_sq_opt =
      get_map_value_if_exists(config_.pruning_distance_thresholds_sq, snapshots[i].label);
    if (!max_search_dist_sq_opt) {
      continue;
    }
    const double max_search_dist_sq = max_search_dist_sq_opt->get();
    const double max_search_dist = std::sqrt(max_search_dist_sq);
    const double x = snapshots[i].position.x;
    const double y = snapshots[i].position.y;
    // The box predicate lets the R-tree prune subtrees; satisfies alone would scan linearly.
    const bg::model::box<OmPoint> search_box(
      OmPoint(x - max_search_dist, y - max_search_dist),
      OmPoint(x + max_search_dist, y + max_search_dist));

    nearby.clear();
    rtree.query(
      bgi::intersects(search_box) && bgi::satisfies([&](const OmValue & v) {
        if (v.second == i) return false;
        const double dx = bg::get<0>(v.first) - x;
        const double dy = bg::get<1>(v.first) - y;
        return dx * dx + dy * dy <= max_search_dist_sq;
      }),
      std::back_inserter(nearby));
    for (const auto & [point, j] : nearby) {
      candidate_pairs.emplace_back(std::min(i, j), std::max(i, j));
    }
  }
  std::sort(candidate_pairs.begin(), candidate_pairs.end());
  candidate_pairs.erase(
    std::unique(candidate_pairs.begin(), candidate_pairs.end()), candidate_pairs.end());

  // ---- Pair decision ----
  // Direction comes from a lexicographic comparison ending in the UUID, so every pair has a
  // strict, list-order-independent winner. The per-channel existence tier is evaluated in both
  // directions; mutual or no dominance falls through to the next tier.
  const auto outranks = [&](TrackerSnapshot & a, TrackerSnapshot & b) {
    if (a.priority != b.priority) {
      return a.priority < b.priority;
    }
    const bool a_confident = is_confident(a);
    const bool b_confident = is_confident(b);
    if (a_confident != b_confident) {
      return a_confident;
    }
    const bool a_known = a.known_prob >= min_known_prob;
    const bool b_known = b.known_prob >= min_known_prob;
    if (a_known != b_known) {
      return a_known;
    }
    const bool a_dominates = dominatesOnAnyChannel(a.existence_probs, b.existence_probs);
    const bool b_dominates = dominatesOnAnyChannel(b.existence_probs, a.existence_probs);
    if (a_dominates != b_dominates) {
      return a_dominates;
    }
    if (a.cov_det != b.cov_det) {
      return a.cov_det < b.cov_det;
    }
    if (a.measurement_count != b.measurement_count) {
      return a.measurement_count > b.measurement_count;
    }
    return a.uuid < b.uuid;
  };

  struct MergeCandidate
  {
    size_t winner_idx;
    double dist_sq;
  };
  std::unordered_map<size_t, MergeCandidate> best_winner_of_loser;
  for (const auto & [i, j] : candidate_pairs) {
    const bool i_wins = outranks(snapshots[i], snapshots[j]);
    const size_t winner_idx = i_wins ? i : j;
    const size_t loser_idx = i_wins ? j : i;
    auto & winner = snapshots[winner_idx];
    auto & loser = snapshots[loser_idx];

    // Winner eligibility: must be confident and carry a parametric shape. Polygon-only trackers
    // may only be absorbed, never absorb — in particular two polygon-only trackers never merge.
    // An ineligible winner vetoes the pair (no direction flip).
    if (!is_confident(winner)) {
      continue;
    }
    const auto * winner_object = get_object(winner);
    const auto * loser_object = get_object(loser);
    if (winner_object == nullptr || loser_object == nullptr) {
      continue;
    }
    if (winner_object->shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
      continue;
    }
    if (!isRedundant(
          *winner_object, *loser_object, winner.label, loser.label, winner.known_prob,
          loser.known_prob, config_)) {
      continue;
    }

    const double dx = winner.position.x - loser.position.x;
    const double dy = winner.position.y - loser.position.y;
    const double dist_sq = dx * dx + dy * dy;
    const auto it = best_winner_of_loser.find(loser_idx);
    if (
      it == best_winner_of_loser.end() || dist_sq < it->second.dist_sq ||
      (dist_sq == it->second.dist_sq && winner.uuid < snapshots[it->second.winner_idx].uuid)) {
      best_winner_of_loser[loser_idx] = MergeCandidate{winner_idx, dist_sq};
    }
  }
  if (best_winner_of_loser.empty()) {
    return;
  }

  // ---- Apply phase ----
  // Greedy in loser-UUID order, keeping the applied merges a star forest: a tracker may not both
  // absorb and be absorbed within one cycle, so absorbed content never moves through a tracker
  // whose overlap with the final destination was not geometry-checked. Deferred merges re-enter
  // next cycle; the first edge in UUID order always applies, so chains make progress every cycle.
  std::vector<size_t> loser_indices;
  loser_indices.reserve(best_winner_of_loser.size());
  for (const auto & [loser_idx, candidate] : best_winner_of_loser) {
    loser_indices.push_back(loser_idx);
  }
  std::sort(loser_indices.begin(), loser_indices.end(), [&](const size_t a, const size_t b) {
    return snapshots[a].uuid < snapshots[b].uuid;
  });

  std::vector<bool> was_absorbed(snapshots.size(), false);
  std::vector<bool> has_absorbed(snapshots.size(), false);
  std::unordered_set<std::shared_ptr<Tracker>> trackers_to_remove;
  trackers_to_remove.reserve(loser_indices.size());
  for (const size_t loser_idx : loser_indices) {
    const size_t winner_idx = best_winner_of_loser.at(loser_idx).winner_idx;
    if (was_absorbed[winner_idx] || has_absorbed[loser_idx]) {
      continue;
    }
    auto & winner = snapshots[winner_idx];
    auto & loser = snapshots[loser_idx];

    // Existence probabilities
    winner.tracker->updateTotalExistenceProbability(loser.tracker->getTotalExistenceProbability());
    winner.tracker->mergeExistenceProbabilities(loser.existence_probs);

    // Classification: only update if source is known
    if (!loser.is_unknown) {
      winner.tracker->updateClassification(loser.tracker->getClassification());
    }

    // Shape: prefer lower shape type (bounding box < cylinder < convex hull).
    if (winner.object.shape.type > loser.object.shape.type) {
      winner.tracker->setObjectShape(loser.object.shape);
    }
    // If the absorbed tracker carries footprint data (BBOX or POLYGON), union it into the winner.
    if (!loser.object.shape.footprint.points.empty()) {
      winner.tracker->mergeFootprintFrom(
        loser.object.shape.footprint, loser.object.pose, winner.object.pose);
    }

    was_absorbed[loser_idx] = true;
    has_absorbed[winner_idx] = true;
    trackers_to_remove.insert(loser.tracker);
  }

  tracker_list.remove_if([&trackers_to_remove](const std::shared_ptr<Tracker> & t) {
    return trackers_to_remove.count(t) > 0;
  });
}

}  // namespace autoware::multi_object_tracker
