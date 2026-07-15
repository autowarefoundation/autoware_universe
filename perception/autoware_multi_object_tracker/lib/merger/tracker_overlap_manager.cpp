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

#include "autoware/multi_object_tracker/merger/tracker_overlap_manager.hpp"

#include "autoware/multi_object_tracker/merger/detail/overlap_gate.hpp"
#include "autoware/multi_object_tracker/merger/detail/overlap_types.hpp"
#include "autoware/multi_object_tracker/merger/detail/redundancy_check.hpp"
#include "autoware/multi_object_tracker/merger/detail/survival_ranking.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <list>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{

using detail::compareForSurvival;
using detail::compareWinnerSubstance;
using detail::DecisionContext;
using detail::ensureConfident;
using detail::ensureObject;
using detail::findCandidatePairs;
using detail::isRedundant;
using detail::TrackerSnapshot;
using detail::withinDeadband;

namespace
{

// merge() runs the tracker list through four stages:
//   Stage 0  buildSnapshots                        per-tracker scalars, captured once
//   Stage 1  findCandidatePairs + decideMergeEdges  spatial gate + per-pair winner→loser decision
//   Stage 2  groupBestWinnerPerLoser               one best winner per loser
//   Stage 3  applyMerges                           apply the star-forest merges, prune
//
// Shared types live in detail/overlap_types; the spatial gate in detail/overlap_gate; the survival
// and winner ranking in detail/survival_ranking.

// Elapsed time above which a tracker counts as stale on full (trust_extension) measurements.
constexpr double full_measure_stale_threshold = 0.35;  // [sec]

// Deadband for the distance tie-break between equally-strong winners.
constexpr double dist_sq_relative_tol = 1e-3;
constexpr double dist_sq_absolute_tol = 1e-6;  // [m^2]

// Heading angle about +z from a quaternion.
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// ---------------------------------------------------------------------------
// Stage 0 — snapshot
// ---------------------------------------------------------------------------

// Capture per-tracker scalars from the motion model; the full object is fetched lazily later.
// Trackers without a valid motion state this cycle are dropped.
std::vector<TrackerSnapshot> buildSnapshots(
  const std::list<std::shared_ptr<Tracker>> & tracker_list, const rclcpp::Time & time)
{
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
    snap.yaw = yawFromQuaternion(pose.orientation);
    snap.label = tracker->getHighestProbLabel();
    snap.is_unknown = (snap.label == classes::Label::UNKNOWN);
    snap.priority = tracker->getTrackerPriority();
    snap.known_prob = tracker->getKnownObjectProbability();
    snap.cov_det = tracker->getPositionCovarianceDeterminant();
    snap.measurement_count = tracker->getTotalMeasurementCount();
    snap.fully_measured_stale =
      tracker->getElapsedTimeFromFullMeasurement(time) > full_measure_stale_threshold;
    snap.uuid = tracker->getUUID().uuid;
    snap.existence_probs = tracker->getExistenceProbabilityVector();
    snapshots.push_back(std::move(snap));
  }
  return snapshots;
}

// ---------------------------------------------------------------------------
// Stage 1 — decide directed merge edges
//   spatial gate (detail::findCandidatePairs) + per-pair direction & eligibility
// ---------------------------------------------------------------------------

// Outcome of one pair: no merge, or a directional merge in which the survivor absorbs the other.
enum class PairDecision { NoMerge, LeftAbsorbsRight, RightAbsorbsLeft };

// Ranking picks the survivor; it must be confident and carry a parametric shape (never
// polygon-only), and the pair must be spatially redundant. Otherwise the pair does not merge.
PairDecision decidePair(TrackerSnapshot & a, TrackerSnapshot & b, const DecisionContext & ctx)
{
  const bool a_survives = compareForSurvival(a, b, ctx) > 0;
  TrackerSnapshot & winner = a_survives ? a : b;
  TrackerSnapshot & loser = a_survives ? b : a;

  if (!ensureConfident(winner, ctx)) {
    return PairDecision::NoMerge;
  }
  const auto * winner_object = ensureObject(winner, ctx.time);
  const auto * loser_object = ensureObject(loser, ctx.time);
  if (winner_object == nullptr || loser_object == nullptr) {
    return PairDecision::NoMerge;
  }
  if (winner_object->shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    return PairDecision::NoMerge;
  }
  if (!isRedundant(
        *winner_object, *loser_object, winner.label, loser.label, winner.known_prob,
        loser.known_prob, ctx.config)) {
    return PairDecision::NoMerge;
  }
  return a_survives ? PairDecision::LeftAbsorbsRight : PairDecision::RightAbsorbsLeft;
}

// A directed merge decision: winner absorbs loser. dist_sq is the winner↔loser center distance².
struct MergeEdge
{
  size_t winner_idx;
  size_t loser_idx;
  double dist_sq;
};

// Discover spatially-close pairs, decide each, and emit one edge per merging pair.
std::vector<MergeEdge> decideMergeEdges(
  std::vector<TrackerSnapshot> & snapshots, const DecisionContext & ctx)
{
  const std::vector<std::pair<size_t, size_t>> candidate_pairs =
    findCandidatePairs(snapshots, ctx.config);

  std::vector<MergeEdge> edges;
  edges.reserve(candidate_pairs.size());
  for (const auto & [i, j] : candidate_pairs) {
    const PairDecision decision = decidePair(snapshots[i], snapshots[j], ctx);
    if (decision == PairDecision::NoMerge) {
      continue;
    }
    const size_t winner_idx = decision == PairDecision::LeftAbsorbsRight ? i : j;
    const size_t loser_idx = decision == PairDecision::LeftAbsorbsRight ? j : i;
    const double dx = snapshots[winner_idx].position.x - snapshots[loser_idx].position.x;
    const double dy = snapshots[winner_idx].position.y - snapshots[loser_idx].position.y;
    edges.push_back(MergeEdge{winner_idx, loser_idx, dx * dx + dy * dy});
  }
  return edges;
}

// ---------------------------------------------------------------------------
// Stage 2 — group edges into one best winner per loser
// ---------------------------------------------------------------------------

struct MergeCandidate
{
  size_t winner_idx;
  double dist_sq;
};

// A loser may be beaten by several winners; keep its best: highest-ranked, then nearest, then
// lowest UUID.
std::unordered_map<size_t, MergeCandidate> groupBestWinnerPerLoser(
  const std::vector<MergeEdge> & edges, std::vector<TrackerSnapshot> & snapshots,
  const DecisionContext & ctx)
{
  std::unordered_map<size_t, MergeCandidate> best_winner_of_loser;
  for (const auto & edge : edges) {
    const auto it = best_winner_of_loser.find(edge.loser_idx);
    if (it == best_winner_of_loser.end()) {
      best_winner_of_loser[edge.loser_idx] = MergeCandidate{edge.winner_idx, edge.dist_sq};
      continue;
    }
    auto & incumbent = snapshots[it->second.winner_idx];
    const int rank = compareWinnerSubstance(snapshots[edge.winner_idx], incumbent, ctx);
    if (rank > 0) {
      it->second = MergeCandidate{edge.winner_idx, edge.dist_sq};  // stronger substance
      continue;
    }
    if (rank < 0) {
      continue;  // incumbent is intrinsically stronger; keep it
    }
    // Equal substance: the nearer winner wins; inside the distance deadband, the lower UUID wins.
    const bool tie =
      withinDeadband(edge.dist_sq, it->second.dist_sq, dist_sq_relative_tol, dist_sq_absolute_tol);
    const bool prefer_new =
      tie ? snapshots[edge.winner_idx].uuid < incumbent.uuid : edge.dist_sq < it->second.dist_sq;
    if (prefer_new) {
      it->second = MergeCandidate{edge.winner_idx, edge.dist_sq};
    }
  }
  return best_winner_of_loser;
}

// ---------------------------------------------------------------------------
// Stage 3 — order strongest-first, apply the star-forest merges, prune
// ---------------------------------------------------------------------------

// Apply merges greedily, strongest edge first (winner rank, then loser UUID). A tracker may not
// both absorb and be absorbed in one cycle (star forest); skipped merges re-enter next cycle, so
// chains converge over several cycles. All tracker mutation happens here.
void applyMerges(
  const std::unordered_map<size_t, MergeCandidate> & best_winner_of_loser,
  std::vector<TrackerSnapshot> & snapshots, std::list<std::shared_ptr<Tracker>> & tracker_list,
  const DecisionContext & ctx)
{
  std::vector<size_t> loser_indices;
  loser_indices.reserve(best_winner_of_loser.size());
  for (const auto & [loser_idx, candidate] : best_winner_of_loser) {
    loser_indices.push_back(loser_idx);
  }
  std::sort(loser_indices.begin(), loser_indices.end(), [&](const size_t a, const size_t b) {
    const size_t winner_a = best_winner_of_loser.at(a).winner_idx;
    const size_t winner_b = best_winner_of_loser.at(b).winner_idx;
    if (const int r = compareWinnerSubstance(snapshots[winner_a], snapshots[winner_b], ctx)) {
      return r > 0;
    }
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

    winner.tracker->updateTotalExistenceProbability(loser.tracker->getTotalExistenceProbability());
    winner.tracker->mergeExistenceProbabilities(loser.existence_probs);

    if (!loser.is_unknown) {
      winner.tracker->updateClassification(loser.tracker->getClassification());
    }

    // Prefer the lower shape type (bounding box < cylinder < convex hull).
    if (winner.object.shape.type > loser.object.shape.type) {
      winner.tracker->setObjectShape(loser.object.shape);
    }
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

}  // namespace

// ---------------------------------------------------------------------------
// merge — public entry point, runs the four stages in order
// ---------------------------------------------------------------------------

TrackerOverlapManager::TrackerOverlapManager(const TrackerOverlapManagerConfig & config)
: config_(config)
{
}

void TrackerOverlapManager::merge(
  std::list<std::shared_ptr<Tracker>> & tracker_list, const rclcpp::Time & time,
  const AdaptiveThresholdCache & threshold_cache,
  const std::optional<geometry_msgs::msg::Pose> & ego_pose)
{
  // Stage 0 — Snapshot each tracker's scalars once.
  std::vector<TrackerSnapshot> snapshots = buildSnapshots(tracker_list, time);
  if (snapshots.size() < 2) {
    return;
  }
  const DecisionContext ctx{threshold_cache, ego_pose, time, config_};

  // Stage 1 — Decide directed merge edges (spatial gate + per-pair winner→loser).
  const std::vector<MergeEdge> edges = decideMergeEdges(snapshots, ctx);
  if (edges.empty()) {
    return;
  }

  // Stage 2 — Group the edges into one best winner per loser.
  const std::unordered_map<size_t, MergeCandidate> best_winner_of_loser =
    groupBestWinnerPerLoser(edges, snapshots, ctx);

  // Stage 3 — Order by winner strength, apply the star-forest merges, prune absorbed trackers.
  applyMerges(best_winner_of_loser, snapshots, tracker_list, ctx);
}

}  // namespace autoware::multi_object_tracker
