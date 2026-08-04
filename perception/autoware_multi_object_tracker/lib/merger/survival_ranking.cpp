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

#include "autoware/multi_object_tracker/merger/detail/survival_ranking.hpp"

#include "autoware/multi_object_tracker/merger/detail/overlap_types.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace autoware::multi_object_tracker::detail
{

namespace
{

constexpr float min_known_prob = 0.2f;

// Dead-bands for the ranking tiers that compare continuous quantities.
constexpr double cov_det_relative_tol = 0.05;
constexpr double cov_det_absolute_tol = 1e-9;

// Each tier returns +1 (a survives), -1 (b survives), or 0 (indistinguishable, defer to next tier).

int compareByPriority(const TrackerSnapshot & a, const TrackerSnapshot & b)
{
  if (a.priority == b.priority) return 0;
  return a.priority < b.priority ? 1 : -1;  // lower TrackerType value outranks
}

int compareByFreshFullMeasurement(const TrackerSnapshot & a, const TrackerSnapshot & b)
{
  if (a.fully_measured_stale == b.fully_measured_stale) return 0;
  return a.fully_measured_stale ? -1 : 1;  // the fresh (non-stale) one survives
}

int compareByKnownProbability(const TrackerSnapshot & a, const TrackerSnapshot & b)
{
  const bool a_known = a.known_prob >= min_known_prob;
  const bool b_known = b.known_prob >= min_known_prob;
  if (a_known == b_known) return 0;
  return a_known ? 1 : -1;
}

int compareByChannelSupport(const TrackerSnapshot & a, const TrackerSnapshot & b)
{
  constexpr float support_buffer = 0.4f;
  if (a.channel_support > b.channel_support + support_buffer) return 1;
  if (b.channel_support > a.channel_support + support_buffer) return -1;
  return 0;  // comparable support → next tier
}

int compareByPositionCovariance(const TrackerSnapshot & a, const TrackerSnapshot & b)
{
  if (withinDeadband(a.cov_det, b.cov_det, cov_det_relative_tol, cov_det_absolute_tol)) return 0;
  return a.cov_det < b.cov_det ? 1 : -1;  // tighter position estimate survives
}

int compareByMeasurementCount(const TrackerSnapshot & a, const TrackerSnapshot & b)
{
  if (a.measurement_count == b.measurement_count) return 0;
  return a.measurement_count > b.measurement_count ? 1 : -1;
}

int compareByUuid(const TrackerSnapshot & a, const TrackerSnapshot & b)
{
  return a.uuid < b.uuid ? 1 : -1;  // total tie-break: distinct trackers always differ
}

}  // namespace

bool ensureConfident(TrackerSnapshot & snap, const DecisionContext & ctx)
{
  if (!snap.confident) {
    snap.confident = snap.tracker->isConfident(ctx.threshold_cache, ctx.ego_pose, ctx.time);
  }
  return *snap.confident;
}

bool ensurePublishConfident(TrackerSnapshot & snap, const DecisionContext & ctx)
{
  if (!snap.publish_confident) {
    snap.publish_confident =
      snap.tracker->isConfident(ctx.threshold_cache, ctx.ego_pose, std::nullopt);
  }
  return *snap.publish_confident;
}

const types::DynamicObject * ensureObject(TrackerSnapshot & snap, const rclcpp::Time & time)
{
  if (!snap.object_valid) {
    snap.object_valid = snap.tracker->getTrackedObject(time, snap.object);
  }
  return *snap.object_valid ? &snap.object : nullptr;
}

int compareForSurvival(TrackerSnapshot & a, TrackerSnapshot & b, const DecisionContext & ctx)
{
  if (const int r = compareByPriority(a, b)) return r;
  if (const int r = compareByFreshFullMeasurement(a, b)) return r;
  {
    const bool a_confident = ensureConfident(a, ctx);
    const bool b_confident = ensureConfident(b, ctx);
    if (a_confident != b_confident) return a_confident ? 1 : -1;
  }
  if (const int r = compareByKnownProbability(a, b)) return r;
  if (const int r = compareByChannelSupport(a, b)) return r;
  if (const int r = compareByPositionCovariance(a, b)) return r;
  if (const int r = compareByMeasurementCount(a, b)) return r;
  return compareByUuid(a, b);
}

int compareWinnerSubstance(TrackerSnapshot & a, TrackerSnapshot & b, const DecisionContext & ctx)
{
  if (const int r = compareByPriority(a, b)) return r;
  if (const int r = compareByFreshFullMeasurement(a, b)) return r;
  {
    const bool a_confident = ensureConfident(a, ctx);
    const bool b_confident = ensureConfident(b, ctx);
    if (a_confident != b_confident) return a_confident ? 1 : -1;
  }
  if (const int r = compareByKnownProbability(a, b)) return r;
  if (a.cov_det != b.cov_det) return a.cov_det < b.cov_det ? 1 : -1;  // tighter estimate survives
  return compareByMeasurementCount(a, b);
}

}  // namespace autoware::multi_object_tracker::detail
