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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__UPDATE__ORIENTATION_SIGN_BELIEF_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__UPDATE__ORIENTATION_SIGN_BELIEF_HPP_

#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::multi_object_tracker
{

// Log-odds belief that the tracker's heading sign is correct. Each detection votes by its raw yaw
// sign; a strongly negative belief triggers a 180° flip, and negation on flip provides hysteresis
// against oscillation.
class OrientationSignBelief
{
public:
  OrientationSignBelief(
    const object_model::OrientationSignBelief & params,
    const types::OrientationAvailability initial_availability)
  : params_(params)
  {
    // The seeding detection also seeds the tracker yaw, so it counts as one agreeing vote.
    if (initial_availability == types::OrientationAvailability::AVAILABLE) {
      log_odds_ = params_.vote_available;
    } else if (initial_availability == types::OrientationAvailability::SIGN_UNKNOWN) {
      log_odds_ = params_.vote_sign_unknown;
    }
  }

  // yaw_diff: normalized measurement yaw minus tracker yaw [rad]
  void vote(const double yaw_diff, const bool is_sign_known)
  {
    // Near-perpendicular boxes carry axis ambiguity, not sign evidence.
    if (std::abs(std::abs(yaw_diff) - M_PI_2) < params_.dead_zone) return;
    const double weight = is_sign_known ? params_.vote_available : params_.vote_sign_unknown;
    log_odds_ += std::abs(yaw_diff) < M_PI_2 ? weight : -weight;
    log_odds_ = std::clamp(log_odds_, -params_.log_odds_max, params_.log_odds_max);
  }

  bool shouldFlip() const { return log_odds_ < -params_.flip_threshold; }

  // A 180° state flip turns every past disagreeing vote into an agreeing one.
  void onFlipped() { log_odds_ = -log_odds_; }

  double logOdds() const { return log_odds_; }

private:
  object_model::OrientationSignBelief params_;
  double log_odds_{0.0};
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__UPDATE__ORIENTATION_SIGN_BELIEF_HPP_
