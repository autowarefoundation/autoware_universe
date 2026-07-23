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

#include "autoware/multi_object_tracker/types.hpp"

#include <rclcpp/time.hpp>

#include <algorithm>
#include <cmath>

namespace autoware::multi_object_tracker
{

// Heading-sign belief with two separated evidence sources. Detection-yaw votes accumulate in a
// scalar OU-Kalman agreement estimate (+1 = sign correct, -1 = wrong) that relaxes toward an
// uninformed prior over time; the tracked longitudinal velocity contributes memorylessly at
// decision time, weighted by speed and velocity certainty. A confidently negative fused
// agreement triggers a 180° flip, and negation on flip provides hysteresis against oscillation.
// Class-independent tuning constants shared by every vehicle tracker.
struct OrientationSignBeliefParams
{
  double tau{3.0};                 // [s] evidence memory time constant
  double stationary_var{0.25};     // [-] variance of the uninformed agreement prior
  double r_yaw_available{0.5};     // [-] yaw-vote noise variance, sign known
  double r_yaw_sign_unknown{2.0};  // [-] yaw-vote noise variance, sign unknown
  double dead_zone{M_PI / 6.0};    // [rad] no-vote band around |yaw_diff| = 90 deg
  double r_vel{0.1};               // [-] velocity-evidence noise variance at the par speed
  double vote_vel_par{5.0 / 3.6};  // [m/s] speed where the velocity evidence saturates
  double vote_vel_var{1.0};        // [m^2/s^2] velocity variance where its certainty halves
  double flip_z{1.3};              // [-] posterior confidence gate (z-score) for flipping
  double flip_min_agreement{0.2};  // [-] fused-agreement magnitude floor for flipping
};

class OrientationSignBelief
{
public:
  using Params = OrientationSignBeliefParams;

  OrientationSignBelief(
    const types::OrientationAvailability initial_availability, const rclcpp::Time & time,
    const Params & params = Params{})
  : params_(params), variance_(params.stationary_var), last_update_time_(time)
  {
    // The seeding detection also seeds the tracker yaw, so it counts as one agreeing vote.
    if (initial_availability == types::OrientationAvailability::AVAILABLE) {
      correct(1.0, params_.r_yaw_available);
    } else if (initial_availability == types::OrientationAvailability::SIGN_UNKNOWN) {
      correct(1.0, params_.r_yaw_sign_unknown);
    }
    fused_agreement_ = agreement_;
    fused_variance_ = variance_;
  }

  // yaw_diff: normalized measurement yaw minus tracker yaw [rad]
  void vote(const rclcpp::Time & time, const double yaw_diff, const bool is_sign_known)
  {
    predictTo(time);
    // Near-perpendicular boxes carry axis ambiguity, not sign evidence.
    if (std::abs(std::abs(yaw_diff) - M_PI_2) < params_.dead_zone) return;
    const double z = std::abs(yaw_diff) < M_PI_2 ? 1.0 : -1.0;
    correct(z, is_sign_known ? params_.r_yaw_available : params_.r_yaw_sign_unknown);
  }

  // Fuses the accumulated agreement with the instantaneous velocity-sign evidence. The velocity
  // weight vanishes at standstill, saturates at vote_vel_par, and is discounted by the velocity
  // variance, so only sustained, well-estimated motion can override the yaw evidence. The flip
  // fires when the fused sign is wrong with high posterior confidence.
  bool shouldFlip(const rclcpp::Time & time, const double vel_long, const double vel_var)
  {
    predictTo(time);
    const double vel_ratio = vel_long / params_.vote_vel_par;
    const double z_vel = std::clamp(vel_ratio, -1.0, 1.0);
    const double info_vel = std::min(vel_ratio * vel_ratio, 1.0) / params_.r_vel *
                            params_.vote_vel_var / (params_.vote_vel_var + vel_var);
    fused_variance_ = 1.0 / (1.0 / variance_ + info_vel);
    fused_agreement_ = (agreement_ / variance_ + z_vel * info_vel) * fused_variance_;
    return fused_agreement_ + params_.flip_z * std::sqrt(fused_variance_) < 0.0 &&
           fused_agreement_ < -params_.flip_min_agreement;
  }

  // A 180° state flip turns every past disagreeing vote into an agreeing one.
  void onFlipped()
  {
    agreement_ = -agreement_;
    fused_agreement_ = -fused_agreement_;
  }

  double agreement() const { return agreement_; }
  double variance() const { return variance_; }
  // Yaw-vote agreement magnitude as a z-score.
  double confidence() const { return std::abs(agreement_) / std::sqrt(variance_); }
  double fusedAgreement() const { return fused_agreement_; }
  double fusedVariance() const { return fused_variance_; }

private:
  // OU relaxation toward the (0, stationary_var) prior over the elapsed time. Zero elapsed time
  // leaves the state untouched, so near-simultaneous votes fuse batch-equivalently and
  // contradicting detections of one frame compensate independent of processing order.
  void predictTo(const rclcpp::Time & time)
  {
    const double dt = (time - last_update_time_).seconds();
    if (dt <= 0.0) return;
    last_update_time_ = time;
    const double alpha = std::exp(-dt / params_.tau);
    agreement_ *= alpha;
    variance_ = alpha * alpha * variance_ + (1.0 - alpha * alpha) * params_.stationary_var;
  }

  // Scalar Kalman update of the agreement estimate.
  void correct(const double z, const double r)
  {
    const double gain = variance_ / (variance_ + r);
    agreement_ += gain * (z - agreement_);
    variance_ *= (1.0 - gain);
  }

  Params params_;
  double agreement_{0.0};
  double variance_{0.0};
  rclcpp::Time last_update_time_;
  double fused_agreement_{0.0};
  double fused_variance_{0.0};
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__UPDATE__ORIENTATION_SIGN_BELIEF_HPP_
