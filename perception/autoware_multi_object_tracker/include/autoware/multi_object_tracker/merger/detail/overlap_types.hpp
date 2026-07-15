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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__MERGER__DETAIL__OVERLAP_TYPES_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__MERGER__DETAIL__OVERLAP_TYPES_HPP_

#include "autoware/multi_object_tracker/association/adaptive_threshold_cache.hpp"
#include "autoware/multi_object_tracker/configurations.hpp"
#include "autoware/multi_object_tracker/tracker/trackers/tracker_base.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::multi_object_tracker::detail
{

// Per-tracker state captured once per merge cycle; pair decisions read snapshots, not the live
// tracker, so the outcome is independent of list order.
struct TrackerSnapshot
{
  std::shared_ptr<Tracker> tracker;
  geometry_msgs::msg::Point position;
  double yaw{0.0};  // heading [rad]
  classes::Label label{classes::Label::UNKNOWN};
  bool is_unknown{true};
  int priority{0};
  float known_prob{0.0f};
  double cov_det{0.0};
  int measurement_count{0};
  bool fully_measured_stale{true};
  std::array<uint8_t, 16> uuid{};
  std::vector<types::ExistenceProbability> existence_probs;
  // Filled lazily for trackers that appear in a gated pair.
  std::optional<bool> confident;
  std::optional<bool> object_valid;
  types::DynamicObject object;
};

struct DecisionContext
{
  const AdaptiveThresholdCache & threshold_cache;
  const std::optional<geometry_msgs::msg::Pose> & ego_pose;
  rclcpp::Time time;
  const TrackerOverlapManagerConfig & config;
};

// True when |a - b| is within a relative-or-absolute tolerance band.
inline bool withinDeadband(
  const double a, const double b, const double relative_tol, const double absolute_tol)
{
  const double diff = std::abs(a - b);
  return diff <= std::max(absolute_tol, relative_tol * std::max(std::abs(a), std::abs(b)));
}

}  // namespace autoware::multi_object_tracker::detail

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__MERGER__DETAIL__OVERLAP_TYPES_HPP_
