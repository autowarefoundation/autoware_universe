// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_STRUCTS_HPP_
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_STRUCTS_HPP_
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <vector>

namespace autoware::trajectory_optimizer
{
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

struct InitialMotion
{
  double speed_mps{0.0};
  double acc_mps2{0.0};
};

// Runtime data struct - contains vehicle state updated each cycle from topics
// This is NOT configuration, it's runtime state passed to plugins
struct TrajectoryOptimizerData
{
  Odometry current_odometry;
  AccelWithCovarianceStamped current_acceleration;
};

// Tracks detected stop approaches in the trajectory for use across plugins.
// A stop approach is a deceleration zone leading to a full stop: it has a start index
// (onset of deceleration), an end index (stop point), and arc length coordinates for
// remapping after resampling.
struct SemanticSpeedTracker
{
public:
  struct SlowSpeedInfo
  {
    size_t start_index{0};
    size_t end_index{0};
    double start_s_m{0.0};
    double end_s_m{0.0};
  };

  void remap_to_trajectory(const std::vector<double> & new_arc_lengths)
  {
    if (new_arc_lengths.empty() || slow_down_ranges.empty()) {
      return;
    }

    const double max_s = new_arc_lengths.back();

    auto find_nearest_index = [&](double target_s) -> size_t {
      target_s = std::max(0.0, std::min(target_s, max_s));
      const auto it = std::lower_bound(new_arc_lengths.begin(), new_arc_lengths.end(), target_s);
      if (it == new_arc_lengths.end()) {
        return new_arc_lengths.size() - 1;
      }
      if (it == new_arc_lengths.begin()) {
        return 0;
      }
      const auto prev_it = std::prev(it);
      return (target_s - *prev_it <= *it - target_s)
               ? static_cast<size_t>(std::distance(new_arc_lengths.begin(), prev_it))
               : static_cast<size_t>(std::distance(new_arc_lengths.begin(), it));
    };

    for (auto & range : slow_down_ranges) {
      range.start_index = find_nearest_index(range.start_s_m);
      range.end_index = find_nearest_index(range.end_s_m);
    }
  }

  [[nodiscard]] const std::vector<SlowSpeedInfo> & get_slow_down_ranges() const
  {
    return slow_down_ranges;
  }

  void add_stop_approach(const SlowSpeedInfo & info) { slow_down_ranges.push_back(info); }

  void clear_stop_approaches() { slow_down_ranges.clear(); }

  // Staging area: detection functions push candidate stop indices here before
  // build_stop_approach_ranges() processes them into slow_down_ranges.
  std::vector<size_t> stop_point_candidates;

private:
  std::vector<SlowSpeedInfo> slow_down_ranges;
};

// Main node parameters struct - contains only plugin activation flags
// Plugin-specific parameters are managed by each plugin independently
struct TrajectoryOptimizerParams
{
  bool use_akima_spline_interpolation{false};
  bool use_eb_smoother{false};
  bool use_qp_smoother{false};
  bool use_trajectory_point_fixer{false};
  bool use_velocity_optimizer{false};
  bool use_trajectory_extender{false};
  bool use_kinematic_feasibility_enforcer{false};
  bool use_mpt_optimizer{false};
};
}  // namespace autoware::trajectory_optimizer
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_STRUCTS_HPP_
