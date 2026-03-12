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

// Struct that tracks model-predicted intentions for slow down or stopping of the ego vehicle across
// multiple plugins. This can be used to coordinate between plugins and ensure consistent behavior
// when the ego vehicle is predicted to slow down or stop. Point ranges and durations can be tracked
// to determine when to apply certain optimizations or modifications. Stops are determined by a
// single point (there can be several stop points in a single trajectory though), while slow downs
// are tracked by a vector with ranges of points and duration.
struct SemanticSpeedTracker
{
public:
  struct SlowSpeedInfo
  {
    size_t start_index{0};
    size_t end_index{0};
    double duration_s{0.0};
    double start_s_m{0.0};
    double end_s_m{0.0};
    bool is_stop_approach{false};
  };

  std::vector<size_t> stop_points;

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

    stop_points.clear();
    for (auto & range : slow_down_ranges) {
      range.start_index = find_nearest_index(range.start_s_m);
      range.end_index = find_nearest_index(range.end_s_m);
      stop_points.push_back(range.end_index);
    }
  }

  [[nodiscard]] bool is_stop_point(size_t index) const
  {
    return std::find(stop_points.begin(), stop_points.end(), index) != stop_points.end();
  }

  [[nodiscard]] const std::vector<SlowSpeedInfo> & get_slow_down_ranges() const
  {
    return slow_down_ranges;
  }

  void add_stop_approach(const SlowSpeedInfo & info)
  {
    slow_down_ranges.push_back(info);
    stop_points.push_back(info.end_index);
  }

  void clear_stop_approaches()
  {
    slow_down_ranges.clear();
    stop_points.clear();
  }

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
