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
  };

  std::vector<SlowSpeedInfo> slow_speed_ranges;
  std::vector<size_t> stop_points;

  [[nodiscard]] bool is_slowing_down_to_a_stop(const SlowSpeedInfo & info) const
  {
    if (stop_points.empty()) {
      return false;
    }
    return std::any_of(stop_points.begin(), stop_points.end(), [&info](size_t stop_index) {
      return stop_index >= info.start_index && stop_index <= info.end_index;
    });
  }

  // When slow speed ranges coincide with stop points, we consider that the vehicle is slowing down
  // to a stop. This function updates the slow down ranges and stop points accordingly, so that
  // plugins can use this information to adjust their behavior (e.g., apply more aggressive
  // smoothing when slowing down to a stop).
  void update_slow_down_ranges_and_stop_points()
  {
    slow_down_ranges.clear();
    std::vector<size_t> new_stop_points;
    for (const auto & range : slow_speed_ranges) {
      if (is_slowing_down_to_a_stop(range)) {
        slow_down_ranges.push_back(range);
        new_stop_points.push_back(range.end_index);
      }
    }
    stop_points = new_stop_points;
  }

  [[nodiscard]] bool is_stop_point(size_t index) const
  {
    return std::find(stop_points.begin(), stop_points.end(), index) != stop_points.end();
  }

  [[nodiscard]] const std::vector<SlowSpeedInfo> & get_slow_down_ranges() const
  {
    return slow_down_ranges;
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
