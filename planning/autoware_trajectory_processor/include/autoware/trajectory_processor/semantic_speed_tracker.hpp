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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__SEMANTIC_SPEED_TRACKER_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__SEMANTIC_SPEED_TRACKER_HPP_

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <vector>

namespace autoware::trajectory_processor
{

/// @brief Tracks detected stop approaches across a trajectory processor plugin pipeline.
class SemanticSpeedTracker
{
public:
  /// @brief Arc-length and index bounds of a detected slow-down range.
  struct SlowSpeedInfo
  {
    std::size_t start_index{0};
    std::size_t end_index{0};
    double start_s_m{0.0};
    double end_s_m{0.0};
  };

  /// @brief Remap tracked range indices to a trajectory with new arc-length samples.
  void remap_to_trajectory(const std::vector<double> & new_arc_lengths)
  {
    if (new_arc_lengths.empty() || slow_down_ranges_.empty()) {
      return;
    }

    const double max_s = new_arc_lengths.back();

    auto find_nearest_index = [&](double target_s) -> std::size_t {
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
               ? static_cast<std::size_t>(std::distance(new_arc_lengths.begin(), prev_it))
               : static_cast<std::size_t>(std::distance(new_arc_lengths.begin(), it));
    };

    for (auto & range : slow_down_ranges_) {
      range.start_index = find_nearest_index(range.start_s_m);
      range.end_index = find_nearest_index(range.end_s_m);
    }
  }

  /// @brief Return all tracked slow-down ranges.
  [[nodiscard]] const std::vector<SlowSpeedInfo> & get_slow_down_ranges() const
  {
    return slow_down_ranges_;
  }

  /// @brief Add a completed stop-approach range.
  void add_stop_approach(const SlowSpeedInfo & info) { slow_down_ranges_.push_back(info); }

  /// @brief Remove all completed stop-approach ranges.
  void clear_stop_approaches() { slow_down_ranges_.clear(); }

  /// @brief Stage a trajectory index as a possible stop point.
  void add_stop_candidate(const std::size_t idx) { stop_point_candidates_.push_back(idx); }

  /// @brief Return all staged stop candidates and clear the staging area.
  std::vector<std::size_t> take_stop_point_candidates()
  {
    std::vector<std::size_t> result;
    result.swap(stop_point_candidates_);
    return result;
  }

private:
  std::vector<std::size_t> stop_point_candidates_;
  std::vector<SlowSpeedInfo> slow_down_ranges_;
};

}  // namespace autoware::trajectory_processor

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__SEMANTIC_SPEED_TRACKER_HPP_
