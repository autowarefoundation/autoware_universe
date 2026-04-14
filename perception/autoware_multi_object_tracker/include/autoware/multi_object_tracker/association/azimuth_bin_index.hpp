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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__AZIMUTH_BIN_INDEX_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__AZIMUTH_BIN_INDEX_HPP_

#include "autoware/multi_object_tracker/association/scoring/polar_assignment_scoring.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace autoware::multi_object_tracker
{

/// Per-bin tracker record: index + range of the near face.
struct AzimuthBinEntry
{
  size_t tracker_idx;
  float r_min;  // near-face range [m]; float precision is sufficient (< 1 cm error at 300 m)
};

/// 1-D azimuth bin index with soft radial pre-filtering.

class AzimuthBinIndex
{
public:
  static constexpr int kNumBins = 24;                         // 15° per bin, full 360°
  static constexpr double kBinWidth = 2.0 * M_PI / kNumBins;  // [rad]

  void clear()
  {
    for (auto & bin : bins_) bin.clear();
  }

  /// Register @p tracker_idx to every bin its @p interval covers.
  /// The +1 guard ensures boundary trackers are always findable by find().
  void add(const polar_scoring::AzimuthInterval & interval, size_t tracker_idx, double r_min)
  {
    const int n =
      std::min(static_cast<int>(std::ceil(2.0 * interval.half_span / kBinWidth)) + 1, kNumBins);
    const int start = angleToBin(interval.center - interval.half_span);
    const AzimuthBinEntry entry{tracker_idx, static_cast<float>(r_min)};
    for (int i = 0; i < n; ++i) {
      bins_[(start + i) % kNumBins].push_back(entry);
    }
  }

  /// Return unique tracker indices whose bins overlap @p interval and whose
  /// stored r_min passes the soft radial pre-filter for @p r_min_query.
  std::vector<size_t> find(
    const polar_scoring::AzimuthInterval & interval, double r_min_query) const
  {
    // Exact bin count — no +1, relying on add()'s guard for boundary coverage.
    const int n = std::max(1, static_cast<int>(std::ceil(2.0 * interval.half_span / kBinWidth)));
    const int start = angleToBin(interval.center - interval.half_span);

    std::vector<size_t> candidates;
    for (int i = 0; i < n; ++i) {
      for (const auto & entry : bins_[(start + i) % kNumBins]) {
        if (radialPrePass(r_min_query, static_cast<double>(entry.r_min))) {
          candidates.push_back(entry.tracker_idx);
        }
      }
    }
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
    return candidates;
  }

private:
  // Sigma formula mirrors radialCompatibility() in polar_assignment_scoring.cpp.
  static constexpr double kSigmaBase = 2.0;   // [m]
  static constexpr double kSigmaRate = 0.03;  // [m/m]
  // Gate factor: -ln(0.05) ≈ 3.0.  Skip when exp(-gap/sigma) < 0.05,
  // i.e., gap > 3σ.  Always above the 2 m hard gate, so no true match is lost.
  static constexpr double kGateFactor = 3.0;

  std::array<std::vector<AzimuthBinEntry>, kNumBins> bins_;

  /// Returns true when the radial pre-filter passes (candidate should be kept).
  /// Equivalent to exp(-r_gap/sigma) >= 0.05, computed without std::exp.
  static bool radialPrePass(double r_query, double r_cand)
  {
    const double r_gap = std::abs(r_query - r_cand);
    const double sigma = kSigmaBase + kSigmaRate * std::max(r_query, r_cand);
    return r_gap <= kGateFactor * sigma;
  }

  /// Map any angle [rad] to a bin index in [0, kNumBins).
  /// Shifts [-π, π) → [0, 2π) by adding π before bucketing.
  static int angleToBin(double angle)
  {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) a += 2.0 * M_PI;
    const int bin = static_cast<int>(a / kBinWidth);
    return std::min(bin, kNumBins - 1);
  }
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__AZIMUTH_BIN_INDEX_HPP_
