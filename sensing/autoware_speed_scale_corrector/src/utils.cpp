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

#include "autoware/speed_scale_corrector/utils.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace autoware::speed_scale_corrector
{

std::vector<double> smooth_gaussian(
  const std::vector<double> & times, const std::vector<double> & values, double sigma,
  double cutoff)
{
  // Basic checks
  if (times.size() != values.size()) {
    return {};
  }
  const size_t n = times.size();
  if (n == 0 || sigma <= 0.0) {
    return values;  // Nothing to do
  }

  // Verify monotonic non-decreasing times
  for (size_t i = 1; i < n; ++i) {
    if (times[i] < times[i - 1]) {
      return {};
    }
  }

  const double radius = cutoff * sigma;
  const double inv_two_sigma2 = 1.0 / (2.0 * sigma * sigma);

  std::vector<double> out(n);
  size_t left = 0;   // left index of active window (inclusive)
  size_t right = 0;  // right index of active window (exclusive)

  for (size_t i = 0; i < n; ++i) {
    const double tc = times[i];

    // Move left to the first index with times[left] >= tc - radius
    while (left < n && times[left] < tc - radius) {
      ++left;
    }
    // Move right to one past the last index with times[right] <= tc + radius
    while (right < n && times[right] <= tc + radius) {
      ++right;
    }

    // Accumulate Gaussian-weighted sum within [left, right)
    double w_sum = 0.0;
    double x_sum = 0.0;
    for (size_t j = left; j < right; ++j) {
      const double dt = times[j] - tc;
      // Gaussian kernel weight
      const double w = std::exp(-(dt * dt) * inv_two_sigma2);
      w_sum += w;
      x_sum += w * values[j];
    }

    // Fallback if numerical underflow makes w_sum ~ 0
    out[i] = (w_sum > 0.0) ? (x_sum / w_sum) : values[i];
  }

  return out;
}

std::optional<std::pair<double, double>> intersect_intervals(
  const std::vector<std::pair<double, double>> & intervals)
{
  if (intervals.empty()) {
    return std::nullopt;
  }

  double left = intervals[0].first;
  double right = intervals[0].second;

  for (size_t i = 1; i < intervals.size(); ++i) {
    left = std::max(left, intervals[i].first);
    right = std::min(right, intervals[i].second);
    if (left > right) {
      return std::nullopt;  // no common intersection
    }
  }

  return std::make_pair(left, right);
}

std::vector<double> arange(double start, double end, double interval)
{
  if (interval <= 0.0) {
    return {};
  }

  std::vector<double> result;
  if (start >= end) {
    return result;  // Empty vector if start >= end
  }

  // Calculate the number of elements
  const auto num_elements = static_cast<size_t>(std::ceil((end - start) / interval));
  result.reserve(num_elements);

  for (double value = start; value < end; value += interval) {
    result.emplace_back(value);
  }

  return result;
}

}  // namespace autoware::speed_scale_corrector
