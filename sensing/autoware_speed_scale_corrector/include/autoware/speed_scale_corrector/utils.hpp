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

#ifndef AUTOWARE__SPEED_SCALE_CORRECTOR__UTILS_HPP_
#define AUTOWARE__SPEED_SCALE_CORRECTOR__UTILS_HPP_

#include <sensor_msgs/msg/imu.hpp>

#include <cmath>
#include <optional>
#include <vector>

namespace autoware::speed_scale_corrector
{

/**
 * @brief Apply Gaussian kernel smoothing for irregular time samples
 *
 * This function applies Gaussian kernel smoothing to a time series with irregular sampling.
 * Each output value is computed as a weighted average of nearby input values, where the
 * weights follow a Gaussian distribution centered at the target time point.
 *
 * @param times Time stamps (must be monotonically non-decreasing)
 * @param values Signal values corresponding to each time stamp
 * @param sigma Bandwidth parameter in seconds (must be > 0)
 * @param cutoff Window radius as multiple of sigma (e.g., 3.0 means ±3σ window)
 * @return std::vector<double> Smoothed signal values
 *
 * @throw std::invalid_argument If times and values have different sizes
 * @throw std::invalid_argument If times are not monotonically non-decreasing
 *
 * @note If sigma <= 0 or input is empty, returns the original values unchanged
 * @note Uses efficient sliding window approach for O(n) performance on sorted data
 */
std::vector<double> smooth_gaussian(
  const std::vector<double> & times, const std::vector<double> & values, double sigma,
  double cutoff = 3.0);

/**
 * @brief Find the intersection of multiple intervals
 *
 * This function computes the intersection of a collection of intervals [left, right].
 * The intersection is the largest interval that is contained within all input intervals.
 * If no common intersection exists, returns std::nullopt.
 *
 * @param intervals Vector of intervals, where each interval is represented as a pair (left, right)
 * @return std::optional<std::pair<double, double>> The intersection interval if it exists,
 * std::nullopt otherwise
 *
 * @note If the input vector is empty, returns std::nullopt
 * @note Each interval is assumed to be valid (i.e., left <= right)
 * @note The intersection is computed as [max(all_lefts), min(all_rights)]
 *
 * @par Example:
 * @code
 * std::vector<std::pair<double, double>> intervals = {{1.0, 5.0}, {3.0, 7.0}, {2.0, 4.0}};
 * auto result = intersect_intervals(intervals);
 * // result will be {3.0, 4.0} since [3.0,4.0] is the common intersection
 * @endcode
 */
std::optional<std::pair<double, double>> intersect_intervals(
  const std::vector<std::pair<double, double>> & intervals);

/**
 * @brief Generate a sequence of evenly spaced values over a specified interval
 *
 * This function creates a vector of double values starting from 'start' and incrementing
 * by 'interval' until reaching (but not exceeding) 'end'. Similar to NumPy's arange
 * function or MATLAB's colon operator.
 *
 * @param start Starting value of the sequence (inclusive)
 * @param end End value of the sequence (exclusive)
 * @param interval Step size between consecutive values (must be > 0)
 * @return std::vector<double> Vector containing the generated sequence
 *
 * @throw std::invalid_argument If interval <= 0
 *
 * @note If start >= end, returns an empty vector
 * @note The function reserves memory based on the calculated number of elements for efficiency
 *
 * @par Example:
 * @code
 * auto result = arange(0.0, 5.0, 1.0);
 * // result will be {0.0, 1.0, 2.0, 3.0, 4.0}
 *
 * auto result2 = arange(1.5, 4.0, 0.5);
 * // result2 will be {1.5, 2.0, 2.5, 3.0, 3.5}
 * @endcode
 */
std::vector<double> arange(double start, double end, double interval);

}  // namespace autoware::speed_scale_corrector

#endif  // AUTOWARE__SPEED_SCALE_CORRECTOR__UTILS_HPP_
