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

#ifndef AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CALIBRATION_STATUS_CLASSIFIER_FILTERS_HPP_
#define AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CALIBRATION_STATUS_CLASSIFIER_FILTERS_HPP_

#include "autoware/calibration_status_classifier/filter.hpp"

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <utility>
#include <vector>

namespace autoware::calibration_status_classifier
{

/// @brief Aggregated result of evaluating all prerequisite filters
struct FiltersResult
{
  bool all_passed{true};
  std::vector<FilterResult> filter_results;
};

/**
 * @brief Thread-safe container for prerequisite filters
 *
 * Manages a collection of Filter instances with thread-safe update (exclusive lock)
 * and evaluate (shared lock) operations. Filters are stored as unique_ptr<Filter>
 * and accessed by type via dynamic_cast.
 */
class CalibrationStatusClassifierFilters
{
public:
  CalibrationStatusClassifierFilters();
  ~CalibrationStatusClassifierFilters();

  CalibrationStatusClassifierFilters(CalibrationStatusClassifierFilters &&) noexcept;
  CalibrationStatusClassifierFilters & operator=(CalibrationStatusClassifierFilters &&) noexcept;
  CalibrationStatusClassifierFilters(const CalibrationStatusClassifierFilters &) = delete;
  CalibrationStatusClassifierFilters & operator=(const CalibrationStatusClassifierFilters &) =
    delete;

  template <typename... Filters>
  explicit CalibrationStatusClassifierFilters(std::unique_ptr<Filters> &&... filters)
  {
    filters_.reserve(sizeof...(Filters));
    (filters_.emplace_back(std::move(filters)), ...);
  }

  explicit CalibrationStatusClassifierFilters(std::vector<std::unique_ptr<Filter>> filters);

  /// Thread-safe update: exclusive lock (writes filter state)
  template <typename FilterT, typename... Args>
  bool update(Args &&... args)
  {
    std::unique_lock lock(mutex_);
    if (auto * f = get_impl<FilterT>()) {
      f->update(std::forward<Args>(args)...);
      return true;
    }
    return false;
  }

  /// Thread-safe evaluate: shared lock (read-only access to filter states)
  [[nodiscard]] FiltersResult evaluate(double stamp) const;

  /// Non-locked accessor - use only during single-threaded setup (e.g. constructor)
  template <typename T>
  [[nodiscard]] T * get()
  {
    return get_impl<T>();
  }

  template <typename T>
  [[nodiscard]] const T * get() const
  {
    return get_impl<T>();
  }

private:
  template <typename T>
  T * get_impl()
  {
    for (auto & f : filters_) {
      if (auto * typed = dynamic_cast<T *>(f.get())) return typed;
    }
    return nullptr;
  }

  template <typename T>
  const T * get_impl() const
  {
    for (const auto & f : filters_) {
      if (const auto * typed = dynamic_cast<const T *>(f.get())) return typed;
    }
    return nullptr;
  }

  std::vector<std::unique_ptr<Filter>> filters_;
  mutable std::shared_mutex mutex_;
};

}  // namespace autoware::calibration_status_classifier

#endif  // AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CALIBRATION_STATUS_CLASSIFIER_FILTERS_HPP_
