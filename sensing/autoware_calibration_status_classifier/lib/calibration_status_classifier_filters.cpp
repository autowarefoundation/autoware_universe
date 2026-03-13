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

#include "autoware/calibration_status_classifier/calibration_status_classifier_filters.hpp"

#include <shared_mutex>
#include <utility>
#include <vector>

namespace autoware::calibration_status_classifier
{

CalibrationStatusClassifierFilters::CalibrationStatusClassifierFilters() = default;
CalibrationStatusClassifierFilters::~CalibrationStatusClassifierFilters() = default;

CalibrationStatusClassifierFilters::CalibrationStatusClassifierFilters(
  CalibrationStatusClassifierFilters && other) noexcept
: filters_(std::move(other.filters_))
{
}

CalibrationStatusClassifierFilters & CalibrationStatusClassifierFilters::operator=(
  CalibrationStatusClassifierFilters && other) noexcept
{
  if (this != &other) {
    filters_ = std::move(other.filters_);
  }
  return *this;
}

CalibrationStatusClassifierFilters::CalibrationStatusClassifierFilters(
  std::vector<std::unique_ptr<Filter>> filters)
: filters_(std::move(filters))
{
}

FiltersResult CalibrationStatusClassifierFilters::evaluate(double stamp) const
{
  std::shared_lock lock(mutex_);
  FiltersResult result;
  result.all_passed = true;
  result.filter_results.reserve(filters_.size());
  for (const auto & filter : filters_) {
    auto fr = filter->get_result(stamp);
    if (!fr.is_passed) {
      result.all_passed = false;
    }
    result.filter_results.push_back(std::move(fr));
  }
  return result;
}

}  // namespace autoware::calibration_status_classifier
