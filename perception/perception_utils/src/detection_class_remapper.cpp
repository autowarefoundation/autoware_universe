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

#include "perception_utils/detection_class_remapper.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace perception_utils
{

void DetectionClassRemapper::setParameters(
  const std::vector<std::int64_t> & allow_remapping_by_area_matrix,
  const std::vector<double> & min_area_matrix, const std::vector<double> & max_area_matrix)
{
  if (
    allow_remapping_by_area_matrix.size() != min_area_matrix.size() ||
    allow_remapping_by_area_matrix.size() != max_area_matrix.size()) {
    throw std::invalid_argument("Class remapping matrices must have equal sizes.");
  }

  const auto num_labels = static_cast<std::size_t>(std::sqrt(min_area_matrix.size()));
  if (num_labels == 0 || num_labels * num_labels != min_area_matrix.size()) {
    throw std::invalid_argument("Class remapping matrices must be non-empty and square.");
  }

  num_labels_ = num_labels;
  allow_remapping_by_area_matrix_.assign(
    allow_remapping_by_area_matrix.begin(), allow_remapping_by_area_matrix.end());
  min_area_matrix_ = min_area_matrix;
  max_area_matrix_ = max_area_matrix;

  const auto replace_non_finite = [](double value) {
    return std::isfinite(value) ? value : std::numeric_limits<double>::max();
  };
  std::transform(
    min_area_matrix_.begin(), min_area_matrix_.end(), min_area_matrix_.begin(), replace_non_finite);
  std::transform(
    max_area_matrix_.begin(), max_area_matrix_.end(), max_area_matrix_.begin(), replace_non_finite);
}

std::size_t DetectionClassRemapper::matrixIndex(
  const std::size_t source_label, const std::size_t destination_label) const
{
  return source_label * num_labels_ + destination_label;
}

void DetectionClassRemapper::mapClasses(autoware_perception_msgs::msg::DetectedObjects & msg) const
{
  for (auto & object : msg.objects) {
    const float bev_area = object.shape.dimensions.x * object.shape.dimensions.y;

    for (auto & classification : object.classification) {
      auto & label = classification.label;
      const auto source_label = static_cast<std::size_t>(label);
      if (source_label >= num_labels_) {
        continue;
      }

      for (std::size_t destination_label = 0; destination_label < num_labels_;
           ++destination_label) {
        const auto index = matrixIndex(source_label, destination_label);
        if (
          allow_remapping_by_area_matrix_.at(index) && bev_area >= min_area_matrix_.at(index) &&
          bev_area <= max_area_matrix_.at(index)) {
          label = static_cast<std::uint8_t>(destination_label);
          break;
        }
      }
    }
  }
}

}  // namespace perception_utils
