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

#include "autoware/ptv3/detection_class_remapper.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace autoware::ptv3
{

void DetectionClassRemapper::set_parameters(
  const std::vector<std::int64_t> & allow_remapping_by_area_matrix,
  const std::vector<double> & min_area_matrix, const std::vector<double> & max_area_matrix,
  const std::size_t num_detection_classes)
{
  if (
    allow_remapping_by_area_matrix.size() != min_area_matrix.size() ||
    allow_remapping_by_area_matrix.size() != max_area_matrix.size()) {
    throw std::runtime_error("Detection class remapper matrices must have the same size.");
  }

  const auto matrix_size = min_area_matrix.size();
  const auto expected_matrix_size = num_detection_classes * num_detection_classes;
  if (matrix_size != expected_matrix_size) {
    throw std::runtime_error(
      "Detection class remapper matrix size must match detection3d.class_names size squared.");
  }
  if (!std::all_of(
        allow_remapping_by_area_matrix.begin(), allow_remapping_by_area_matrix.end(),
        [](const std::int64_t value) { return value == 0 || value == 1; })) {
    throw std::runtime_error("Detection class remapper allow matrix values must be 0 or 1.");
  }
  num_labels_ = static_cast<int>(num_detection_classes);

  Eigen::Map<const Eigen::Matrix<std::int64_t, Eigen::Dynamic, Eigen::Dynamic>>
    allow_remapping_by_area_matrix_tmp(
      allow_remapping_by_area_matrix.data(), num_labels_, num_labels_);
  allow_remapping_by_area_matrix_ = allow_remapping_by_area_matrix_tmp.transpose().cast<bool>();

  Eigen::Map<const Eigen::MatrixXd> min_area_matrix_tmp(
    min_area_matrix.data(), num_labels_, num_labels_);
  min_area_matrix_ = min_area_matrix_tmp.transpose();

  Eigen::Map<const Eigen::MatrixXd> max_area_matrix_tmp(
    max_area_matrix.data(), num_labels_, num_labels_);
  max_area_matrix_ = max_area_matrix_tmp.transpose();

  min_area_matrix_ = min_area_matrix_.unaryExpr(
    [](double value) { return std::isfinite(value) ? value : std::numeric_limits<double>::max(); });
  max_area_matrix_ = max_area_matrix_.unaryExpr(
    [](double value) { return std::isfinite(value) ? value : std::numeric_limits<double>::max(); });
}

void DetectionClassRemapper::map_classes(std::vector<Box3D> & boxes)
{
  for (auto & box : boxes) {
    if (box.label < 0 || box.label >= num_labels_) {
      continue;
    }
    const float bev_area = box.length * box.width;
    for (int target_label = 0; target_label < num_labels_; ++target_label) {
      if (
        allow_remapping_by_area_matrix_(box.label, target_label) &&
        bev_area >= min_area_matrix_(box.label, target_label) &&
        bev_area <= max_area_matrix_(box.label, target_label)) {
        box.label = target_label;
        break;
      }
    }
  }
}

}  // namespace autoware::ptv3
