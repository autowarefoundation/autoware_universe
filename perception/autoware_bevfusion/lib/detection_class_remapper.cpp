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

#include <autoware/bevfusion/detection_class_remapper.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::bevfusion
{

namespace
{

constexpr char kAllowRemappingByAreaMatrix[] = "allow_remapping_by_area_matrix";
constexpr char kMinAreaMatrix[] = "min_area_matrix";
constexpr char kMaxAreaMatrix[] = "max_area_matrix";

void validateSameSize(
  const std::size_t lhs_size, const std::size_t rhs_size, const char * lhs_name,
  const char * rhs_name)
{
  if (lhs_size != rhs_size) {
    throw std::invalid_argument(
      std::string(lhs_name) + " and " + rhs_name + " must have the same size");
  }
}

std::size_t validateNonEmptyMatrixSize(const std::size_t matrix_size)
{
  if (matrix_size == 0U) {
    throw std::invalid_argument("Detection class remapper matrices must not be empty");
  }

  return matrix_size;
}

int validateAndGetNumLabels(
  const std::vector<std::int64_t> & allow_remapping_by_area_matrix,
  const std::vector<double> & min_area_matrix, const std::vector<double> & max_area_matrix)
{
  const auto matrix_size = validateNonEmptyMatrixSize(allow_remapping_by_area_matrix.size());
  validateSameSize(
    matrix_size, min_area_matrix.size(), kAllowRemappingByAreaMatrix, kMinAreaMatrix);
  validateSameSize(
    matrix_size, max_area_matrix.size(), kAllowRemappingByAreaMatrix, kMaxAreaMatrix);

  const auto num_labels = static_cast<int>(std::sqrt(matrix_size));
  if (static_cast<std::size_t>(num_labels) * static_cast<std::size_t>(num_labels) != matrix_size) {
    throw std::invalid_argument("Detection class remapper matrices must define a square matrix");
  }

  return num_labels;
}

struct RemapContext
{
  int num_labels;
  const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> & allow_remapping_by_area_matrix;
  const Eigen::MatrixXd & min_area_matrix;
  const Eigen::MatrixXd & max_area_matrix;

  bool shouldRemapLabel(
    const int source_label, const int target_label, const float bev_area) const
  {
    return allow_remapping_by_area_matrix(source_label, target_label) &&
           bev_area >= min_area_matrix(source_label, target_label) &&
           bev_area <= max_area_matrix(source_label, target_label);
  }

  std::uint8_t remapLabel(const std::uint8_t label, const float bev_area) const
  {
    if (static_cast<int>(label) >= num_labels) {
      return label;
    }

    for (int target_label = 0; target_label < num_labels; ++target_label) {
      if (shouldRemapLabel(label, target_label, bev_area)) {
        return static_cast<std::uint8_t>(target_label);
      }
    }

    return label;
  }
};

}  // namespace

void DetectionClassRemapper::setParameters(
  const std::vector<std::int64_t> & allow_remapping_by_area_matrix,
  const std::vector<double> & min_area_matrix, const std::vector<double> & max_area_matrix)
{
  num_labels_ =
    validateAndGetNumLabels(allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

  Eigen::Map<const Eigen::Matrix<std::int64_t, Eigen::Dynamic, Eigen::Dynamic>>
    allow_remapping_by_area_matrix_tmp(
      allow_remapping_by_area_matrix.data(), num_labels_, num_labels_);
  allow_remapping_by_area_matrix_ = allow_remapping_by_area_matrix_tmp.transpose()
                                      .cast<bool>();  // Eigen is column major by default

  Eigen::Map<const Eigen::MatrixXd> min_area_matrix_tmp(
    min_area_matrix.data(), num_labels_, num_labels_);
  min_area_matrix_ = min_area_matrix_tmp.transpose();  // Eigen is column major by default

  Eigen::Map<const Eigen::MatrixXd> max_area_matrix_tmp(
    max_area_matrix.data(), num_labels_, num_labels_);
  max_area_matrix_ = max_area_matrix_tmp.transpose();  // Eigen is column major by default

  min_area_matrix_ = min_area_matrix_.unaryExpr(
    [](double v) { return std::isfinite(v) ? v : std::numeric_limits<double>::max(); });
  max_area_matrix_ = max_area_matrix_.unaryExpr(
    [](double v) { return std::isfinite(v) ? v : std::numeric_limits<double>::max(); });
}

void DetectionClassRemapper::mapClasses(autoware_perception_msgs::msg::DetectedObjects & msg)
{
  const RemapContext context{
    num_labels_, allow_remapping_by_area_matrix_, min_area_matrix_, max_area_matrix_};

  for (auto & object : msg.objects) {
    const float bev_area = object.shape.dimensions.x * object.shape.dimensions.y;

    for (auto & classification : object.classification) {
      classification.label = context.remapLabel(classification.label, bev_area);
    }
  }
}

}  // namespace autoware::bevfusion
