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

#ifndef AUTOWARE__PTV3__POSTPROCESS__DETECTION_CLASS_REMAPPER_HPP_
#define AUTOWARE__PTV3__POSTPROCESS__DETECTION_CLASS_REMAPPER_HPP_

#include "autoware/ptv3/utils.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace autoware::ptv3
{

/**
 * @brief Remap detection labels using BEV area thresholds.
 *
 * Matrices are interpreted as source class by target class in parameter order.
 */
class DetectionClassRemapper
{
public:
  /**
   * @brief Load remapping matrices and validate them against detection class count.
   *
   * @param allow_remapping_by_area_matrix Row-major matrix with 0 or 1 values.
   * @param min_area_matrix Row-major minimum BEV area per source and target class.
   * @param max_area_matrix Row-major maximum BEV area per source and target class.
   * @param num_detection_classes Number of labels in detection3d.class_names.
   * @throws std::runtime_error if matrix sizes or allow values are invalid.
   */
  void setParameters(
    const std::vector<std::int64_t> & allow_remapping_by_area_matrix,
    const std::vector<double> & min_area_matrix, const std::vector<double> & max_area_matrix,
    std::size_t num_detection_classes);

  /**
   * @brief Apply the first matching area remap to each box.
   *
   * @param boxes Boxes to update in place.
   */
  void mapClasses(std::vector<Box3D> & boxes);

private:
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> allow_remapping_by_area_matrix_;
  Eigen::MatrixXd min_area_matrix_;
  Eigen::MatrixXd max_area_matrix_;
  int num_labels_{};
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__POSTPROCESS__DETECTION_CLASS_REMAPPER_HPP_
