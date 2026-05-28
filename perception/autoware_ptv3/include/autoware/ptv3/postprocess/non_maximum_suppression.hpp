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

#ifndef AUTOWARE__PTV3__POSTPROCESS__NON_MAXIMUM_SUPPRESSION_HPP_
#define AUTOWARE__PTV3__POSTPROCESS__NON_MAXIMUM_SUPPRESSION_HPP_

#include <Eigen/Eigen>

#include "autoware_perception_msgs/msg/detected_object.hpp"

#include <vector>

namespace autoware::ptv3
{

using autoware_perception_msgs::msg::DetectedObject;

/** @brief Parameters for BEV IoU NMS. */
struct NMSParams
{
  double search_distance_2d_{};
  double iou_threshold_{};
};

/**
 * @brief Apply BEV IoU NMS to detected objects.
 *
 * Objects are compared only when they are close enough. Pedestrians are kept separate from other
 * classes unless both objects are pedestrians.
 */
class NonMaximumSuppression
{
public:
  /**
   * @brief Load NMS thresholds.
   *
   * @param params Search distance and IoU threshold.
   */
  void set_parameters(const NMSParams & params);

  /**
   * @brief Return objects that survive NMS.
   *
   * @param input_objects Objects sorted by score before NMS.
   * @return Filtered object list in input order.
   */
  std::vector<DetectedObject> apply(const std::vector<DetectedObject> & input_objects);

private:
  [[nodiscard]] bool is_target_pair_object(
    const DetectedObject & object1, const DetectedObject & object2) const;
  Eigen::MatrixXd generate_iou_matrix(const std::vector<DetectedObject> & input_objects);

  NMSParams params_{};
  double search_distance_2d_sq_{};
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__POSTPROCESS__NON_MAXIMUM_SUPPRESSION_HPP_
