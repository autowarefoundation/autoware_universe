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

#include "autoware/ptv3/postprocess/non_maximum_suppression.hpp"

#include <autoware/object_recognition_utils/geometry.hpp>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <vector>

namespace autoware::ptv3
{

using Label = autoware_perception_msgs::msg::ObjectClassification;

void NonMaximumSuppression::set_parameters(const NMSParams & params)
{
  params_ = params;
  search_distance_2d_sq_ = params.search_distance_2d_ * params.search_distance_2d_;
}

bool NonMaximumSuppression::is_target_pair_object(
  const DetectedObject & object1, const DetectedObject & object2) const
{
  const auto label1 =
    autoware::object_recognition_utils::getHighestProbLabel(object1.classification);
  const auto label2 =
    autoware::object_recognition_utils::getHighestProbLabel(object2.classification);

  if (label1 != label2 && (label1 == Label::PEDESTRIAN || label2 == Label::PEDESTRIAN)) {
    return false;
  }

  const auto sqr_dist_2d = autoware_utils::calc_squared_distance2d(
    autoware::object_recognition_utils::getPose(object1),
    autoware::object_recognition_utils::getPose(object2));
  return sqr_dist_2d <= search_distance_2d_sq_;
}

Eigen::MatrixXd NonMaximumSuppression::generate_iou_matrix(
  const std::vector<DetectedObject> & input_objects)
{
  Eigen::MatrixXd triangular_matrix =
    Eigen::MatrixXd::Zero(input_objects.size(), input_objects.size());
  for (std::size_t target_i = 0; target_i < input_objects.size(); ++target_i) {
    for (std::size_t source_i = 0; source_i < target_i; ++source_i) {
      const auto & target_obj = input_objects.at(target_i);
      const auto & source_obj = input_objects.at(source_i);
      if (!is_target_pair_object(target_obj, source_obj)) {
        continue;
      }

      const double iou = autoware::object_recognition_utils::get2dIoU(target_obj, source_obj);
      triangular_matrix(target_i, source_i) = iou;
      if (iou > params_.iou_threshold_) {
        break;
      }
    }
  }

  return triangular_matrix;
}

std::vector<DetectedObject> NonMaximumSuppression::apply(
  const std::vector<DetectedObject> & input_objects)
{
  Eigen::MatrixXd iou_matrix = generate_iou_matrix(input_objects);

  std::vector<DetectedObject> output_objects;
  output_objects.reserve(input_objects.size());
  for (std::size_t i = 0; i < input_objects.size(); ++i) {
    if (iou_matrix.row(i).maxCoeff() <= params_.iou_threshold_) {
      output_objects.emplace_back(input_objects.at(i));
    }
  }

  return output_objects;
}

}  // namespace autoware::ptv3
