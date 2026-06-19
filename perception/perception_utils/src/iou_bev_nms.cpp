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

#include "perception_utils/iou_bev_nms.hpp"

#include <autoware/object_recognition_utils/geometry.hpp>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace perception_utils
{
namespace
{

using DetectedObject = autoware_perception_msgs::msg::DetectedObject;
using Label = autoware_perception_msgs::msg::ObjectClassification;

}  // namespace

void IouBevNms::setParameters(const IouBevNmsParams & params)
{
  if (!std::isfinite(params.search_distance_2d) || params.search_distance_2d < 0.0) {
    throw std::invalid_argument("search_distance_2d must be a finite non-negative value.");
  }
  if (
    !std::isfinite(params.iou_threshold) || params.iou_threshold < 0.0 ||
    params.iou_threshold > 1.0) {
    throw std::invalid_argument("iou_threshold must be a finite value between 0 and 1.");
  }

  params_ = params;
  search_distance_2d_sq_ = params.search_distance_2d * params.search_distance_2d;
}

bool IouBevNms::isTargetPairObject(
  const DetectedObject & object1, const DetectedObject & object2) const
{
  const auto label1 =
    autoware::object_recognition_utils::getHighestProbLabel(object1.classification);
  const auto label2 =
    autoware::object_recognition_utils::getHighestProbLabel(object2.classification);

  if (label1 != label2 && (label1 == Label::PEDESTRIAN || label2 == Label::PEDESTRIAN)) {
    return false;
  }

  const auto sqr_dist_2d = autoware_utils_geometry::calc_squared_distance2d(
    autoware::object_recognition_utils::getPose(object1),
    autoware::object_recognition_utils::getPose(object2));
  return sqr_dist_2d <= search_distance_2d_sq_;
}

std::vector<DetectedObject> IouBevNms::apply(
  const std::vector<DetectedObject> & input_objects, const bool sort) const
{
  std::vector<DetectedObject> ordered_objects = input_objects;
  if (sort) {
    std::stable_sort(
      ordered_objects.begin(), ordered_objects.end(), [](const auto & lhs, const auto & rhs) {
        return lhs.existence_probability > rhs.existence_probability;
      });
  }

  std::vector<DetectedObject> output_objects;
  output_objects.reserve(ordered_objects.size());
  for (std::size_t target_i = 0; target_i < ordered_objects.size(); ++target_i) {
    double max_iou = 0.0;
    for (std::size_t source_i = 0; source_i < target_i; ++source_i) {
      const auto & target_object = ordered_objects.at(target_i);
      const auto & source_object = ordered_objects.at(source_i);
      if (!isTargetPairObject(target_object, source_object)) {
        continue;
      }

      const double iou = autoware::object_recognition_utils::get2dIoU(target_object, source_object);
      max_iou = std::max(max_iou, iou);
      if (iou > params_.iou_threshold) {
        break;
      }
    }

    if (max_iou <= params_.iou_threshold) {
      output_objects.emplace_back(ordered_objects.at(target_i));
    }
  }

  return output_objects;
}

}  // namespace perception_utils
