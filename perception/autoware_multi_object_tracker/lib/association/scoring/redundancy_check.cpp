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

#include "autoware/multi_object_tracker/association/scoring/redundancy_check.hpp"

#include "autoware/multi_object_tracker/object_model/shapes.hpp"

#include <cmath>

namespace autoware::multi_object_tracker
{

double calcAdaptiveGIoUThreshold(
  const double object_speed, const double generalized_iou_threshold,
  const double static_object_speed, const double moving_object_speed,
  const double static_iou_threshold)
{
  // If the threshold is already larger than static threshold, just return it
  if (generalized_iou_threshold > static_iou_threshold) {
    return generalized_iou_threshold;
  }
  if (object_speed >= moving_object_speed) {
    return generalized_iou_threshold;
  }
  if (object_speed > static_object_speed) {
    // Linear interpolation between static and moving thresholds
    const double speed_ratio =
      (object_speed - static_object_speed) / (moving_object_speed - static_object_speed);
    return static_iou_threshold + speed_ratio * (generalized_iou_threshold - static_iou_threshold);
  }
  return static_iou_threshold;
}

bool isRedundant(
  const types::DynamicObject & source_object, const types::DynamicObject & target_object,
  const classes::Label source_label, const classes::Label target_label,
  const float source_known_prob, const float target_known_prob,
  const TrackerOverlapManagerConfig & config)
{
  constexpr double min_union_iou_area = 1e-2;
  constexpr float min_known_prob = 0.2;
  constexpr double min_valid_iou = 1e-6;
  constexpr double precision_threshold = 0.;
  constexpr double recall_threshold = 0.5;

  const auto generalized_iou_threshold_opt =
    get_map_value_if_exists(config.pruning_giou_thresholds, source_label);
  if (!generalized_iou_threshold_opt) {
    return false;
  }
  const double generalized_iou_threshold = generalized_iou_threshold_opt->get();

  const bool is_pedestrian =
    (source_label == classes::Label::PEDESTRIAN && target_label == classes::Label::PEDESTRIAN);
  const bool is_target_known = target_known_prob >= min_known_prob;
  const bool is_source_known = source_known_prob >= min_known_prob;

  if (is_pedestrian) {
    double iou = shapes::get1dIoU(source_object, target_object);
    if (iou < min_valid_iou) return false;
    return iou > config.min_known_object_removal_iou;
  } else if (is_target_known && is_source_known) {
    double iou = shapes::get2dIoU(source_object, target_object, min_union_iou_area);
    if (iou < min_valid_iou) return false;
    return iou > config.min_known_object_removal_iou;
  } else if (is_target_known || is_source_known) {
    // One object is unknown (typically the target)
    double precision = 0.0;
    double recall = 0.0;
    double generalized_iou = 0.0;
    if (!shapes::get2dPrecisionRecallGIoU(
          source_object, target_object, precision, recall, generalized_iou)) {
      return false;
    }
    // Adjust threshold based on known partner's speed and static/moving status
    const double known_object_speed =
      is_target_known ? std::hypot(target_object.twist.linear.x, target_object.twist.linear.y)
                      : std::hypot(source_object.twist.linear.x, source_object.twist.linear.y);
    const double adaptive_threshold = calcAdaptiveGIoUThreshold(
      known_object_speed, generalized_iou_threshold, config.pruning_static_object_speed,
      config.pruning_moving_object_speed, config.pruning_static_iou_threshold);
    return (
      precision > precision_threshold || recall > recall_threshold ||
      generalized_iou > adaptive_threshold);
  } else {
    // Both are unknown: use generalized IoU
    double iou = shapes::get2dGeneralizedIoU(source_object, target_object);
    return iou > generalized_iou_threshold;
  }
}

}  // namespace autoware::multi_object_tracker
