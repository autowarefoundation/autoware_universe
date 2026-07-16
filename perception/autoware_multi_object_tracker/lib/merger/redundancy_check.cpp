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

#include "autoware/multi_object_tracker/merger/detail/redundancy_check.hpp"

#include "autoware/multi_object_tracker/object_model/shapes_iou.hpp"

namespace autoware::multi_object_tracker::detail
{

bool isRedundant(
  const types::DynamicObject & source_object, const types::DynamicObject & target_object,
  const classes::Label source_label, const classes::Label target_label,
  const float source_known_prob, const float target_known_prob,
  const TrackerOverlapManagerConfig & config)
{
  constexpr float min_known_prob = 0.2;
  constexpr double min_valid_iou = 1e-6;
  constexpr double precision_threshold = 0.;
  constexpr double recall_threshold = 0.5;

  const bool is_pedestrian =
    (source_label == classes::Label::PEDESTRIAN && target_label == classes::Label::PEDESTRIAN);
  const bool is_target_known = target_known_prob >= min_known_prob;
  const bool is_source_known = source_known_prob >= min_known_prob;

  if (is_pedestrian) {
    double iou = shapes::get1dIoU(source_object, target_object);
    if (iou < min_valid_iou) return false;
    return iou > config.pedestrian_pair_min_iou;
  } else if (is_target_known && is_source_known) {
    // Both known. Plain IoU misses over-segmented fragments that carry a classification
    // (e.g. semantic-segmentation clusters): a small fragment inside a full-size object has
    // low IoU. Add a containment criterion limited to clearly smaller targets so two adjacent
    // full-size objects are not merged by containment alone.
    double precision = 0.0;
    double recall = 0.0;
    double generalized_iou = 0.0;
    if (!shapes::get2dPrecisionRecallGIoU(
          source_object, target_object, precision, recall, generalized_iou)) {
      return false;
    }
    if (precision < min_valid_iou || recall < min_valid_iou) return false;
    // IoU recovered from precision (I/src) and recall (I/tgt): U = I*(1/p + 1/r - 1)
    const double iou = 1.0 / (1.0 / precision + 1.0 / recall - 1.0);
    if (iou > config.known_pair_min_iou) return true;
    // Containment: target mostly covered by source, and target area (= source area * p/r)
    // less than half of source area
    constexpr double containment_recall_threshold = 0.5;
    constexpr double max_fragment_area_ratio = 0.5;
    return recall > containment_recall_threshold && precision < max_fragment_area_ratio * recall;
  } else if (is_target_known || is_source_known) {
    // One object is unknown (typically the target)
    double precision = 0.0;
    double recall = 0.0;
    double generalized_iou = 0.0;
    if (!shapes::get2dPrecisionRecallGIoU(
          source_object, target_object, precision, recall, generalized_iou)) {
      return false;
    }
    // Any overlap (precision > 0) or a majority-covered target marks the unknown as redundant.
    return precision > precision_threshold || recall > recall_threshold;
  } else {
    // Both are unknown: generalized IoU, with disjoint pairs (GIoU <= 0, reachable under a
    // negative threshold) merge-eligible only within the boundary-gap bound.
    double iou = shapes::get2dGeneralizedIoU(source_object, target_object);
    if (iou <= config.unknown_pair_min_giou) return false;
    return iou > 0.0 ||
           shapes::get2dMinimumGap(source_object, target_object) <= config.unknown_pair_max_gap;
  }
}

}  // namespace autoware::multi_object_tracker::detail
