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

#include "traffic_light_selector_utils.hpp"

#include <algorithm>
#include <vector>

namespace autoware::traffic_light
{
namespace utils
{

bool isInsideRoughRoi(const RegionOfInterest & detected_roi, const RegionOfInterest & rough_roi)
{
  const auto tl_x = detected_roi.x_offset;
  const auto tl_y = detected_roi.y_offset;
  const auto br_x = detected_roi.x_offset + detected_roi.width;
  const auto br_y = detected_roi.y_offset + detected_roi.height;

  bool is_tl_inside = rough_roi.x_offset <= tl_x && tl_x <= rough_roi.x_offset + rough_roi.width &&
                      rough_roi.y_offset <= tl_y && tl_y <= rough_roi.y_offset + rough_roi.height;
  if (!is_tl_inside) {
    return false;
  }

  bool is_br_inside = rough_roi.x_offset <= br_x && br_x <= rough_roi.x_offset + rough_roi.width &&
                      rough_roi.y_offset <= br_y && br_y <= rough_roi.y_offset + rough_roi.height;
  if (!is_br_inside) {
    return false;
  }

  return true;
}

void computeCenterOffset(
  const RegionOfInterest & source, const RegionOfInterest & target, int32_t & shift_x,
  int32_t & shift_y)
{
  const auto source_roi_center_x = static_cast<int32_t>(source.x_offset + source.width / 2);
  const auto source_roi_center_y = static_cast<int32_t>(source.y_offset + source.height / 2);
  const auto target_roi_center_x = static_cast<int32_t>(target.x_offset + target.width / 2);
  const auto target_roi_center_y = static_cast<int32_t>(target.y_offset + target.height / 2);
  shift_x = source_roi_center_x - target_roi_center_x;
  shift_y = source_roi_center_y - target_roi_center_y;
}

RegionOfInterest getShiftedRoi(
  const RegionOfInterest & input_ROI, const uint32_t & image_width, const uint32_t & image_height,
  const int32_t & shift_x, const int32_t & shift_y)
{
  RegionOfInterest shifted_ROI = input_ROI;
  const auto x_offset = static_cast<int32_t>(input_ROI.x_offset) + shift_x;
  const auto y_offset = static_cast<int32_t>(input_ROI.y_offset) + shift_y;
  if (
    x_offset < 0 || y_offset < 0 || x_offset + input_ROI.width > image_width ||
    y_offset + input_ROI.height > image_height) {
    shifted_ROI.x_offset = 0;
    shifted_ROI.y_offset = 0;
    shifted_ROI.width = 0;
    shifted_ROI.height = 0;
  } else {
    shifted_ROI.x_offset = static_cast<uint32_t>(x_offset);
    shifted_ROI.y_offset = static_cast<uint32_t>(y_offset);
  }

  return shifted_ROI;
}

double getIoU(const RegionOfInterest & bbox1, const RegionOfInterest & bbox2)
{
  const auto x1 = std::max(bbox1.x_offset, bbox2.x_offset);
  const auto x2 = std::min(bbox1.x_offset + bbox1.width, bbox2.x_offset + bbox2.width);
  const auto y1 = std::max(bbox1.y_offset, bbox2.y_offset);
  const auto y2 = std::min(bbox1.y_offset + bbox1.height, bbox2.y_offset + bbox2.height);

  const auto intersection_area = std::max(static_cast<int32_t>(x2) - static_cast<int32_t>(x1), 0) *
                                 std::max(static_cast<int32_t>(y2) - static_cast<int32_t>(y1), 0);
  const auto union_area = static_cast<int32_t>(bbox1.width * bbox1.height) +
                          static_cast<int32_t>(bbox2.width * bbox2.height) - intersection_area;

  if (union_area == 0) {
    return 0.0;
  }

  return static_cast<double>(intersection_area) / static_cast<double>(union_area);
}

double getGenIoU(const RegionOfInterest & bbox1, const RegionOfInterest & bbox2)
{
  const auto intersection_x1 = std::max(bbox1.x_offset, bbox2.x_offset);
  const auto intersection_x2 = std::min(bbox1.x_offset + bbox1.width, bbox2.x_offset + bbox2.width);
  const auto intersection_y1 = std::max(bbox1.y_offset, bbox2.y_offset);
  const auto intersection_y2 =
    std::min(bbox1.y_offset + bbox1.height, bbox2.y_offset + bbox2.height);

  const auto intersection_area =
    std::max(static_cast<int32_t>(intersection_x2) - static_cast<int32_t>(intersection_x1), 0) *
    std::max(static_cast<int32_t>(intersection_y2) - static_cast<int32_t>(intersection_y1), 0);
  const auto union_area = static_cast<int32_t>(bbox1.width * bbox1.height) +
                          static_cast<int32_t>(bbox2.width * bbox2.height) - intersection_area;

  const auto convex_x1 = std::min(bbox1.x_offset, bbox2.x_offset);
  const auto convex_x2 = std::max(bbox1.x_offset + bbox1.width, bbox2.x_offset + bbox2.width);
  const auto convex_y1 = std::min(bbox1.y_offset, bbox2.y_offset);
  const auto convex_y2 = std::max(bbox1.y_offset + bbox1.height, bbox2.y_offset + bbox2.height);

  const auto convex_area = (static_cast<int32_t>(convex_x2) - static_cast<int32_t>(convex_x1)) *
                           (static_cast<int32_t>(convex_y2) - static_cast<int32_t>(convex_y1));
  const auto convex_without_1_and_2_area = convex_area - union_area;

  double iou;
  if (union_area == 0) {
    iou = 0.0;
  } else {
    iou = static_cast<double>(intersection_area) / static_cast<double>(union_area);
  }

  double giou;
  if (convex_area == 0) {
    giou = iou;
  } else {
    giou =
      iou - static_cast<double>(convex_without_1_and_2_area) / static_cast<double>(convex_area);
  }

  return giou;
}

}  // namespace utils
}  // namespace autoware::traffic_light
