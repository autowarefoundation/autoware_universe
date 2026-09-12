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
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace perception_utils
{
namespace
{

using DetectedObject = autoware_perception_msgs::msg::DetectedObject;
using Label = autoware_perception_msgs::msg::ObjectClassification;
using autoware_utils_geometry::Box2d;
using autoware_utils_geometry::Polygon2d;

struct CachedObject
{
  DetectedObject object;
  std::uint8_t label{};
  double x{};
  double y{};
  double area{};
  Polygon2d polygon;
  Box2d envelope;
};

CachedObject makeCachedObject(const DetectedObject & object)
{
  const auto & pose = object.kinematics.pose_with_covariance.pose;

  CachedObject cached;
  cached.object = object;
  cached.label = autoware::object_recognition_utils::getHighestProbLabel(object.classification);
  cached.x = pose.position.x;
  cached.y = pose.position.y;
  cached.polygon = autoware_utils_geometry::to_polygon2d(object);
  cached.area = boost::geometry::area(cached.polygon);
  boost::geometry::envelope(cached.polygon, cached.envelope);

  return cached;
}

bool isPairSubjectToNms(
  const CachedObject & object1, const CachedObject & object2, const double search_distance_2d_sq)
{
  if (
    object1.label != object2.label &&
    (object1.label == Label::PEDESTRIAN || object2.label == Label::PEDESTRIAN)) {
    return false;
  }

  const auto dx = object1.x - object2.x;
  const auto dy = object1.y - object2.y;
  return dx * dx + dy * dy <= search_distance_2d_sq;
}

double compute2dIoU(const CachedObject & source_object, const CachedObject & target_object)
{
  constexpr double kMinArea = 1.0e-6;
  constexpr double kMinUnionArea = 0.01;

  // These guards are not required for correctness, but can improve performance by avoiding
  // unnecessary calculations.
  if (source_object.area < kMinArea || target_object.area < kMinArea) {
    return 0.0;
  }
  if (boost::geometry::disjoint(source_object.envelope, target_object.envelope)) {
    return 0.0;
  }

  std::vector<Polygon2d> intersection_polygons;
  boost::geometry::intersection(
    source_object.polygon, target_object.polygon, intersection_polygons);

  double intersection_area = 0.0;
  for (const auto & polygon : intersection_polygons) {
    intersection_area += boost::geometry::area(polygon);
  }
  if (intersection_area < kMinArea) {
    return 0.0;
  }

  const double union_area = source_object.area + target_object.area - intersection_area;
  if (union_area < kMinUnionArea) {
    return 0.0;
  }

  return std::min(1.0, intersection_area / union_area);
}

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

std::vector<DetectedObject> IouBevNms::apply(
  const std::vector<DetectedObject> & input_objects, const bool sort) const
{
  std::vector<CachedObject> ordered_objects;
  ordered_objects.reserve(input_objects.size());
  for (const auto & object : input_objects) {
    ordered_objects.emplace_back(makeCachedObject(object));
  }
  if (sort) {
    std::stable_sort(
      ordered_objects.begin(), ordered_objects.end(), [](const auto & lhs, const auto & rhs) {
        return lhs.object.existence_probability > rhs.object.existence_probability;
      });
  }

  std::vector<DetectedObject> output_objects;
  output_objects.reserve(ordered_objects.size());
  for (std::size_t target_i = 0; target_i < ordered_objects.size(); ++target_i) {
    double max_iou = 0.0;
    for (std::size_t source_i = 0; source_i < target_i; ++source_i) {
      const auto & target_object = ordered_objects[target_i];
      const auto & source_object = ordered_objects[source_i];
      if (!isPairSubjectToNms(target_object, source_object, search_distance_2d_sq_)) {
        continue;
      }

      const double iou = compute2dIoU(target_object, source_object);
      max_iou = std::max(max_iou, iou);
      if (iou > params_.iou_threshold) {
        break;
      }
    }

    if (max_iou <= params_.iou_threshold) {
      output_objects.emplace_back(ordered_objects[target_i].object);
    }
  }

  return output_objects;
}

}  // namespace perception_utils
