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

// cspell : ignore circumradius

#include "autoware/proximity_hazard_object_checker/proximity_hazard_object_checker.hpp"

#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_math/unit_conversion.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace autoware::proximity_hazard_object_checker
{
ProximityHazardObjectChecker::ProximityHazardObjectChecker(
  proximity_hazard_object::Params params, LinearRing2d vehicle_footprint)
: params_(std::move(params)),
  vehicle_footprint_(std::move(vehicle_footprint)),
  vehicle_circumradius_(0.0),
  max_detection_range_squared_(0.0)
{
  boost::geometry::centroid(vehicle_footprint_, ego_center_);

  max_detection_range_squared_ = params_.max_detection_range_m * params_.max_detection_range_m;

  double max_r2 = 0.0;
  for (const auto & p : vehicle_footprint_) {
    max_r2 = std::max(max_r2, p.x() * p.x() + p.y() * p.y());
  }
  vehicle_circumradius_ = std::sqrt(max_r2);

  constexpr double wedge_radius_offset = 1e-3;
  const double wedge_radius =
    vehicle_circumradius_ + params_.max_detection_range_m + wedge_radius_offset;

  const std::array<std::vector<double>, g_num_sectors> sector_refs_in_enum_order = {{
    {params_.sector_range.front},
    {params_.sector_range.front_right},
    {params_.sector_range.right},
    {params_.sector_range.rear_right},
    {params_.sector_range.rear},
    {params_.sector_range.rear_left},
    {params_.sector_range.left},
    {params_.sector_range.front_left},
  }};
  for (uint8_t s = 0; s < g_num_sectors; ++s) {
    const double a_from = autoware_utils_math::deg2rad(
      sector_refs_in_enum_order[s].front() - params_.sector_boundary_tolerance_deg);
    const double a_to = autoware_utils_math::deg2rad(
      sector_refs_in_enum_order[s].back() + params_.sector_boundary_tolerance_deg);
    boost::geometry::append(
      sector_wedges_[s], autoware_utils_geometry::Point2d{ego_center_.x(), ego_center_.y()});
    boost::geometry::append(
      sector_wedges_[s], autoware_utils_geometry::Point2d{
                           ego_center_.x() + wedge_radius * std::cos(a_from),
                           ego_center_.y() + wedge_radius * std::sin(a_from)});
    boost::geometry::append(
      sector_wedges_[s], autoware_utils_geometry::Point2d{
                           ego_center_.x() + wedge_radius * std::cos(a_to),
                           ego_center_.y() + wedge_radius * std::sin(a_to)});
    boost::geometry::append(
      sector_wedges_[s], autoware_utils_geometry::Point2d{ego_center_.x(), ego_center_.y()});
    boost::geometry::correct(sector_wedges_[s]);
  }
}

ProximityHazardObjects ProximityHazardObjectChecker::process(
  const PredictedObjects & input, const geometry_msgs::msg::TransformStamped & to_base_link) const
{
  ProximityHazardObjects out;
  out.header = input.header;

  std::array<double, g_num_sectors> closest_cd{};
  closest_cd.fill(std::numeric_limits<double>::infinity());

  for (const auto & object : input.objects) {
    if (!params_.include_unknown_objects) {
      const bool is_unknown =
        std::any_of(object.classification.begin(), object.classification.end(), [](const auto & c) {
          return c.label == autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
        });
      if (is_unknown) {
        continue;
      }
    }

    geometry_msgs::msg::Pose pose_in_base_link;
    tf2::doTransform(
      object.kinematics.initial_pose_with_covariance.pose, pose_in_base_link, to_base_link);

    const double object_radius = compute_circumradius(object.shape);
    const double center_d2 = pose_in_base_link.position.x * pose_in_base_link.position.x +
                             pose_in_base_link.position.y * pose_in_base_link.position.y;
    const double reject_radius =
      vehicle_circumradius_ + object_radius + params_.max_detection_range_m;
    if (center_d2 > reject_radius * reject_radius) {
      continue;
    }

    // The object polygon is assumed to have a closed topology (i.e., the first and last vertices
    // are connected, forming a closed loop).
    const auto object_polygon =
      autoware_utils_geometry::to_polygon2d(pose_in_base_link, object.shape);

    const double cd = boost::geometry::comparable_distance(vehicle_footprint_, object_polygon);
    if (cd > max_detection_range_squared_) {
      continue;
    }

    for (uint8_t s = 0; s < g_num_sectors; ++s) {
      boost::geometry::model::multi_polygon<Polygon2d> clipped;
      boost::geometry::intersection(object_polygon, sector_wedges_[s], clipped);
      if (clipped.empty()) {
        continue;
      }

      double sector_cd = std::numeric_limits<double>::infinity();
      for (const auto & part : clipped) {
        sector_cd =
          std::min(sector_cd, boost::geometry::comparable_distance(part, vehicle_footprint_));
      }
      if (sector_cd > max_detection_range_squared_) {
        continue;
      }

      if (sector_cd < closest_cd[s]) {
        closest_cd[s] = sector_cd;
        auto & slot = out.sectors[s];
        slot.has_object = true;
        slot.distance_m = static_cast<float>(std::sqrt(sector_cd));
        slot.predicted_object = object;
      }
    }
  }

  return out;
}

std::optional<uint8_t> ProximityHazardObjectChecker::bearing_to_sector(double bearing_rad) const
{
  double b = std::fmod(bearing_rad + M_PI, 2.0 * M_PI);
  if (b < 0.0) {
    b += 2.0 * M_PI;
  }
  b -= M_PI;

  auto sector = [](const auto & range) -> std::pair<double, double> {
    const auto start = autoware_utils_math::deg2rad(range.front());
    const auto end = autoware_utils_math::deg2rad(range.back());
    return std::make_pair(start, end);
  };

  auto in_range = [b](double start, double end) {
    return (end > start) ? (b >= start && b < end) : (b >= start || b < end);
  };

  const auto [front_start, front_end] = sector(params_.sector_range.front);
  if (in_range(front_start, front_end)) return ProximityHazardObjects::FRONT;

  const auto [front_left_start, front_left_end] = sector(params_.sector_range.front_left);
  if (in_range(front_left_start, front_left_end)) return ProximityHazardObjects::FRONT_LEFT;

  const auto [left_start, left_end] = sector(params_.sector_range.left);
  if (in_range(left_start, left_end)) return ProximityHazardObjects::LEFT;

  const auto [rear_left_start, rear_left_end] = sector(params_.sector_range.rear_left);
  if (in_range(rear_left_start, rear_left_end)) return ProximityHazardObjects::REAR_LEFT;

  const auto [rear_start, rear_end] = sector(params_.sector_range.rear);
  if (in_range(rear_start, rear_end)) return ProximityHazardObjects::REAR;

  const auto [rear_right_start, rear_right_end] = sector(params_.sector_range.rear_right);
  if (in_range(rear_right_start, rear_right_end)) return ProximityHazardObjects::REAR_RIGHT;

  const auto [right_start, right_end] = sector(params_.sector_range.right);
  if (in_range(right_start, right_end)) return ProximityHazardObjects::RIGHT;

  const auto [front_right_start, front_right_end] = sector(params_.sector_range.front_right);
  if (in_range(front_right_start, front_right_end)) return ProximityHazardObjects::FRONT_RIGHT;

  return std::nullopt;
}

double ProximityHazardObjectChecker::compute_circumradius(
  const autoware_perception_msgs::msg::Shape & shape)
{
  switch (shape.type) {
    case autoware_perception_msgs::msg::Shape::BOUNDING_BOX:
      return 0.5 * std::hypot(shape.dimensions.x, shape.dimensions.y);
    case autoware_perception_msgs::msg::Shape::CYLINDER:
      return 0.5 * shape.dimensions.x;
    case autoware_perception_msgs::msg::Shape::POLYGON: {
      double max_r2 = 0.0;
      for (const auto & p : shape.footprint.points) {
        max_r2 = std::max(max_r2, static_cast<double>(p.x * p.x + p.y * p.y));
      }
      return std::sqrt(max_r2);
    }
    default:
      return 0.0;
  }
}

}  // namespace autoware::proximity_hazard_object_checker
