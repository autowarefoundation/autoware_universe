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

#ifndef AUTOWARE__PROXIMITY_HAZARD_OBJECT_CHECKER__PROXIMITY_HAZARD_OBJECT_CHECKER_HPP_
#define AUTOWARE__PROXIMITY_HAZARD_OBJECT_CHECKER__PROXIMITY_HAZARD_OBJECT_CHECKER_HPP_

#include <autoware_proximity_hazard_object_checker/autoware_proximity_hazard_object_checker_param.hpp>
#include <autoware_proximity_hazard_object_checker/msg/proximity_hazard_objects.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cstdint>
#include <optional>

namespace autoware::proximity_hazard_object_checker
{
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_proximity_hazard_object_checker::msg::ProximityHazardObjects;
using autoware_utils_geometry::LinearRing2d;
using autoware_utils_geometry::Polygon2d;

constexpr int g_num_sectors = 8;

class ProximityHazardObjectChecker
{
public:
  ProximityHazardObjectChecker(
    proximity_hazard_object::Params params, LinearRing2d vehicle_footprint);

  // Build the per-sector hazard message from a PredictedObjects input and the
  // transform from the input's frame to base_link.
  [[nodiscard]] ProximityHazardObjects process(
    const PredictedObjects & input,
    const geometry_msgs::msg::TransformStamped & to_base_link) const;

  // Maps a bearing in radians (any range; normalized internally) to a sector index
  // per the configured per-sector ranges. Returns nullopt if the bearing does not
  // fall in any configured sector (misconfiguration: gap or under-specified).
  [[nodiscard]] std::optional<uint8_t> bearing_to_sector(double bearing_rad) const;

  // Bounding-circle pre-filter helper. Returns the radius of the smallest circle
  // centered at the object-local origin that contains the shape.
  static double compute_circumradius(const autoware_perception_msgs::msg::Shape & shape);

  // Centroid of the vehicle footprint, used as the reference point for sector
  // bearing computation. Exposed for debug visualization.
  [[nodiscard]] const autoware_utils_geometry::Point2d & ego_center() const { return ego_center_; }

private:
  proximity_hazard_object::Params params_;
  LinearRing2d vehicle_footprint_;
  autoware_utils_geometry::Point2d ego_center_;  // Centroid of vehicle_footprint_
  double vehicle_circumradius_;  // Max distance from base_link origin to any footprint vertex.
  double max_detection_range_squared_;  // Cached: params_.max_detection_range_m^2.
  std::array<Polygon2d, g_num_sectors>
    sector_wedges_;  // Triangular wedge polygons use to clip object polygons per sector.
};

}  // namespace autoware::proximity_hazard_object_checker

#endif  // AUTOWARE__PROXIMITY_HAZARD_OBJECT_CHECKER__PROXIMITY_HAZARD_OBJECT_CHECKER_HPP_
