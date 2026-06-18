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

#ifndef AUTOWARE__PTV3__EXPERIMENTAL__SEMANTIC_LABEL_HPP_
#define AUTOWARE__PTV3__EXPERIMENTAL__SEMANTIC_LABEL_HPP_

#include <cstdint>

namespace autoware::ptv3::experimental
{
/// @brief Semantic labels for point cloud segmentation.
/// @details
/// Consolidated labels representing different point cloud categories:
/// - CAR, TRUCK, BUS, BICYCLE, PEDESTRIAN: Object classes
/// - HAZARD: Small hazardous objects (e.g., debris, barriers)
/// - GROUND: Drivable ground surface
/// - STRUCTURE: Non-drivable structures (e.g., buildings, walls)
/// - VEGETATION: Vegetation (trees, bushes)
/// - NOISE: Noise points and outliers
enum class SemanticLabel : std::uint8_t {
  CAR = 0,
  TRUCK = 1,
  BUS = 2,
  BICYCLE = 3,
  PEDESTRIAN = 4,
  HAZARD = 5,
  GROUND = 6,
  STRUCTURE = 7,
  VEGETATION = 8,
  NOISE = 9,
};

}  // namespace autoware::ptv3::experimental

#endif  // AUTOWARE__PTV3__EXPERIMENTAL__SEMANTIC_LABEL_HPP_
