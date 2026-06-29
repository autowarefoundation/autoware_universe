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
/**
 * @brief Semantic labels for point cloud segmentation.
 */
enum class SemanticLabel : std::uint8_t {
  CAR = 0,
  TRUCK = 1,
  BUS = 2,
  MOTORCYCLE = 3,
  BICYCLE = 4,
  PEDESTRIAN = 5,
  ANIMAL = 6,
  HAZARD = 7,
  GROUND = 8,       ///< Drivable ground surface.
  STRUCTURE = 9,    ///< Non-drivable structures, such as buildings and walls.
  VEGETATION = 10,  ///< Vegetation, such as trees and bushes.
  NOISE = 11,       ///< Noise points and outliers.
};

}  // namespace autoware::ptv3::experimental

#endif  // AUTOWARE__PTV3__EXPERIMENTAL__SEMANTIC_LABEL_HPP_
