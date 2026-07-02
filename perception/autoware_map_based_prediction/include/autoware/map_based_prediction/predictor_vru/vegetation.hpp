// Copyright 2026 TIER IV, inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__VEGETATION_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__VEGETATION_HPP_

#include "autoware/map_based_prediction/path_generator/path_generator.hpp"

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <vector>

namespace autoware::map_based_prediction
{

class VegetationModule
{
public:
  VegetationModule() = default;

  /// @pre lanelet_map_ptr is non-null when building from a map; nullptr clears the layer.
  void build_from_map(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr);

  /// Return the object's predicted paths trimmed where the footprint enters a vegetation area.
  [[nodiscard]] std::vector<PredictedPath> cut_paths_crossing_vegetation(
    const autoware_perception_msgs::msg::PredictedObject & predicted_object) const;

private:
  lanelet::LaneletMapUPtr vegetation_layer_{nullptr};
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__VEGETATION_HPP_
